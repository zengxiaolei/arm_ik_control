#!/usr/bin/env python3
import rospy
import numpy as np
from math import pi, acos, fabs
import xml.dom.minidom
from tf.transformations import translation_matrix, quaternion_matrix, concatenate_matrices, quaternion_from_matrix
from trac_ik_python.trac_ik import IK
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
import PyKDL as kdl
from kdl_parser_py.urdf import treeFromParam
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

# ===================== 全局配置 =====================
IK_BOX_TOL = {"bx": 0.0015, "by": 0.0015, "bz": 0.0015, "brx": 0.01, "bry": 0.01, "brz": 0.01}
DEFAULT_JOINT_DELTA = [3.0] * 6
TRAJ_BASE = {"min_dur": 0.1, "speed_scale": 1.0}
USE_DEFAULT_TARGET = True
# 格式：[(x,y,z), (qx,qy,qz,qw)]
DEFAULT_TARGET = [
    ([0.911, 1.841, 1.557], [-0.045, -0.223, 0.948, 0.224]),
    ([1.298, -0.201, 1.524], [-0.027, -0.043, 0.684, 0.728])
]

# ===================== 工具函数 =====================
def get_robot_urdf() -> str:
    return rospy.get_param("robot_description")

def calc_pose_error(fk_xyz, fk_quat, target_xyz, target_quat):
    tx, ty, tz = target_xyz
    tqx, tqy, tqz, tqw = target_quat
    fx, fy, fz = fk_xyz
    fqx, fqy, fqz, fqw = fk_quat
    pos_err = np.sqrt((fx - tx)**2 + (fy - ty)**2 + (fz - tz)**2)
    dot = np.clip(fqx * tqx + fqy * tqy + fqz * tqz + fqw * tqw, -1.0, 1.0)
    rot_deg = np.rad2deg(2.0 * acos(fabs(dot)))
    return pos_err, rot_deg

# ===================== 核心控制节点 =====================
class ArmIKControlNode:
    def __init__(self):
        # 私有参数
        self.joint_state_topic = rospy.get_param("~joint_state_topic", "/joint_states")
        self.base_link = rospy.get_param("~base_link", "base_arm")
        self.ee_link = rospy.get_param("~ee_link", "tool0")
        self.ik_timeout = rospy.get_param("~timeout", 0.05)
        self.ik_eps = rospy.get_param("~eps", 0.001)
        self.solve_type = rospy.get_param("~solve_type", "Speed")
        self.joint_delta_thresh = rospy.get_param("~joint_delta_threshold", DEFAULT_JOINT_DELTA)
        self.min_traj_dur = TRAJ_BASE["min_dur"]
        self.speed_scale = TRAJ_BASE["speed_scale"]
        self.arrive_tol = rospy.get_param("~arrive_tolerance", 0.01)

        # 运行缓存
        self.current_joint_msg = None
        self.current_target_pose = None
        self.joint_names = None
        self.target_joint_pos = None
        self.joint_limits = {}
        self.multi_pose_buffer = None

        # 求解器
        self.ik_solver = None
        self.fk_solver = None
        self.kdl_joint_array = None

        # Action客户端
        self.traj_action_client = actionlib.SimpleActionClient(
            "/position_trajectory_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction
        )
        rospy.loginfo("Waiting trajectory action server...")
        self.traj_action_client.wait_for_server()
        rospy.loginfo("Trajectory action connected")

        # 初始化流程
        self._wait_first_joint_state()
        self._load_joint_limits()
        self._align_delta_threshold()
        self._init_ik_solver()
        self._init_kdl_fk()
        self._create_subscribers()

        rospy.loginfo(f"Arm IK Ready | Joint num:{len(self.joint_names)}, tol:{self.arrive_tol:.4f}")
        if USE_DEFAULT_TARGET:
            # 统一调用通用多点入口
            self._solve_multi_points(DEFAULT_TARGET, source_name="DEFAULT_TARGET")

    def _wait_first_joint_state(self):
        msg = rospy.wait_for_message(self.joint_state_topic, JointState, timeout=10.0)
        self.joint_names = msg.name
        self.current_joint_msg = msg

    def _load_joint_limits(self):
        urdf_text = get_robot_urdf()
        robot = xml.dom.minidom.parseString(urdf_text).getElementsByTagName("robot")[0]
        limit_dict = {}
        for child in robot.childNodes:
            if child.nodeType == child.TEXT_NODE or child.localName != "joint":
                continue
            jt = child.getAttribute("type")
            if jt == "fixed":
                continue
            jn = child.getAttribute("name")
            limit_tag = child.getElementsByTagName("limit")
            if not limit_tag:
                limit_dict[jn] = {"min_position": -pi, "max_position": pi, "max_velocity": 1.0}
                continue
            lt = limit_tag[0]
            min_p = float(lt.getAttribute("lower")) if lt.hasAttribute("lower") else -pi
            max_p = float(lt.getAttribute("upper")) if lt.hasAttribute("upper") else -pi
            max_v = float(lt.getAttribute("velocity")) if lt.hasAttribute("velocity") else 1.0
            limit_dict[jn] = {"min_position": min_p, "max_position": max_p, "max_velocity": max_v}
        self.joint_limits = limit_dict

    def _align_delta_threshold(self):
        joint_num = len(self.joint_names)
        self.joint_delta_thresh = self.joint_delta_thresh[:joint_num]
        while len(self.joint_delta_thresh) < joint_num:
            self.joint_delta_thresh.append(DEFAULT_JOINT_DELTA[0])

    def _init_ik_solver(self):
        rospy.loginfo(f"Init TRAC-IK base:{self.base_link}, tip:{self.ee_link}")
        self.ik_solver = IK(base_link=self.base_link, tip_link=self.ee_link,
                            timeout=self.ik_timeout, epsilon=self.ik_eps, solve_type=self.solve_type)

    def _init_kdl_fk(self):
        ok, tree = treeFromParam("robot_description")
        if not ok:
            rospy.logerr("URDF parse failed, FK disabled")
            return
        chain = tree.getChain(self.base_link, self.ee_link)
        self.fk_solver = kdl.ChainFkSolverPos_recursive(chain)
        self.kdl_joint_array = kdl.JntArray(chain.getNrOfJoints())

    def _create_subscribers(self):
        self.sub_joint_state = rospy.Subscriber("/joint_states", JointState, self._cb_joint_state, queue_size=1)
        self.sub_target_pose = rospy.Subscriber("target_pose", PoseStamped, self._cb_single_pose, queue_size=1)
        self.sub_multi_poses = rospy.Subscriber("multi_target_poses", PoseArray, self._cb_pose_array, queue_size=1)

    # 回调
    def _cb_joint_state(self, msg):
        self.current_joint_msg = msg

    def _cb_single_pose(self, msg: PoseStamped):
        self.current_target_pose = msg.pose
        self.target_joint_pos = None
        self.solve_ik_single()

    def _cb_pose_array(self, msg: PoseArray):
        if len(msg.poses) == 0:
            rospy.logwarn("Empty PoseArray, skip plan")
            return
        # 统一格式转换为 [(xyz), (quat)]，复用通用多点求解
        target_list = []
        for pose in msg.poses:
            xyz = [pose.position.x, pose.position.y, pose.position.z]
            quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
            target_list.append((xyz, quat))
        rospy.loginfo(f"Receive PoseArray, total points:{len(target_list)}")
        self._solve_multi_points(target_list, source_name="PoseArray Topic")

    # 工具：获取当前关节种子
    def _get_seed_joints(self) -> list:
        if not self.current_joint_msg:
            try:
                self.current_joint_msg = rospy.wait_for_message("/joint_states", JointState, timeout=2.0)
            except rospy.ROSException:
                rospy.logwarn("No joint state, zero seed")
                return [0.0] * len(self.joint_names)
        name2pos = dict(zip(self.current_joint_msg.name, self.current_joint_msg.position))
        seed = []
        missing = []
        for j in self.joint_names:
            if j in name2pos:
                seed.append(name2pos[j])
            else:
                seed.append(0.0)
                missing.append(j)
        if missing:
            rospy.logwarn(f"Missing joints: {missing}")
        return seed

    # 工具：判断是否到达目标
    def _check_arrived(self, curr, target) -> bool:
        if target is None:
            return False
        for c, t in zip(curr, target):
            if fabs(c - t) > self.arrive_tol:
                return False
        return True

    # 工具：下发单段轨迹
    def _publish_single_traj(self, seed, ik_sol, calc_ms):
        traj = JointTrajectory()
        traj.header.stamp = rospy.Time.now()
        traj.joint_names = self.joint_names
        pt = JointTrajectoryPoint()
        dur_list = []
        for idx, jn in enumerate(traj.joint_names):
            t_pos = ik_sol[idx]
            c_pos = seed[idx]
            lim = self.joint_limits[jn]
            cmd_pos = np.clip(t_pos, lim["min_position"], lim["max_position"])
            delta = abs(cmd_pos - c_pos)
            seg_dur = max(delta / lim["max_velocity"], self.min_traj_dur)
            dur_list.append(seg_dur)
            pt.positions.append(cmd_pos)
        max_dur = max(dur_list) / self.speed_scale
        pt.time_from_start = rospy.Duration(max_dur)
        traj.points.append(pt)
        rospy.loginfo(f"Single point calc cost: {calc_ms:.2f} ms")
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = traj
        self.traj_action_client.send_goal(goal)
        self.traj_action_client.wait_for_result()
        res = self.traj_action_client.get_result()
        rospy.loginfo(f"Single finish err code: {res.error_code}")

    # 工具：下发多点连续轨迹（无手动velocities，控制器自动插值）
    def _publish_multi_traj(self, start_seed, sol_list, calc_ms):
        traj = JointTrajectory()
        traj.header.stamp = rospy.Time.now()
        traj.joint_names = self.joint_names
        seg_dur_list = []
        pos_clip_list = []
        curr_seed = start_seed

        # 预计算分段时长、限位裁剪点位
        for sol in sol_list:
            dur_buf = []
            clip_buf = []
            for idx, jn in enumerate(traj.joint_names):
                t_p = sol[idx]
                c_p = curr_seed[idx]
                lim = self.joint_limits[jn]
                cmd_p = np.clip(t_p, lim["min_position"], lim["max_position"])
                delta = abs(cmd_p - c_p)
                seg_dur = max(delta / lim["max_velocity"], self.min_traj_dur)
                dur_buf.append(seg_dur)
                clip_buf.append(cmd_p)
            seg_max = max(dur_buf) / self.speed_scale
            seg_dur_list.append(seg_max)
            pos_clip_list.append(clip_buf)
            curr_seed = sol

        total_time = 0.0
        total_pt = len(sol_list)
        for i in range(total_pt):
            pt = JointTrajectoryPoint()
            pt.positions = pos_clip_list[i]
            total_time += seg_dur_list[i]
            pt.time_from_start = rospy.Duration(total_time)
            traj.points.append(pt)
            rospy.loginfo(f"Waypoint {i} | t={total_time:.2f}s")

        rospy.loginfo(f"Multi calc done, points:{total_pt}, cost:{calc_ms:.2f} ms")
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = traj
        self.traj_action_client.send_goal(goal)
        self.traj_action_client.wait_for_result()
        res = self.traj_action_client.get_result()
        rospy.loginfo(f"Multi finish total time:{total_time:.2f}s, err:{res.error_code}")

    # 【核心统一多点入口】DEFAULT_TARGET / PoseArray 共用此函数，仅输入list格式不同
    def _solve_multi_points(self, target_list, source_name: str):
        t_start_sec = rospy.Time.now().to_sec()  # 直接转浮点秒数，修复Time减float报错
        rospy.loginfo(f"Start solve multi points from [{source_name}], total:{len(target_list)}")
        current_seed = self._get_seed_joints()
        all_sols = []

        for idx, (xyz, quat) in enumerate(target_list):
            tx, ty, tz = xyz
            rx, ry, rz, rw = quat
            sol = self.ik_solver.get_ik(
                current_seed, x=tx, y=ty, z=tz, rx=rx, ry=ry, rz=rz, rw=rw,
                bx=IK_BOX_TOL["bx"], by=IK_BOX_TOL["by"], bz=IK_BOX_TOL["bz"],
                brx=IK_BOX_TOL["brx"], bry=IK_BOX_TOL["bry"], brz=IK_BOX_TOL["brz"]
            )
            if sol is None:
                rospy.logerr(f"Point {idx} IK no solution, abort full trajectory")
                return
            # 关节步长校验
            over_step = []
            for j_idx in range(len(current_seed)):
                delta = fabs(sol[j_idx] - current_seed[j_idx])
                if delta > self.joint_delta_thresh[j_idx]:
                    over_step.append((self.joint_names[j_idx], delta, self.joint_delta_thresh[j_idx]))
            if over_step:
                rospy.logwarn(f"Point {idx} joint over limit, abort trajectory")
                for name, d, th in over_step:
                    rospy.logwarn(f"  {name} delta {d:.4f} > {th:.4f}")
                return
            all_sols.append(sol)
            current_seed = sol
            rospy.loginfo(f"Point {idx} IK solved")

        calc_ms = (rospy.Time.now().to_sec() - t_start_sec) * 1000
        self._publish_multi_traj(self._get_seed_joints(), all_sols, calc_ms)
        self.target_joint_pos = all_sols[-1]

    # 单点PoseStamped求解（原有逻辑保留）
    def solve_ik_single(self):
        t_start_sec = rospy.Time.now().to_sec()  # 修复Time浮点运算
        if self.current_target_pose is None:
            rospy.loginfo_throttle(5, "Wait target_pose")
            return False, None
        seed = self._get_seed_joints()
        if self._check_arrived(seed, self.target_joint_pos):
            rospy.loginfo_throttle(3, "Already reach target, skip")
            return True, None
        pose = self.current_target_pose
        tx, ty, tz = pose.position.x, pose.position.y, pose.position.z
        rx, ry, rz, rw = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
        ik_sol = self.ik_solver.get_ik(
            seed, x=tx, y=ty, z=tz, rx=rx, ry=ry, rz=rz, rw=rw,
            bx=IK_BOX_TOL["bx"], by=IK_BOX_TOL["by"], bz=IK_BOX_TOL["bz"],
            brx=IK_BOX_TOL["brx"], bry=IK_BOX_TOL["bry"], brz=IK_BOX_TOL["brz"]
        )
        if ik_sol is None:
            cost = (rospy.Time.now().to_sec() - t_start_sec) * 1000
            rospy.logwarn(f"IK fail, cost {cost:.2f} ms")
            return False, None
        self.target_joint_pos = list(ik_sol)
        # 步长校验
        over_step = []
        for idx in range(len(seed)):
            delta = fabs(ik_sol[idx] - seed[idx])
            if delta > self.joint_delta_thresh[idx]:
                over_step.append((self.joint_names[idx], delta, self.joint_delta_thresh[idx]))
        if over_step:
            rospy.logwarn("Joint step over limit:")
            for name, d, th in over_step:
                rospy.logwarn(f"  {name}: {d:.4f} > {th:.4f}")
            return False, None
        # FK误差校验
        if self.fk_solver and self.kdl_joint_array:
            for idx, val in enumerate(ik_sol):
                self.kdl_joint_array[idx] = val
            fk_frame = kdl.Frame()
            self.fk_solver.JntToCart(self.kdl_joint_array, fk_frame)
            fk_xyz = [fk_frame.p.x(), fk_frame.p.y(), fk_frame.p.z()]
            fk_q = fk_frame.M.GetQuaternion()
            pos_err, ang_err = calc_pose_error(fk_xyz, fk_q, [tx, ty, tz], [rx, ry, rz, rw])
            rospy.loginfo(f"FK error: {pos_err*1000:.3f} mm | {ang_err:.2f} deg")
        calc_ms = (rospy.Time.now().to_sec() - t_start_sec) * 1000
        self._publish_single_traj(seed, ik_sol, calc_ms)
        total_ms = (rospy.Time.now().to_sec() - t_start_sec) * 1000
        rospy.loginfo(f"Single full cost: {total_ms:.2f} ms")
        return True, ik_sol

def main():
    rospy.init_node("arm_ik_control", anonymous=False)
    node = ArmIKControlNode()
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node shutdown")
#!/usr/bin/env python3
# 必须放在所有import最顶部
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import numpy as np
import queue
import rospy
from control_msgs.msg import FollowJointTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectory
# 线程安全队列
data_queue = queue.Queue(maxsize=2)
# 全局只创建一次画布，复用窗口
fig = None
axes_list = None

def plot_trajectory(joint_names, points):
    global fig, axes_list
    num_joint = len(joint_names)
    if num_joint == 0 or len(points) == 0:
        rospy.logwarn("Empty trajectory data, skip plot")
        return
    # 解析轨迹数据
    time_arr = []
    pos_data = [[] for _ in range(num_joint)]
    for pt in points:
        t = pt.time_from_start.secs + pt.time_from_start.nsecs * 1e-9
        time_arr.append(t)
        for j in range(num_joint):
            pos_data[j].append(pt.positions[j])
    # 首次运行：创建画布
    if fig is None:
        fig, axes_list = plt.subplots(num_joint, 1, sharex=True, figsize=(11, 2.1 * num_joint))
        fig.suptitle("Joint Position - Time Curve", fontsize=14)
        if num_joint == 1:
            axes_list = [axes_list]
        plt.show(block=False)
    else:
        # 清空旧曲线与旧标注
        for ax in axes_list:
            ax.clear()
    # 绘制每条关节曲线 + 每个点标注坐标
    for j_idx, ax in enumerate(axes_list):
        t_list = time_arr
        p_list = pos_data[j_idx]
        # 绘制曲线+圆点
        ax.plot(t_list, p_list, 'o-', c="#1f77b4", lw=1.2, ms=2.5)
        
        # ========== 新增：遍历所有点，标注 (时间, 关节角度) ==========
        for xi, yi in zip(t_list, p_list):
            ax.annotate(
                f"({xi:.2f}, {yi:.3f})",  # 格式化：时间保留2位，角度3位小数
                xy=(xi, yi),
                xytext=(5, 5),           # 文字向右上偏移5像素，避免盖住圆点
                textcoords="offset points",
                fontsize=6,              # 小字防止密集遮挡
                bbox=dict(boxstyle="round,pad=0.2", facecolor="#fff980", alpha=0.6),  # 浅黄底色
                ha="left", va="bottom"
            )
        # 坐标轴设置
        ax.set_ylabel(f"{joint_names[j_idx]}\npos (rad)")
        ax.grid(True, alpha=0.3)
    axes_list[-1].set_xlabel("time_from_start [sec]")
    fig.tight_layout()
    # 强制刷新GUI
    fig.canvas.draw()
    fig.canvas.flush_events()

# 回调：仅入队，不绘图
def callback_action_goal(msg: FollowJointTrajectoryActionGoal):
    traj = msg.goal.trajectory
    rospy.loginfo("Receive FollowJointTrajectoryActionGoal, points: %d", len(traj.points))
    try:
        data_queue.put_nowait((traj.joint_names, traj.points))
    except queue.Full:
        rospy.logwarn("Queue full, drop old trajectory")

def callback_traj_command(msg: JointTrajectory):
    rospy.loginfo("Receive JointTrajectory command, points: %d", len(msg.points))
    try:
        data_queue.put_nowait((msg.joint_names, msg.points))
    except queue.Full:
        rospy.logwarn("Queue full, drop old trajectory")

def main():
    rospy.init_node("trajectory_plot_both_node")
    rospy.Subscriber(
        "/position_trajectory_controller/follow_joint_trajectory/goal",
        FollowJointTrajectoryActionGoal,
        callback_action_goal
    )
    rospy.Subscriber(
        "/position_trajectory_controller/command",
        JointTrajectory,
        callback_traj_command
    )
    rospy.loginfo("Trajectory plot node started, waiting trajectory...")
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        try:
            joint_names, points = data_queue.get(timeout=0.05)
            plot_trajectory(joint_names, points)
        except queue.Empty:
            pass
        if fig is not None:
            fig.canvas.flush_events()
        rate.sleep()

if __name__ == "__main__":
    main()
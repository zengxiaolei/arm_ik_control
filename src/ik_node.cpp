#include "arm_ik_control/ik_node.h"
#include <urdf_parser/urdf_parser.h>
#include <kdl_parser/kdl_parser.hpp>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_kdl.h>
#include <cmath>
#include <algorithm>
#include <stdexcept>
#include <utility>
#include <memory>

ArmIkNode::ArmIkNode(ros::NodeHandle &nh, ros::NodeHandle &pnh)
    : nh_(nh), pnh_(pnh),
      joint_state_topic_("/joint_states"),
      base_link_("base_arm"),
      tip_link_("tool0"),
      ik_timeout_(0.05),
      ik_eps_(0.001),
      ik_solve_type_("Speed"),
      joint_delta_thresh_(DEFAULT_JOINT_DELTA),
      arrive_tol_(0.01),
      ik_solver_(nullptr),
      fk_solver_(nullptr),
      joint_state_cache_(nullptr),
      totg_(nullptr),
      use_default_target_(false),
      collision_check_(false)
{
    pnh_.param<std::string>("joint_state_topic", joint_state_topic_, "/joint_states");
    pnh_.param<std::string>("base_link", base_link_, "base_arm");
    pnh_.param<std::string>("ee_link", tip_link_, "tool0");
    pnh_.param<double>("timeout", ik_timeout_, 0.05);
    pnh_.param<double>("eps", ik_eps_, 0.001);
    pnh_.param<std::string>("solve_type", ik_solve_type_, "Speed");
    pnh_.param<std::vector<double>>("joint_delta_threshold", joint_delta_thresh_, DEFAULT_JOINT_DELTA);
    pnh_.param<double>("arrive_tolerance", arrive_tol_, 0.01);
    pnh_.param<double>("traj_min_dur", traj_min_dur_, 0.1);
    pnh_.param<std::string>("arm_group_name", arm_group_name_, "single_dof_group");
    pnh_.param<double>("totg_path_tolerance", path_tolerance_, 0.1);
    pnh_.param<double>("totg_resample_dt", resample_dt_, 0.2);
    pnh_.param<double>("totg_min_angle_change", min_angle_change_, 0.1);
    ROS_INFO("TOTG config: path_tolerance=%.3f, resample_dt=%.3fs, min_angle_change=%.6frad, ", path_tolerance_, resample_dt_, min_angle_change_);
    pnh_.param<bool>("use_action", use_action_, true);

    pnh_.param<bool>("use_default_target", use_default_target_, false);
    pnh_.param<bool>("collision_check", collision_check_, false);

    pnh_.param<bool>("enable_ruckig_smooth", enable_ruckig_smooth_, true);
    pnh_.param<bool>("ruckig_mitigate_overshoot", ruckig_mitigate_overshoot_, false);
    pnh_.param<double>("ruckig_overshoot_thresh", ruckig_overshoot_thresh_, 0.005);

    traj_raw_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>(
        "arm_controller/command",
        1,
        false);

    TRAC_IK::SolveType stype = stringToSolveType(ik_solve_type_);
    ROS_INFO("IK solve type: %s", ik_solve_type_.c_str());

    ik_solver_.reset(new TRAC_IK::TRAC_IK(base_link_, tip_link_, "/robot_description", ik_timeout_, ik_eps_, stype));

    std::string action_server_name;
    pnh_.param<std::string>("action_server", action_server_name, "position_trajectory_controller/follow_joint_trajectory");
    traj_client_ = std::make_shared<TrajActionClient>(action_server_name);
    ROS_INFO("Waiting for trajectory action server [%s]...", action_server_name.c_str());
    traj_client_->waitForServer();
    ROS_INFO("Trajectory action server connected successfully");

    waitFirstJointState();
    loadUrdfAndLimits();
    alignDeltaThreshold();
    createSubscribers();
    loadCustomJointLimits();

    // bool computeTimeStamps(robot_trajectory::RobotTrajectory& trajectory,
    //                        const double max_velocity_scaling_factor = 1.0,
    //                        const double max_acceleration_scaling_factor = 1.0) const override

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model_ = robot_model_loader.getModel();
    arm_group_ = robot_model_->getJointModelGroup(arm_group_name_);
    totg_.reset(new trajectory_processing::TimeOptimalTrajectoryGeneration(
        path_tolerance_, resample_dt_, min_angle_change_));

    planning_scene_.reset(new planning_scene::PlanningScene(robot_model_));

    ROS_INFO("Arm IK Node Ready | Joint count:%zu, position tolerance:%.4f rad", joint_names_.size(), arrive_tol_);

    if (use_default_target_)
    {
        std::vector<double> temp_target_flat;
        pnh_.param<std::vector<double>>("default_target_points", temp_target_flat,
                                        {0.911, 1.841, 1.557, -0.045, -0.223, 0.948, 0.224,
                                         1.298, -0.201, 1.524, -0.027, -0.043, 0.684, 0.728,
                                         1.677, -0.647, 1.168, -0.037, -0.014, 0.749, 0.662});
        default_target_poses_.clear();
        for (size_t i = 0; i + 6 < temp_target_flat.size(); i += 7)
        {
            std::vector<double> point(temp_target_flat.begin() + i, temp_target_flat.begin() + i + 7);
            default_target_poses_.push_back(point);
        }
        ROS_INFO("Load default target points count: %zu", default_target_poses_.size());
        std::vector<TargetPoint> target_list;
        for (auto &pt : default_target_poses_)
        {
            std::vector<double> xyz{pt[0], pt[1], pt[2]};
            std::vector<double> q{pt[3], pt[4], pt[5], pt[6]};
            target_list.emplace_back(xyz, q);
        }
        solveMultiPoints(target_list, "ROS_PARAM_DEFAULT_TARGET");
    }
}

ArmIkNode::~ArmIkNode()
{
    ROS_INFO("Destructor: releasing TRAC-IK & MoveIt plugin resources");

    if (traj_client_)
    {
        traj_client_->cancelAllGoals();
        traj_client_.reset();
    }

    totg_.reset();
    ik_solver_.reset();
    fk_solver_.reset();
    joint_state_cache_.reset();
    arm_group_ = nullptr;
    robot_model_ = nullptr;
    ROS_INFO("All kinematics plugin resources released completely");
}

void ArmIkNode::spin()
{
    ros::spin();
}

void ArmIkNode::loadCustomJointLimits()
{
    std::map<std::string, double> temp_vel;
    pnh_.getParam("custom_vel", temp_vel);
    for (auto &pair : temp_vel)
    {
        custom_vel_limits_[pair.first] = pair.second;
        ROS_INFO("Custom vel limit [%s] = %.3f", pair.first.c_str(), pair.second);
    }

    std::map<std::string, double> temp_acc;
    pnh_.getParam("custom_acc", temp_acc);
    for (auto &pair : temp_acc)
    {
        custom_acc_limits_[pair.first] = pair.second;
        ROS_INFO("Custom acc limit [%s] = %.3f", pair.first.c_str(), pair.second);
    }

    std::map<std::string, double> temp_jerk;
    pnh_.getParam("custom_jerk", temp_jerk);
    for (auto &pair : temp_jerk)
    {
        custom_jerk_limits_[pair.first] = pair.second;
        ROS_INFO("Custom jerk limit [%s] = %.3f", pair.first.c_str(), pair.second);
    }

    ROS_INFO("Loaded custom joint limits from yaml completed");
}

void ArmIkNode::waitFirstJointState()
{
    ROS_INFO("Waiting for joint state topic: %s", joint_state_topic_.c_str());
    sensor_msgs::JointStateConstPtr msg = ros::topic::waitForMessage<sensor_msgs::JointState>(
        joint_state_topic_, ros::Duration(10.0));

    if (!msg)
    {
        ROS_FATAL("Timeout waiting for joint_state topic [%s]! Check topic name and joint state publisher.", joint_state_topic_.c_str());
        ros::shutdown();
        return;
    }

    joint_state_cache_ = msg;
    joint_names_ = joint_state_cache_->name;
}

void ArmIkNode::loadUrdfAndLimits()
{
    std::string urdf_str;
    if (!nh_.getParam("robot_description", urdf_str))
        throw std::runtime_error("Missing robot_description parameter on node handle");

    if (!urdf_model_.initString(urdf_str))
        throw std::runtime_error("URDF model parse failed, invalid robot_description");

    KDL::Tree tree;
    if (!kdl_parser::treeFromUrdfModel(urdf_model_, tree))
        throw std::runtime_error("Generate KDL tree from URDF failed");

    if (!tree.getChain(base_link_, tip_link_, kdl_chain_))
        throw std::runtime_error("Cannot extract KDL chain from " + base_link_ + " to " + tip_link_);

    fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
    jnt_array_ = KDL::JntArray(kdl_chain_.getNrOfJoints());

    for (auto &pair : urdf_model_.joints_)
    {
        auto j = pair.second;
        if (j->type == urdf::Joint::FIXED)
            continue;
        JointLimit lim;
        if (j->limits)
        {
            lim.min_pos = j->limits->lower;
            lim.max_pos = j->limits->upper;
            lim.max_vel = j->limits->velocity;
        }
        else
        {
            ROS_WARN("Joint [%s] has no position/velocity limits, using default [-π, π]", j->name.c_str());
            lim.min_pos = -M_PI;
            lim.max_pos = M_PI;
            lim.max_vel = 1.0;
        }
        joint_limits_[j->name] = lim;
    }
}

void ArmIkNode::alignDeltaThreshold()
{
    size_t j_num = joint_names_.size();
    joint_delta_thresh_.resize(j_num);
    for (size_t i = 0; i < j_num; i++)
        if (i >= joint_delta_thresh_.size())
            joint_delta_thresh_[i] = DEFAULT_JOINT_DELTA[0];
}

void ArmIkNode::createSubscribers()
{
    sub_joint_ = nh_.subscribe(joint_state_topic_, 1, &ArmIkNode::cbJointState, this);
    sub_single_ = nh_.subscribe("target_pose", 1, &ArmIkNode::cbSinglePose, this);
    sub_multi_ = nh_.subscribe("multi_target_poses", 1, &ArmIkNode::cbPoseArray, this);
}

void ArmIkNode::cbJointState(const sensor_msgs::JointState::ConstPtr &msg)
{
    joint_state_cache_ = msg;
}
void ArmIkNode::cbSinglePose(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    ROS_INFO("... ...");
    geometry_msgs::Pose use_pose;
    const std::string &frame_id = msg->header.frame_id;

    if (frame_id.empty() || frame_id == base_link_)
    {
        use_pose = msg->pose;
        ROS_INFO("[cbSinglePose] use target pose directly");
    }
    else if (frame_id == tip_link_)
    {
        use_pose = toolPoseToBase(msg->pose);
        ROS_INFO("[cbSinglePose] frame_id is tool0, converted target");
    }
    else
    {
        ROS_WARN("[cbSinglePose] Unsupported frame_id [%s]", frame_id.c_str());
        return;
    }

    single_target_pose_ = use_pose;
    last_target_joints_.clear();
    solveSingleIk();
}

void ArmIkNode::cbPoseArray(const geometry_msgs::PoseArray::ConstPtr &msg)
{
    ROS_INFO("... ...");
    if (msg->poses.empty())
    {
        ROS_WARN("Received empty PoseArray trajectory, skip planning");
        return;
    }
    std::vector<TargetPoint> list;
    for (auto &p : msg->poses)
    {
        std::vector<double> xyz{p.position.x, p.position.y, p.position.z};
        std::vector<double> q{p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w};
        list.emplace_back(xyz, q);
    }
    ROS_INFO("Receive multi-point trajectory, total points:%zu", list.size());
    solveMultiPoints(list, "PoseArray Topic");
}

std::vector<double> ArmIkNode::getSeedJoints()
{
    if (!joint_state_cache_)
    {
        try
        {
            joint_state_cache_ = ros::topic::waitForMessage<sensor_msgs::JointState>(joint_state_topic_, ros::Duration(2.0));
        }
        catch (...)
        {
            ROS_WARN("No joint state feedback available, use zero initial seed");
            return std::vector<double>(joint_names_.size(), 0.0);
        }
    }
    std::map<std::string, double> name2pos;
    for (size_t i = 0; i < joint_state_cache_->name.size(); i++)
        name2pos[joint_state_cache_->name[i]] = joint_state_cache_->position[i];

    std::vector<double> seed;
    std::vector<std::string> missing;
    for (auto &jn : joint_names_)
    {
        if (name2pos.count(jn))
            seed.push_back(name2pos[jn]);
        else
        {
            seed.push_back(0.0);
            missing.push_back(jn);
        }
    }
    if (!missing.empty())
    {
        std::string s;
        for (auto &m : missing)
            s += m + " ";
        ROS_WARN("Missing joint feedback: %s", s.c_str());
    }
    return seed;
}

bool ArmIkNode::checkArrived(const std::vector<double> &curr, const std::vector<double> &target)
{
    if (target.empty())
        return false;
    for (size_t i = 0; i < curr.size(); i++)
    {
        if (std::fabs(curr[i] - target[i]) > arrive_tol_)
            return false;
    }
    return true;
}

TargetPoint ArmIkNode::poseToTarget(const geometry_msgs::Pose &pose)
{
    std::vector<double> xyz{pose.position.x, pose.position.y, pose.position.z};
    std::vector<double> q{pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w};
    return {xyz, q};
}

void ArmIkNode::calcFkError(const std::vector<double> &joints, const geometry_msgs::Pose &target, double &pos_mm, double &rot_deg)
{
    if (!fk_solver_)
    {
        ROS_WARN("FK solver uninitialized, skip error calculation");
        pos_mm = 0.0;
        rot_deg = 0.0;
        return;
    }
    for (size_t i = 0; i < joints.size(); i++)
        jnt_array_(i) = joints[i];
    KDL::Frame frame;
    fk_solver_->JntToCart(jnt_array_, frame);
    geometry_msgs::Pose fk_pose;
    tf::poseKDLToMsg(frame, fk_pose);

    double dx = fk_pose.position.x - target.position.x;
    double dy = fk_pose.position.y - target.position.y;
    double dz = fk_pose.position.z - target.position.z;
    pos_mm = std::sqrt(dx * dx + dy * dy + dz * dz) * 1000.0;

    double dot = fk_pose.orientation.x * target.orientation.x + fk_pose.orientation.y * target.orientation.y + fk_pose.orientation.z * target.orientation.z + fk_pose.orientation.w * target.orientation.w;
    dot = std::max(-1.0, std::min(dot, 1.0));
    rot_deg = 2.0 * std::acos(std::fabs(dot)) * 180.0 / M_PI;
}

trajectory_msgs::JointTrajectory ArmIkNode::genSingleTraj(const std::vector<double> &seed, const std::vector<double> &sol)
{
    trajectory_msgs::JointTrajectory traj;
    traj.joint_names = joint_names_;
    trajectory_msgs::JointTrajectoryPoint pt;
    std::vector<double> durs;

    for (size_t i = 0; i < joint_names_.size(); i++)
    {
        auto &lim = joint_limits_[joint_names_[i]];
        double cmd = std::max(lim.min_pos, std::min(sol[i], lim.max_pos));
        double delta = std::fabs(cmd - seed[i]);
        double seg_dur = std::max(delta / lim.max_vel, traj_min_dur_);
        durs.push_back(seg_dur);
        pt.positions.push_back(cmd);
    }
    double max_dur = *std::max_element(durs.begin(), durs.end()) / TRAJ_SPEED_SCALE;
    pt.time_from_start = ros::Duration(max_dur);
    traj.points.push_back(pt);
    return traj;
}

trajectory_msgs::JointTrajectory ArmIkNode::genMultiTraj(const std::vector<double> &seed_start,
                                                         const std::vector<std::vector<double>> &sol_list,
                                                         std::vector<double> &seg_durs)
{
    trajectory_msgs::JointTrajectory traj;
    traj.joint_names = joint_names_;
    std::vector<std::vector<double>> clipped_list;
    std::vector<double> curr_seed = seed_start;

    for (auto &sol : sol_list)
    {
        std::vector<double> dur_buf, clip_buf;
        for (size_t i = 0; i < joint_names_.size(); i++)
        {
            auto &lim = joint_limits_[joint_names_[i]];
            double cmd = std::max(lim.min_pos, std::min(sol[i], lim.max_pos));
            double delta = std::fabs(cmd - curr_seed[i]);
            double axis_dur = std::max(delta / lim.max_vel, traj_min_dur_);
            dur_buf.push_back(axis_dur);
            clip_buf.push_back(cmd);
        }
        double seg_max = *std::max_element(dur_buf.begin(), dur_buf.end()) / TRAJ_SPEED_SCALE;
        seg_durs.push_back(seg_max);
        clipped_list.push_back(clip_buf);
        curr_seed = sol;
    }

    double total_t = 0.0;
    for (size_t i = 0; i < sol_list.size(); i++)
    {
        trajectory_msgs::JointTrajectoryPoint pt;
        pt.positions = clipped_list[i];
        total_t += seg_durs[i];
        pt.time_from_start = ros::Duration(total_t);
        traj.points.push_back(pt);
    }
    return traj;
}

trajectory_msgs::JointTrajectory ArmIkNode::genMultiTraj2(const std::vector<double> &seed_start,
                                                          const std::vector<std::vector<double>> &sol_list,
                                                          std::vector<double> &seg_durs)
{
    trajectory_msgs::JointTrajectory out_traj;
    // 1. 拼接完整路点：起始当前关节 + 所有IK求解点位
    std::vector<std::vector<double>> full_waypoints;
    full_waypoints.reserve(sol_list.size() + 1);
    full_waypoints.push_back(seed_start);
    full_waypoints.insert(full_waypoints.end(), sol_list.begin(), sol_list.end());

    // 2. 构建RobotTrajectory
    robot_trajectory::RobotTrajectory robot_traj(robot_model_, arm_group_);
    for (const std::vector<double> &joint_pos : full_waypoints)
    {
        moveit::core::RobotState state(robot_model_);
        state.setJointGroupPositions(arm_group_, joint_pos);
        // 初始占位时间填0，TOTG会重新完整计算时序
        robot_traj.addSuffixWayPoint(state, 0.0);
    }

    // 3. 使用无自定义限位的标准重载 computeTimeStamps(轨迹, 速度缩放, 加速度缩放)
    // 自动从robot_model读取URDF/joint_limits.yaml限速，官方标准用法
    // bool totg_success = totg_->computeTimeStamps(robot_traj);

    bool totg_success = totg_->computeTimeStamps(
        robot_traj,
        custom_vel_limits_,
        custom_acc_limits_,
        1.0,
        1.0);

    if (!totg_success)
    {
        ROS_ERROR("[genMultiTraj2] TOTG calculation failed, trajectory is not feasible");
        return trajectory_msgs::JointTrajectory();
    }

    // if (enable_ruckig_smooth_)
    // {

    //     bool smooth_ret = trajectory_processing::RuckigSmoothing::applySmoothing(
    //         robot_traj,
    //         custom_vel_limits_,
    //         custom_acc_limits_,
    //         custom_jerk_limits_,
    //         ruckig_mitigate_overshoot_,
    //         ruckig_overshoot_thresh_);
    //     if (!smooth_ret)
    //     {
    //         ROS_WARN("[genMultiTraj2] Ruckig smooth failed, fallback raw TOTG trajectory");
    //     }
    //     ROS_INFO("[genMultiTraj2] Ruckig jerk-limited smooth complete");
    // }

    if (collision_check_)
    {
        size_t traj_point_num = robot_traj.getWayPointCount();
        ROS_INFO("[genMultiTraj2] Start self collision check for all %zu trajectory sample points", traj_point_num);

        for (size_t i = 0; i < traj_point_num; ++i)
        {
            const moveit::core::RobotState &state = robot_traj.getWayPoint(i);
            std::vector<double> q;
            state.copyJointGroupPositions(arm_group_, q);
            if (checkSelfCollisionSingle(q))
            {
                ROS_ERROR("[genMultiTraj2] Trajectory sample point %zu self-collision detected, discard trajectory", i);
                return trajectory_msgs::JointTrajectory();
            }
        }
        ROS_INFO("[genMultiTraj2] All trajectory sample points pass self collision check");
    }

    // 4. RobotTrajectory 转 ROS 标准 JointTrajectory 消息
    moveit_msgs::RobotTrajectory moveit_traj_msg;
    robot_traj.getRobotTrajectoryMsg(moveit_traj_msg);
    out_traj = moveit_traj_msg.joint_trajectory;
    out_traj.joint_names = joint_names_;

    // 5. 计算每一段的时长，填充seg_durs
    seg_durs.clear();
    double prev_time = 0.0;
    for (const trajectory_msgs::JointTrajectoryPoint &pt : out_traj.points)
    {
        double curr_t = pt.time_from_start.toSec();
        seg_durs.push_back(curr_t - prev_time);
        prev_time = curr_t;
    }

    return out_traj;
}

void ArmIkNode::sendSingleTraj(const trajectory_msgs::JointTrajectory &traj)
{

    if (use_action_)
    {
        TrajGoal goal;
        goal.trajectory = traj;
        goal.trajectory.header.stamp = ros::Time::now();
        traj_client_->sendGoal(goal);
        traj_client_->waitForResult();
        auto res = traj_client_->getResult();
        auto state = traj_client_->getState();
        ROS_INFO("[SingleTraj] Action state:%s, error code:%d", state.toString().c_str(), res->error_code);
    }
    else
    {
        trajectory_msgs::JointTrajectory pub_traj = traj;
        pub_traj.header.stamp = ros::Time::now();
        traj_raw_pub_.publish(pub_traj);
        ROS_INFO("[SingleTraj RawPub] Direct publish to trajectory to command topic");
    }
}

void ArmIkNode::sendMultiTraj(const trajectory_msgs::JointTrajectory &traj)
{
    if (use_action_)
    {
        TrajGoal goal;
        goal.trajectory = traj;
        goal.trajectory.header.stamp = ros::Time::now();
        traj_client_->sendGoal(goal);
        traj_client_->waitForResult();
        auto res = traj_client_->getResult();
        auto state = traj_client_->getState();
        ROS_INFO("[MultiTraj] state:%s, error code:%d", state.toString().c_str(), res->error_code);
    }
    else
    {
        trajectory_msgs::JointTrajectory pub_traj = traj;
        pub_traj.header.stamp = ros::Time::now();
        traj_raw_pub_.publish(pub_traj);
        ROS_INFO("[MultiTraj RawPub] Direct publish multi-point trajectory to command topic");
    }
}

KDL::Frame ArmIkNode::poseToKdlFrame(double tx, double ty, double tz, double rx, double ry, double rz, double rw)
{
    tf::Quaternion quat(rx, ry, rz, rw);
    tf::Vector3 pos(tx, ty, tz);
    tf::Transform transform(quat, pos);
    KDL::Frame frame;
    tf::transformTFToKDL(transform, frame);
    return frame;
}

KDL::Twist ArmIkNode::createBoundsTwist()
{
    KDL::Twist bounds = KDL::Twist::Zero();
    bounds.vel.x(IK_BX);
    bounds.vel.y(IK_BY);
    bounds.vel.z(IK_BZ);
    bounds.rot.x(IK_BRX);
    bounds.rot.y(IK_BRY);
    bounds.rot.z(IK_BRZ);
    return bounds;
}

std::vector<double> ArmIkNode::kdlArrayToVector(const KDL::JntArray &arr)
{
    std::vector<double> vec(arr.data.size());
    for (size_t i = 0; i < arr.data.size(); i++)
    {
        vec[i] = arr.data[i];
    }
    return vec;
}

KDL::JntArray ArmIkNode::vectorToKdlArray(const std::vector<double> &vec)
{
    KDL::JntArray arr(vec.size());
    for (size_t i = 0; i < vec.size(); i++)
    {
        arr(i) = vec[i];
    }
    return arr;
}

TRAC_IK::SolveType ArmIkNode::stringToSolveType(const std::string &str)
{
    if (str == "Distance")
        return TRAC_IK::Distance;
    if (str == "Manip1")
        return TRAC_IK::Manip1;
    if (str == "Manip2")
        return TRAC_IK::Manip2;
    return TRAC_IK::Speed;
}

bool ArmIkNode::solveSingleIk()
{
    // double t_start = ros::Time::now().toSec();
    auto target_tp = poseToTarget(single_target_pose_);
    std::vector<double> seed = getSeedJoints();

    if (checkArrived(seed, last_target_joints_))
    {
        ROS_INFO_THROTTLE(3.0, "Already at target position, skip IK solve");
        return true;
    }

    double tx = target_tp.first[0], ty = target_tp.first[1], tz = target_tp.first[2];
    double rx = target_tp.second[0], ry = target_tp.second[1], rz = target_tp.second[2], rw = target_tp.second[3];

    KDL::JntArray seed_kdl = vectorToKdlArray(seed);
    KDL::JntArray sol_kdl(seed.size());
    KDL::Frame frame = poseToKdlFrame(tx, ty, tz, rx, ry, rz, rw);
    KDL::Twist bounds = createBoundsTwist();

    int ik_ret = -1;
    // int ik_ret = ik_solver_->CartToJnt(seed_kdl, frame, sol_kdl, bounds);

    for (int retry = 0; retry < 3; retry++)
    {
        ik_ret = ik_solver_->CartToJnt(seed_kdl, frame, sol_kdl, bounds);
        if (ik_ret >= 0)
            break;
    }

    if (ik_ret < 0)
    {
        ROS_WARN("[solveSingleIk] IK no valid solution");
        return false;
    }

    std::vector<double> sol = kdlArrayToVector(sol_kdl);
    last_target_joints_ = sol;

    bool over = false;
    for (size_t i = 0; i < seed.size(); i++)
    {
        double delta = std::fabs(sol[i] - seed[i]);
        if (delta > joint_delta_thresh_[i])
        {
            ROS_WARN("[solveSingleIk] Joint %s step delta=%.4f exceeds limit %.4f", joint_names_[i].c_str(), delta, joint_delta_thresh_[i]);
            over = true;
        }
    }
    if (over)
        return false;

    // double pos_err, rot_err;
    // calcFkError(sol, single_target_pose_, pos_err, rot_err);
    // ROS_INFO("[solveSingleIk] FK projection error: %.3f mm position, %.3f deg rotation", pos_err, rot_err);

    // double calc_ms = (ros::Time::now().toSec() - t_start) * 1000.0;
    auto traj = genSingleTraj(seed, sol);
    ROS_INFO("[solveSingleIk] Full pipeline finished");
    sendSingleTraj(traj);
    return true;
}

void ArmIkNode::solveMultiPoints(const std::vector<TargetPoint> &target_list, const std::string &src_name)
{
    double t_start = ros::Time::now().toSec();
    ROS_INFO("Start multi-point IK planning, source:%s, total waypoints:%zu", src_name.c_str(), target_list.size());
    std::vector<double> curr_seed = getSeedJoints();
    std::vector<std::vector<double>> all_sols;

    for (size_t idx = 0; idx < target_list.size(); idx++)
    {
        auto &tp = target_list[idx];
        double tx = tp.first[0], ty = tp.first[1], tz = tp.first[2];
        double rx = tp.second[0], ry = tp.second[1], rz = tp.second[2], rw = tp.second[3];

        KDL::JntArray seed_kdl = vectorToKdlArray(curr_seed);
        KDL::JntArray sol_kdl(curr_seed.size());
        KDL::Frame frame = poseToKdlFrame(tx, ty, tz, rx, ry, rz, rw);
        KDL::Twist bounds = createBoundsTwist();

        // int ret = ik_solver_->CartToJnt(seed_kdl, frame, sol_kdl, bounds);
        // if (ret < 0)
        // {
        //     ROS_ERROR("[solveMultiPoints] Waypoint %zu IK solve failed, abort full trajectory", idx);
        //     return;
        // }

        int ik_ret = -1;
        for (int retry = 0; retry < 3; retry++)
        {
            ik_ret = ik_solver_->CartToJnt(seed_kdl, frame, sol_kdl, bounds);
            if (ik_ret >= 0)
                break;
        }

        if (ik_ret < 0)
        {
            ROS_ERROR("[solveMultiPoints] Waypoint %zu IK solve failed, abort full trajectory", idx);
            return;
        }

        std::vector<double> sol = kdlArrayToVector(sol_kdl);
        bool over_limit = false;
        for (size_t j = 0; j < curr_seed.size(); j++)
        {
            double delta = std::fabs(sol[j] - curr_seed[j]);
            if (delta > joint_delta_thresh_[j])
            {
                ROS_WARN("[solveMultiPoints] Waypoint %zu joint %s delta %.4f exceeds limit", idx, joint_names_[j].c_str(), delta);
                over_limit = true;
            }
        }
        if (over_limit)
        {
            ROS_WARN("[solveMultiPoints] Waypoint %zu joint step overflow, abort trajectory", idx);
            return;
        }
        all_sols.push_back(sol);
        curr_seed = sol;
        // ROS_INFO("[solveMultiPoints] Waypoint %zu IK solved successfully", idx);
    }

    std::vector<double> seg_durs;
    // auto multi_traj = genMultiTraj(getSeedJoints(), all_sols, seg_durs);
    auto multi_traj = genMultiTraj2(getSeedJoints(), all_sols, seg_durs);
    double calc_ms = (ros::Time::now().toSec() - t_start) * 1000.0;
    sendMultiTraj(multi_traj);

    ROS_INFO("[solveMultiPoints] Multi-point IK and trajectory generation complete, compute cost:%.2f ms", calc_ms);
    double total_t = 0.0;
    for (size_t i = 0; i < seg_durs.size(); i++)
    {
        total_t += seg_durs[i];
        // ROS_INFO("[solveMultiPoints] Waypoint %zu time_from_start: %.3fs", i, total_t);
    }
    ROS_INFO("[solveMultiPoints] Total trajectory duration:%.2fs", total_t);
    last_target_joints_ = all_sols.back();
}

bool ArmIkNode::checkSelfCollisionSingle(const std::vector<double> &q)
{
    moveit::core::RobotState state(robot_model_);
    state.setJointGroupPositions(arm_group_name_, q);
    state.update();

    collision_detection::CollisionRequest req;
    collision_detection::CollisionResult res;
    req.group_name = arm_group_name_;

    planning_scene_->checkSelfCollision(req, res, state);
    return res.collision;
}
geometry_msgs::Pose ArmIkNode::toolPoseToBase(const geometry_msgs::Pose &pose_in_tool)
{
    if (!joint_state_cache_)
    {
        ROS_ERROR("[toolPoseToBase] joint_state cache empty, skip coordinate transform");
        return pose_in_tool;
    }

    size_t dof = kdl_chain_.getNrOfJoints();
    std::map<std::string, double> jnt_map;
    for (size_t i = 0; i < joint_state_cache_->name.size(); ++i)
    {
        jnt_map[joint_state_cache_->name[i]] = joint_state_cache_->position[i];
    }

    for (size_t i = 0; i < dof; ++i)
    {
        const std::string &jn = joint_names_[i];
        if (jnt_map.find(jn) != jnt_map.end())
        {
            jnt_array_(i) = jnt_map[jn];
        }
        else
        {
            ROS_WARN("[toolPoseToBase] joint [%s] missing in joint_states, fill 0 rad", jn.c_str());
            jnt_array_(i) = 0.0;
        }
    }

    KDL::Frame T_base_tool_kdl;
    int fk_ret = fk_solver_->JntToCart(jnt_array_, T_base_tool_kdl);
    if (fk_ret < 0)
    {
        ROS_ERROR("[toolPoseToBase] KDL JntToCart failed, ret code: %d", fk_ret);
        return pose_in_tool;
    }

    tf::Pose tf_pose_tool;
    tf::poseMsgToTF(pose_in_tool, tf_pose_tool);
    KDL::Frame T_tool_target_kdl;
    tf::transformTFToKDL(tf::Transform(tf_pose_tool), T_tool_target_kdl);

    KDL::Frame T_base_target_kdl = T_base_tool_kdl * T_tool_target_kdl;

    geometry_msgs::Pose out_pose;
    tf::poseKDLToMsg(T_base_target_kdl, out_pose);

    geometry_msgs::Pose fk_tool_pose;
    tf::poseKDLToMsg(T_base_tool_kdl, fk_tool_pose);
    // ROS_INFO("[FK Current tool0(base)] x=%.4f y=%.4f z=%.4f qx=%.3f qy=%.3f qz=%.3f qw=%.3f",
    //          fk_tool_pose.position.x, fk_tool_pose.position.y, fk_tool_pose.position.z,
    //          fk_tool_pose.orientation.x, fk_tool_pose.orientation.y, fk_tool_pose.orientation.z, fk_tool_pose.orientation.w);
    // ROS_INFO("[Input target(tool0)] x=%.4f y=%.4f z=%.4f qx=%.3f qy=%.3f qz=%.3f qw=%.3f",
    //          pose_in_tool.position.x, pose_in_tool.position.y, pose_in_tool.position.z,
    //          pose_in_tool.orientation.x, pose_in_tool.orientation.y, pose_in_tool.orientation.z, pose_in_tool.orientation.w);
    // ROS_INFO("[Output target(base)] x=%.4f y=%.4f z=%.4f qx=%.3f qy=%.3f qz=%.3f qw=%.3f",
    //          out_pose.position.x, out_pose.position.y, out_pose.position.z,
    //          out_pose.orientation.x, out_pose.orientation.y, out_pose.orientation.z, out_pose.orientation.w);

    return out_pose;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_ik_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    ArmIkNode node(nh, pnh);
    ros::spin();
    return 0;
}
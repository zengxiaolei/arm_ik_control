#ifndef IK_NODE_H
#define IK_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <control_msgs/QueryTrajectoryState.h>
#include <trac_ik/trac_ik.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <urdf/model.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_kdl.h> // 新增tf-kdl转换头文件
#include <string>
#include <vector>
#include <map>
#include <utility>
#include <memory>

#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/trajectory_processing/ruckig_traj_smoothing.h>

using TRAC_IK::SolveType;

using TrajActionClient = actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>;
using TrajClientPtr = std::shared_ptr<TrajActionClient>;
using TrajGoal = control_msgs::FollowJointTrajectoryGoal;
using ControllerState = control_msgs::JointTrajectoryControllerState;
using ControllerStateConstPtr = control_msgs::JointTrajectoryControllerStateConstPtr;

const double IK_BX = 0.0015, IK_BY = 0.0015, IK_BZ = 0.0015;
const double IK_BRX = 0.01, IK_BRY = 0.01, IK_BRZ = 0.01;
const std::vector<double> DEFAULT_JOINT_DELTA = {3.0, 3.0, 3.0, 3.0, 3.0, 3.0};

const double TRAJ_SPEED_SCALE = 1.0;

using TargetPoint = std::pair<std::vector<double>, std::vector<double>>;

struct JointLimit
{
    double min_pos;
    double max_pos;
    double max_vel;
};

class ArmIkNode
{
public:
    ArmIkNode(ros::NodeHandle &nh, ros::NodeHandle &pnh);
    ~ArmIkNode();
    void spin();

private:
    void waitFirstJointState();
    void loadUrdfAndLimits();
    void alignDeltaThreshold();
    void createSubscribers();
    void loadCustomJointLimits();

    void cbJointState(const sensor_msgs::JointState::ConstPtr &msg);
    void cbSinglePose(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void cbPoseArray(const geometry_msgs::PoseArray::ConstPtr &msg);

    std::vector<double> getSeedJoints();
    bool checkArrived(const std::vector<double> &curr, const std::vector<double> &target);
    TargetPoint poseToTarget(const geometry_msgs::Pose &pose);
    void calcFkError(const std::vector<double> &joints, const geometry_msgs::Pose &target, double &pos_mm, double &rot_deg);

    bool solveSingleIk();
    void solveMultiPoints(const std::vector<TargetPoint> &target_list, const std::string &src_name);

    trajectory_msgs::JointTrajectory genSingleTraj(const std::vector<double> &seed, const std::vector<double> &sol);

    trajectory_msgs::JointTrajectory genMultiTraj(const std::vector<double> &seed_start,
                                                  const std::vector<std::vector<double>> &sol_list,
                                                  std::vector<double> &seg_durs);

    trajectory_msgs::JointTrajectory genMultiTraj2(const std::vector<double> &seed_start,
                                                   const std::vector<std::vector<double>> &sol_list,
                                                   std::vector<double> &seg_durs);

    void sendSingleTraj(const trajectory_msgs::JointTrajectory &traj);
    void sendMultiTraj(const trajectory_msgs::JointTrajectory &traj);

    bool checkSelfCollisionSingle(const std::vector<double> &q);

    KDL::Frame poseToKdlFrame(double tx, double ty, double tz, double rx, double ry, double rz, double rw);
    KDL::Twist createBoundsTwist();
    std::vector<double> kdlArrayToVector(const KDL::JntArray &arr);
    KDL::JntArray vectorToKdlArray(const std::vector<double> &vec);
    TRAC_IK::SolveType stringToSolveType(const std::string &str);

    geometry_msgs::Pose toolPoseToBase(const geometry_msgs::Pose &pose_in_tool);

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber sub_joint_;
    ros::Subscriber sub_single_;
    ros::Subscriber sub_multi_;

    ros::Publisher traj_raw_pub_;
    bool use_action_;

    TrajClientPtr traj_client_;

    std::string joint_state_topic_;
    std::string base_link_;
    std::string tip_link_;
    double ik_timeout_;
    double ik_eps_;
    std::string ik_solve_type_;
    std::vector<double> joint_delta_thresh_;
    double arrive_tol_;
    double traj_min_dur_;

    std::unique_ptr<TRAC_IK::TRAC_IK> ik_solver_;
    KDL::Chain kdl_chain_;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
    KDL::JntArray jnt_array_;
    urdf::Model urdf_model_;

    sensor_msgs::JointState::ConstPtr joint_state_cache_;

    geometry_msgs::Pose single_target_pose_;
    std::vector<std::string> joint_names_;
    std::map<std::string, JointLimit> joint_limits_;
    std::vector<double> last_target_joints_;

    std::unordered_map<std::string, double> custom_vel_limits_;
    std::unordered_map<std::string, double> custom_acc_limits_;

    std::string arm_group_name_;
    double resample_dt_;
    double min_angle_change_;
    double path_tolerance_;
    moveit::core::RobotModelConstPtr robot_model_;
    const moveit::core::JointModelGroup *arm_group_;
    // trajectory_processing::TimeOptimalTrajectoryGeneration totg_;
    std::unique_ptr<trajectory_processing::TimeOptimalTrajectoryGeneration> totg_;

    bool use_default_target_;
    std::vector<std::vector<double>> default_target_poses_;

    bool collision_check_;
    std::shared_ptr<planning_scene::PlanningScene> planning_scene_;

    std::unordered_map<std::string, double> custom_jerk_limits_;
    bool ruckig_mitigate_overshoot_;
    double ruckig_overshoot_thresh_;
    
    bool enable_ruckig_smooth_;
};

#endif
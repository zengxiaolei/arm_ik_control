#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
import tf.transformations


# POSE_DATA = [
#     0.911, 1.841, 1.557, -0.045, -0.223, 0.948, 0.224,
#     1.298, -0.201, 1.524, -0.027, -0.043, 0.684, 0.728,
#     1.677, -0.647, 1.168, -0.037, -0.014, 0.749, 0.662
# ]
GROUP_NUM = 7
TOPIC_NAME = "multi_target_poses"
FRAME_ID = "base_arm"


# POSE_DATA = [
#     1.343, -0.193, 1.2, -0.027, -0.043, 0.684, 0.728
# ]
# FRAME_ID = "base_arm"

POSE_DATA = [
    0, 0.0, 0.01,       0, 0, 0, 1
]
FRAME_ID = "tool0"


# ===================== 发布逻辑 =====================
def main():
    rospy.init_node("goal_pub_switch_node")
    # 两个发布器同时创建
    pub_multi = rospy.Publisher(
        'multi_target_poses', PoseArray, queue_size=10, latch=True)
    pub_single = rospy.Publisher(
        'target_pose', PoseStamped, queue_size=10, latch=True)

    data_len = len(POSE_DATA)
    if data_len % GROUP_NUM != 0:
        rospy.logerr(
            f"Data length {data_len} cannot be divided by {GROUP_NUM}, invalid format!")
        return

    point_count = data_len // GROUP_NUM
    rospy.loginfo(f"Detected {point_count} target pose(s)")

    rospy.sleep(0.2)
    if point_count > 1:
        # 多个点位：发布 PoseArray 到 multi_target_poses
        msg_arr = PoseArray()
        msg_arr.header.frame_id = FRAME_ID
        msg_arr.header.stamp = rospy.Time.now()

        for i in range(point_count):
            seg = POSE_DATA[i * GROUP_NUM: (i + 1) * GROUP_NUM]
            x, y, z, qx, qy, qz, qw = seg
            p = Pose()
            p.position.x = x
            p.position.y = y
            p.position.z = z
            p.orientation.x = qx
            p.orientation.y = qy
            p.orientation.z = qz
            p.orientation.w = qw
            msg_arr.poses.append(p)
            rospy.loginfo(f"Point{i}: XYZ({x:.3f},{y:.3f},{z:.3f})")

        pub_multi.publish(msg_arr)
        rospy.loginfo(f"Published PoseArray to /multi_target_poses")
    else:
        # 仅1个点位：发布 PoseStamped 到 target_pose
        seg = POSE_DATA[0:GROUP_NUM]
        x, y, z, qx, qy, qz, qw = seg
        msg_stamp = PoseStamped()
        msg_stamp.header.frame_id = FRAME_ID
        msg_stamp.header.stamp = rospy.Time.now()

        msg_stamp.pose.position.x = x
        msg_stamp.pose.position.y = y
        msg_stamp.pose.position.z = z
        msg_stamp.pose.orientation.x = qx
        msg_stamp.pose.orientation.y = qy
        msg_stamp.pose.orientation.z = qz
        msg_stamp.pose.orientation.w = qw

        pub_single.publish(msg_stamp)
        rospy.loginfo(
            f"Published PoseStamped to /target_pose, XYZ({x:.3f},{y:.3f},{z:.3f})")

        while not rospy.is_shutdown():
            pub_single.publish(msg_stamp)
            rospy.loginfo(
                f"Published PoseStamped to /target_pose, XYZ({x:.3f},{y:.3f},{z:.3f})")

            rospy.sleep(0.1)  # 持续发布，防止丢失

    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("test publisher exit")

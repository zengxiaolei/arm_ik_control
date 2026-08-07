#!/usr/bin/env python3
import rospy
import sys
import tf
import numpy as np

from geometry_msgs.msg import PoseStamped, Pose


class Tool0Controller():
    def __init__(self):
        rospy.Subscriber('tool0_goal',
                         PoseStamped,
                         self.tool0_cb,
                         queue_size=1)
        

    def tool0_cb(self, msg):
        if msg.header.frame_id != "tool0":
            return

        ee_pose = self._move_group.get_current_pose().pose
        ee_pos, ee_quat = self.from_pose_msg_to_pos_quat(
            ee_pose)
        T1 = tf.TransformerROS().fromTranslationRotation(ee_pos, ee_quat)

        tool0_pos, tool0_quat = self.from_pose_msg_to_pos_quat(
            msg.pose)
        T2 = tf.TransformerROS().fromTranslationRotation(tool0_pos, tool0_quat)

        T = np.dot(T1, T2)
        trans_t = tf.transformations.translation_from_matrix(T)
        quat_t = tf.transformations.quaternion_from_matrix(T)

        pose_target = self.from_pos_quat_to_pose_msg(trans_t, quat_t)

    def from_pose_msg_to_pos_quat(self, pose_msg):
        pos = [pose_msg.position.x, pose_msg.position.y, pose_msg.position.z]
        quat = [pose_msg.orientation.x, pose_msg.orientation.y,
                pose_msg.orientation.z, pose_msg.orientation.w]
        return (pos, quat)


    def from_pos_quat_to_pose_msg(self, pos, quat):
        pose_msg = Pose()
        pose_msg.position.x, pose_msg.position.y, pose_msg.position.z = pos
        pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w = quat
        return pose_msg


def main():
    rospy.init_node('tool0_controller')
    Tool0Controller()
    try:
        rospy.spin()
    except:
        print("Shutting down...")


if __name__ == '__main__':
    main()

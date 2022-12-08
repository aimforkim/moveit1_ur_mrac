#!/usr/bin/env python3

from math import pi

import rospy
from geometry_msgs.msg import Point, Pose, Quaternion
from move_group_sequence.move_group_sequence import Circ, Lin, Ptp, Sequence, from_euler
from trajectory_tools.trajectory_handler import TrajectoryHandler
import tf.transformations
from typing import List


def pose_from_list(pose_list: List[float]) -> Pose:
    pose = Pose(
        position=Point(pose_list[0], pose_list[1], pose_list[2]), 
        orientation=Quaternion(pose_list[3], pose_list[4], pose_list[5], pose_list[6]))
    return pose
    
def robot_program():

    th = TrajectoryHandler()
    start = (0.0, -pi / 2.0, pi / 2.0, -pi, -pi / 2.0, 0.0)

    th.sequencer.plan(Ptp(goal=start))
    th.sequencer.execute()

    sequence = Sequence()

    pose_list = rospy.get_param('gh_poses')
    pose_goals = [pose_from_list(pose)for pose in pose_list]
    
    # th.publish_pose_array(pose_goals)
    th.publish_poses_as_pose_array(pose_goals)

    sequence.append(Ptp(goal=start))
    # sequence.append(
    #     Lin(
    #         goal=Pose(
    #             position=Point(0.253, 0.0, 0.1),
    #             orientation=Quaternion(1.0, 0.0, 0.0, 0.0),
    #         ),
    #         vel_scale=0.1,
    #         acc_scale=0.05,
    #     )

    for pose_goal in pose_goals:
        sequence.append(Ptp(goal=(pose_goal), vel_scale = 0.1, acc_scale = 0.1))
    
    
    sequence.append(Ptp(goal=(start), vel_scale=0.2, acc_scale=0.05))
    
    th.sequencer.plan(sequence)

    th.display_trajectory()

    th.sequencer.execute()


if __name__ == "__main__":

    robot_program()
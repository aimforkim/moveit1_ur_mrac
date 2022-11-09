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

def robot_program():

    th = TrajectoryHandler()

    th.sequencer.plan(Ptp(goal=th.start))
    th.sequencer.execute()

    sequence = Sequence()

        pose_list = rospy.get_param('gh_poses')
        pose_goals = [pose_from_list(pose)for pose in pose_list]
        print(pose_goals)

    sequence.append(Ptp(goal=th.start))

######th.start####
##### for i in pose_goals: 
    ###sequence.append(Lin((i),vel_scale = ?, acc_scale =?))
###(0.0, -1.0, 0.0, 0.0)##

    sequence.append(
        Lin(
            goal=Pose(
                position=Point(0.8, 0.4, 0.17),
                orientation=Quaternion(0.0, 1.0, 0.0, 0.0),
            ),
            vel_scale=0.1,
            acc_scale=0.05,
        ),
        blend_radius=0.01,
    )

    sequence.append(
        Lin(
            goal=Pose(
                position=Point(0.8, -0.4, 0.17),
                orientation=Quaternion(0.0, 1.0, 0.0, 0.0),
            ),
            vel_scale=0.1,
            acc_scale=0.05,
        )
    )

    sequence.append(Ptp(goal=(th.start), vel_scale=0.2, acc_scale=0.1))



    th.sequencer.plan(sequence)

    th.display_trajectory()

    th.sequencer.execute()


if __name__ == "__main__":

    if rospy.has_param('gh_poses'):
        pose_list = rospy.get_param('gh_poses')
        pose_goals = [pose_from_list(pose)for pose in pose_list]
        print(pose_goals)
    else:
        print("no poses")
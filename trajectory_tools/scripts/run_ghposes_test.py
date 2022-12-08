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


global ptp_vel
global ptp_acc
global scan_vel
global scan_acc

ptp_vel = 0.3
ptp_acc = 0.3
scan_vel = 0.1
scan_acc = 0.1

def robot_program():

    th = TrajectoryHandler()
    start = (0.0, -pi / 2.0, pi / 2.0, -pi, -pi / 2.0, 0.0)

    th.sequencer.plan(Ptp(goal=start))
    th.sequencer.execute()

    pose_list = rospy.get_param('gh_poses')
    pose_goals = [pose_from_list(pose)for pose in pose_list]
    
    # th.publish_pose_array(pose_goals)
    th.publish_poses_as_pose_array(pose_goals)

    th.sequencer.plan(Ptp(goal=start, vel_scale=ptp_vel, acc_scale=ptp_acc))
    th.sequencer.execute()
    
    th.sequencer.plan(Ptp(goal=pose_goals[0], vel_scale=ptp_vel, acc_scale= ptp_acc))
    th.sequencer.execute()

    th.sequencer.plan(Lin(goal=pose_goals[1], vel_scale=scan_vel, acc_scale=scan_acc))
    th.sequencer.execute()

    th.sequencer.plan(Ptp(goal=pose_goals[2], vel_scale=scan_vel, acc_scale=scan_acc))
    th.sequencer.execute()

    th.sequencer.plan(Ptp(goal=pose_goals[3], vel_scale=scan_vel, acc_scale=scan_acc))
    th.sequencer.execute()

    th.sequencer.plan(Lin(goal=pose_goals[4], vel_scale=scan_vel, acc_scale=scan_acc))
    th.sequencer.execute()

    th.sequencer.plan(Ptp(goal=pose_goals[5], vel_scale=scan_vel, acc_scale=scan_acc))
    th.sequencer.execute()

    th.sequencer.plan(Ptp(goal=pose_goals[6], vel_scale=scan_vel, acc_scale=scan_acc))
    th.sequencer.execute()

    th.sequencer.plan(Lin(goal=pose_goals[7], vel_scale=scan_vel, acc_scale=scan_acc))
    th.sequencer.execute()

    th.sequencer.plan(Ptp(goal=start, vel_scale=ptp_vel, acc_scale=ptp_acc))
    th.sequencer.execute()

    th.display_trajectory()


if __name__ == "__main__":

    robot_program()
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

if __name__ == "__main__":

    if rospy.has_param('gh_poses'):
        pose_list = rospy.get_param('gh_poses')
        #print(pose_list)
        pose_goals = [pose_from_list(pose)for pose in pose_list]
        print(pose_goals)
    else:
        print("no poses")

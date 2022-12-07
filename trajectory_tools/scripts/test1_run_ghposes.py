#!/usr/bin/env python3

from math import pi

import rospy
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from industrial_reconstruction_msgs.msg import NormalFilterParams
from industrial_reconstruction_msgs.srv import (StartReconstruction,
                                                StartReconstructionRequest,
                                                StopReconstruction,
                                                StopReconstructionRequest)
from move_group_sequence.move_group_sequence import (Circ, Lin, Ptp, Sequence,
                                                     from_euler)
from trajectory_tools.trajectory_handler import TrajectoryHandler, poses_from_yaml
from typing import List

def pose_from_list(pose_list: List[float]) -> Pose:
    pose = Pose(
        position=Point(-(pose_list[0]), pose_list[1], pose_list[2]), 
        orientation=Quaternion(pose_list[3], pose_list[4], pose_list[5], pose_list[6]))
    return pose
    
def robot_program():

    th = TrajectoryHandler()
    start = (0.0, -pi / 2.0, pi / 2.0, -pi, -pi / 2.0, 0.0)

    th.sequencer.plan(Ptp(goal=start))
    th.sequencer.execute()

    sequence = Sequence()

    # create pose mgs list form yaml
    poses = poses_from_yaml("/dev_ws/src/trajectory_tools/yaml/scan_path.yaml")

    # publish the poses to rviz for preview
    # th.publish_poses_as_pose_array(poses)
    th.publish_marker_array(poses)

  # Move into position to start reconstruction
    th.sequencer.plan(Ptp(goal=start, vel_scale=0.3, acc_scale=0.3))
    th.sequencer.execute()

    pose1 = poses[0]
    th.sequencer.plan(Ptp(goal=pose1, vel_scale=0.3, acc_scale=0.3))
    th.sequencer.execute()

    for pose_goal in poses[1:]:
        th.sequencer.plan(Lin(goal=(pose_goal), vel_scale = 0.1, acc_scale = 0.05))
        th.sequencer.execute()

    
    sequence.append(Ptp(goal=(start), vel_scale=0.2, acc_scale=0.1))

    th.sequencer.plan(sequence)

    th.display_trajectory()

    th.sequencer.execute()


if __name__ == "__main__":

    robot_program()
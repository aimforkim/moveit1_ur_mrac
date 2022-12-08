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
from yaml import safe_load

def robot_program():

    th = TrajectoryHandler()
    start = (0.0, -pi / 2.0, pi / 2.0, -pi, -pi / 2.0, 0.0)

    th.sequencer.plan(Ptp(goal=start))
    th.sequencer.execute()


    # create pose mgs list form yaml
    poses = poses_from_yaml("/dev_ws/src/trajectory_tools/yaml/scan_path.yaml")

    #define speed
    ptp_vel = 0.3
    ptp_acc = 0.3
    scan_vel = 0.1
    scan_acc = 0.1

    th.publish_poses_as_pose_array(poses)
  # Move into position to start reconstruction
    th.sequencer.plan(Ptp(goal=start, vel_scale=ptp_vel, acc_scale=ptp_acc))
    th.sequencer.execute()
    
    th.sequencer.plan(Ptp(goal=poses[0], vel_scale=ptp_vel, acc_scale= ptp_acc))
    th.sequencer.execute()

    th.sequencer.plan(Lin(goal=poses[1], vel_scale=scan_vel, acc_scale=scan_acc))
    th.sequencer.execute()

    th.sequencer.plan(Ptp(goal=poses[2], vel_scale=scan_vel, acc_scale=scan_acc))
    th.sequencer.execute()

    th.sequencer.plan(Lin(goal=poses[3], vel_scale=scan_vel, acc_scale=scan_acc))
    th.sequencer.execute()

    th.sequencer.plan(Ptp(goal=poses[4], vel_scale=scan_vel, acc_scale=scan_acc))
    th.sequencer.execute()

    th.sequencer.plan(Lin(goal=poses[5], vel_scale=scan_vel, acc_scale=scan_acc))
    th.sequencer.execute()
    # for pose_goal in poses[2:]:
    #     th.sequencer.plan(Ptp(goal=(pose_goal), vel_scale = 0.1, acc_scale = 0.1))
    #     th.sequencer.execute()
    
    th.sequencer.plan(Ptp(goal=(start), vel_scale=ptp_vel, acc_scale=ptp_acc))
    th.sequencer.execute() 

    th.display_trajectory()



if __name__ == "__main__":

    robot_program()
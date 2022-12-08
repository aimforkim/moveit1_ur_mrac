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
from trajectory_tools.trajectory_handler import TrajectoryHandler
from typing import List

# reconstruction parameters
start_srv_req = StartReconstructionRequest()
start_srv_req.tracking_frame = "tool0"
start_srv_req.relative_frame = "base_link"
start_srv_req.translation_distance = 0.0
start_srv_req.rotational_distance = 0.0
start_srv_req.live = False
start_srv_req.tsdf_params.voxel_length = 0.002
start_srv_req.tsdf_params.sdf_trunc = 0.004
start_srv_req.tsdf_params.min_box_values = Vector3(x=0.0, y=0.0, z=0.0)
start_srv_req.tsdf_params.max_box_values = Vector3(x=0.0, y=0.0, z=0.0)
start_srv_req.rgbd_params.depth_scale = 1000
start_srv_req.rgbd_params.depth_trunc = 0.25
start_srv_req.rgbd_params.convert_rgb_to_intensity = False

stop_srv_req = StopReconstructionRequest()
# stop_srv_req.archive_directory = '/dev_ws/src.reconstruction/'
stop_srv_req.mesh_filepath = "/home/aims/test_log_flat.ply"
# stop_srv_req.normal_filters = [NormalFilterParams(
#                     normal_direction=Vector3(x=0.0, y=0.0, z=1.0), angle=90)]
# stop_srv_req.min_num_faces = 1000

def pose_from_list(pose_list: List[float]) -> Pose:
    pose = Pose(
        position=Point(pose_list[0], pose_list[1], pose_list[2]), 
        orientation=Quaternion(pose_list[3], pose_list[4], pose_list[5], pose_list[6]))
    return pose

def robot_program():

    ee_name = "D405"
    th = TrajectoryHandler()

    rospy.wait_for_service("/start_reconstruction")
    rospy.loginfo("robot program: waiting for /start_reconstruction srv")
    start_recon = rospy.ServiceProxy("/start_reconstruction", StartReconstruction)
    stop_recon = rospy.ServiceProxy("/stop_reconstruction", StopReconstruction)


    start = (0.0, -pi / 2.0, pi / 2.0, -pi, -pi / 2.0, 0.0)
    pose_list = rospy.get_param('gh_poses')
    pose_goals = [pose_from_list(pose)for pose in pose_list]
    
    # th.publish_pose_array(pose_goals)
    th.publish_poses_as_pose_array(pose_goals)

    # attach camera and set new tcp
    th.attach_camera(ee_name)
    th.move_group.set_end_effector_link(f"{ee_name}/tcp")
    rospy.loginfo(
        f"{th.name}: end effector link set to {th.move_group.get_end_effector_link()}"
    )

    # Move into position to start reconstruction
    th.sequencer.plan(Ptp(goal=start, vel_scale=0.3, acc_scale=0.3))
    th.sequencer.execute()

    pose1 = pose_goals[0]
    th.sequencer.plan(Ptp(goal=pose1, vel_scale=0.3, acc_scale=0.3))
    th.sequencer.execute()

    # Start reconstruction with service srv_req
    resp = start_recon(start_srv_req)
    if resp:
        rospy.loginfo("robot program: reconstruction started successfully")
    else:
        rospy.loginfo("robot program: failed to start reconstruction")
    
    for pose_goal in pose_goals[1:]:
        th.sequencer.plan(Ptp(goal=(pose_goal), vel_scale = 0.1, acc_scale = 0.1))
        th.sequencer.execute()

    # th.sequencer.plan(Lin(goal=pose_goals[1],vel_scale=0.1, acc_scale=0.1))
    # th.sequencer.execute()
    # th.sequencer.plan(Ptp(goal=pose_goals[2],vel_scale=0.1, acc_scale=0.1))
    # th.sequencer.execute()

    # th.sequencer.plan(Ptp(goal=pose_goals[3],vel_scale=0.1, acc_scale=0.1))
    # th.sequencer.execute()
    # th.sequencer.plan(Lin(goal=pose_goals[4],vel_scale=0.1, acc_scale=0.1))
    # th.sequencer.execute()
    
    # Stop reconstruction with service srv_req
    resp = stop_recon(stop_srv_req)

    th.sequencer.plan(Ptp(goal=start, vel_scale=0.3, acc_scale=0.3))
    th.sequencer.execute()

    if resp:
        rospy.loginfo("robot program: reconstruction stopped successfully")
    else:
        rospy.loginfo("robot program: failed to stop reconstruction")


if __name__ == "__main__":

    robot_program()

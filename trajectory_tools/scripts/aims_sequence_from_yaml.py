#!/usr/bin/env python3
from trajectory_tools.trajectory_handler import TrajectoryHandler, poses_from_yaml
from move_group_sequence.move_group_sequence import Circ, Lin, Ptp, Sequence, from_euler


def robot_program():

    th = TrajectoryHandler()
    sequence = Sequence()

    sequence.append(Ptp(goal=th.start))

    # create pose mgs list form yaml
    poses = poses_from_yaml("/dev_ws/src/trajectory_tools/yaml/scan_path.yaml")

    # publish the poses to rviz for preview
    # th.publish_poses_as_pose_array(poses)
    th.publish_marker_array(poses)

    for p in poses:
        sequence.append(Lin(goal=p))

    sequence.append(Ptp(goal=th.start))
    th.sequencer.plan(sequence)

    th.display_trajectory()
    th.sequencer.execute()


if __name__ == "__main__":

    robot_program()

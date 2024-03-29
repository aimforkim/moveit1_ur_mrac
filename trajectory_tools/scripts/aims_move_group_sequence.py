#!/usr/bin/env python3

from math import pi

import rospy
from geometry_msgs.msg import Point, Pose, Quaternion
from move_group_sequence.move_group_sequence import Circ, Lin, Ptp, Sequence, from_euler
from trajectory_tools.trajectory_handler import TrajectoryHandler
import tf.transformations

def robot_program():

    th = TrajectoryHandler()

    th.sequencer.plan(Ptp(goal=th.start))
    th.sequencer.execute()

    sequence = Sequence()

    sequence.append(Ptp(goal=th.start))
    sequence.append(
        Ptp(
            goal=Pose(
                position=Point(0.6, 0.4, 0.6),
                orientation=Quaternion(0.0, 1.0, 0.0, 0.0),
            ),
            vel_scale=0.2,
            acc_scale=0.1,
        )
    )
###(0.0, -1.0, 0.0, 0.0)##

    sequence.append(
        Lin(
            goal=Pose(
                position=Point(0.75, 0.2, 0.18),
                orientation=Quaternion(0.0, 1.0, 0.0, 0.0),
            ),
            vel_scale=0.15,
            acc_scale=0.05,
        ),
        blend_radius=0.01,
    )

    sequence.append(
        Lin(
            goal=Pose(
                position=Point(0.75, -0.2, 0.18),
                orientation=Quaternion(0.0, 1.0, 0.0, 0.0),
            ),
            vel_scale=0.15,
            acc_scale=0.05,
        )
    )

    sequence.append(Ptp(goal=(th.start), vel_scale=0.2, acc_scale=0.1))

    # sequence.append(
    #     Circ(
    #         goal=Pose(
    #             position=Point(0.4, 0.0, 0.9),
    #             orientation=Quaternion(0.5, 0.5, 0.5, 0.5),
    #         ),
    #         center=Point(0.4, 0.0, 0.6),
    #     ),
    #     blend_radius=0.01,
    # )

    # sequence.append(
    #     Circ(
    #         goal=Pose(
    #             position=Point(0.4, 0.3, 0.6),
    #             orientation=Quaternion(0.5, 0.5, 0.5, 0.5),
    #         ),
    #         center=Point(0.4, 0.0, 0.6),
    #     ),
    #     blend_radius=0.01,
    # )

    # sequence.append(
    #     Lin(
    #         goal=Pose(
    #             position=Point(0.4, 0.0, 0.6),
    #             orientation=Quaternion(0.5, 0.5, 0.5, 0.5),
    #         ),
    #         vel_scale=0.1,
    #         acc_scale=0.05,
    #     )
    # )

    th.sequencer.plan(sequence)

    th.display_trajectory()

    th.sequencer.execute()


if __name__ == "__main__":

    robot_program()

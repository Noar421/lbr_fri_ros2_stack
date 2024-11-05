from typing import List

import rclpy
import math
from geometry_msgs.msg import Point, Pose, Quaternion
from moveit_msgs.action import MoveGroupSequence
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    MotionPlanRequest,
    MotionSequenceItem,
    OrientationConstraint,
    PositionConstraint,
)

from rclpy.action import ActionClient
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header

class SequencedMotionCart(Node):
    def __init__(self, node_name: str, namespace: str = "lbr") -> None:
        super().__init__(node_name, namespace=namespace)

        self.action_server = "/lbr/sequence_move_group"
        self.move_group_name = "arm"
        self.base = "lbr_link_0"
        self.end_effector = "lbr_link_ee"

        # MoveIt action client
        self.move_group_action_client = ActionClient(
            self, MoveGroupSequence, self.action_server
        )
        if not self.move_group_action_client.wait_for_server(timeout_sec=1):
            raise RuntimeError(
                f"Couldn't connect to action server {self.action_server}."
            )

    def _build_motion_plan_request(self, target_pose: Pose) -> MotionPlanRequest:
        req = MotionPlanRequest()

        # general config
        req.pipeline_id = "pilz_industrial_motion_planner"
        req.planner_id = "PTP"  # For Pilz PTP, LIN of CIRC
        req.allowed_planning_time = 10.0
        req.group_name = self.move_group_name
        # req.max_acceleration_scaling_factor = 0.1
        # req.max_velocity_scaling_factor = 0.1
        req.max_acceleration_scaling_factor = 0.5
        req.max_velocity_scaling_factor = 0.25
        req.num_planning_attempts = 1000
        
        # goal constraints
        req.goal_constraints.append(
            Constraints(
                position_constraints=[
                    PositionConstraint(
                        header=Header(frame_id=self.base),
                        link_name=self.end_effector,
                        constraint_region=BoundingVolume(
                            primitives=[SolidPrimitive(type=2, dimensions=[0.0001])],
                            primitive_poses=[Pose(position=target_pose.position)],
                        ),
                        weight=1.0,
                    )
                ],
                orientation_constraints=[
                    OrientationConstraint(
                        header=Header(frame_id=self.base),
                        link_name=self.end_effector,
                        orientation=target_pose.orientation,
                        absolute_x_axis_tolerance=0.001,
                        absolute_y_axis_tolerance=0.001,
                        absolute_z_axis_tolerance=0.001,
                        weight=1.0,
                    )
                ],
            )
        )
        return req

    def execute_sequence(self, target_poses: List[Pose]) -> None:
        goal = MoveGroupSequence.Goal()
        for target_pose in target_poses:
            goal.request.items.append(
                MotionSequenceItem(
                    blend_radius=0.0,
                    req=self._build_motion_plan_request(target_pose),
                )
            )
        goal.request.items[-1].blend_radius = 0.0  # last radius must be 0
        future = self.move_group_action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

def main(args: List = None) -> None:

    rclpy.init(args=args)

    sequenced_motion = SequencedMotionCart(
        "sequenced_motion_node"
        )
    
    target_poses = [
        Pose(position=Point(x=0.3, y=0.0, z=0.6), orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0)),
        Pose(position=Point(x=0.3, y=0.2, z=0.6), orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0)),
        Pose(position=Point(x=0.4, y=0.2, z=0.6), orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0)),
        Pose(position=Point(x=0.4, y=-0.2, z=0.6), orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0)),
        Pose(position=Point(x=0.3, y=-0.2, z=0.6), orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0)),
        Pose(position=Point(x=0.3, y=0.0, z=0.6), orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0)),
        #     # add poses as needed
    ]
    sequenced_motion.execute_sequence(target_poses=target_poses)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
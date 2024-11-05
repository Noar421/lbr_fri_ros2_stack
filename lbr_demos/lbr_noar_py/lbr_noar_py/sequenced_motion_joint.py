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
    JointConstraint
)

from rclpy.action import ActionClient
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header

from sensor_msgs.msg import JointState

class SequencedMotionJoint(Node):
    def __init__(self, node_name: str, namespace: str = "lbr") -> None:
        super().__init__(node_name, namespace=namespace)

        self.action_server = "/lbr/sequence_move_group"
        self.move_group_name = "arm"
        self.base = "lbr_link_0"
        self.end_effector = "lbr_link_ee"

        # Create a joint_state subscriber to get the current robot_state
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        self.subscription  # Prevent unused variable warning

        # MoveIt action client
        self.move_group_action_client = ActionClient(
            self, MoveGroupSequence, self.action_server
        )
        if not self.move_group_action_client.wait_for_server(timeout_sec=1):
            raise RuntimeError(
                f"Couldn't connect to action server {self.action_server}."
            )

    def joint_state_callback(self, msg):
        self.get_logger().info(f"Received joint state:\n{msg}")

    def _build_motion_plan_request(self, target_joints: List[float]) -> MotionPlanRequest:
        req = MotionPlanRequest()

        # general config
        req.pipeline_id = "pilz_industrial_motion_planner"
        req.planner_id = "PTP"  # For Pilz PTP, LIN of CIRC
        req.allowed_planning_time = 10.0
        req.group_name = self.move_group_name
        req.max_acceleration_scaling_factor = 0.5
        req.max_velocity_scaling_factor = 0.25
        req.num_planning_attempts = 1000

        # goal constraints
        req.goal_constraints.append(
            Constraints(
                joint_constraints = [
                    JointConstraint(
                        joint_name = "lbr_A1",
                        position = target_joints[0],
                        tolerance_above = 0.5,
                        tolerance_below = 0.5,
                        weight=1.0
                    ),
                    JointConstraint(
                        joint_name = "lbr_A2",
                        position = target_joints[1],
                        tolerance_above = 0.5,
                        tolerance_below = 0.5,
                        weight=1.0
                    ),
                    JointConstraint(
                        joint_name = "lbr_A3",
                        position = target_joints[2],
                        tolerance_above = 0.5,
                        tolerance_below = 0.5,
                        weight=1.0
                    ),
                    JointConstraint(
                        joint_name = "lbr_A4",
                        position = target_joints[3],
                        tolerance_above = 0.5,
                        tolerance_below = 0.5,
                        weight=1.0
                    ),
                    JointConstraint(
                        joint_name = "lbr_A5",
                        position = target_joints[4],
                        tolerance_above = 0.5,
                        tolerance_below = 0.5,
                        weight=1.0
                    ),
                    JointConstraint(
                        joint_name = "lbr_A6",
                        position = target_joints[5],
                        tolerance_above = 0.5,
                        tolerance_below = 0.5,
                        weight=1.0
                    ),
                    JointConstraint(
                        joint_name = "lbr_A7",
                        position = target_joints[6],
                        tolerance_above = 0.5,
                        tolerance_below = 0.5,
                        weight=1.0
                    ),
                    
                ]
            )
        )
        return req

    def execute_sequence(self, target_poses: List[Pose]) -> None:
        goal = MoveGroupSequence.Goal()
        for target_pose in target_poses:
            goal.request.items.append(
                MotionSequenceItem(
                    blend_radius=0.01,
                    req=self._build_motion_plan_request(target_pose),
                )
            )
        goal.request.items[0].blend_radius = 0.0  # last radius must be 0
        goal.request.items[-1].blend_radius = 0.0  # last radius must be 0
        future = self.move_group_action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

def main(args: List = None) -> None:

    rclpy.init(args=args)

    sequenced_motion = SequencedMotionJoint(
        "sequenced_motion_node"
        )
    
    target_poses = [
        [math.radians(0.0),
         math.radians(-40.0),
         math.radians(0.0),
         math.radians(-80.0),
         math.radians(0.0),
         math.radians(60.0),
         math.radians(0.0)         
         ],
        [math.radians(0.0),
         math.radians(0.0),
         math.radians(0.0),
         math.radians(0.0),
         math.radians(0.0),
         math.radians(0.0),
         math.radians(0.0)         
         ],
        [math.radians(30.0),
         math.radians(-40.0),
         math.radians(0.0),
         math.radians(-80.0),
         math.radians(0.0),
         math.radians(60.0),
         math.radians(0.0)         
         ],
        [math.radians(0.0),
         math.radians(-40.0),
         math.radians(0.0),
         math.radians(-80.0),
         math.radians(0.0),
         math.radians(60.0),
         math.radians(0.0)         
         ],
        [math.radians(0.0),
         math.radians(0.0),
         math.radians(0.0),
         math.radians(0.0),
         math.radians(0.0),
         math.radians(0.0),
         math.radians(0.0)         
         ],
        #     # add poses as needed
    ]
    sequenced_motion.execute_sequence(target_poses=target_poses)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
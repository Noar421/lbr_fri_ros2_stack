from typing import List

import rclpy
import math
from geometry_msgs.msg import PoseArray, Pose 
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from moveit_msgs.action import MoveGroupSequence
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    MotionPlanRequest,
    MotionSequenceItem,
    JointConstraint,
    OrientationConstraint,
    PositionConstraint,
)
from rclpy.executors import ExternalShutdownException
from rclpy.action import ActionClient
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header

from sensor_msgs.msg import JointState

class SequencedMotion(Node):
    
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
        self.get_logger().info(f"Subscribed to {self.subscription.topic}")
        self.subscription  # Prevent unused variable warning

        # Create a custom subscriber to get joint trajectory (PTP) from unity
        self.subscription = self.create_subscription(
            JointTrajectory,
            '/unity_joint_trajectory',
            self.unity_joint_trajectory_callback,
            10)
        self.get_logger().info(f"Subscribed to {self.subscription.topic}")
        self.subscription  # Prevent unused variable warning

        # Create a custom subscriber to get cartesian trajectory (PTP) from unity
        self.subscription = self.create_subscription(
            PoseArray,
            '/unity_cart_trajectory',
            self.unity_cart_trajectory_callback,
            10)
        self.get_logger().info(f"Subscribed to {self.subscription.topic}")
        self.subscription  # Prevent unused variable warning

        # Create a custom subscriber to get joint target (PTP) from unity
        self.subscription = self.create_subscription(
            JointState,
            '/unity_target',
            self.unity_target_callback,
            10)
        self.get_logger().info(f"Subscribed to {self.subscription.topic}")
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

    def unity_joint_trajectory_callback(self, msg:JointTrajectory):
        self.get_logger().info(f"Received trajectory from unity :\n{msg}")
        target_poses = []
        point: JointTrajectoryPoint
        for point in msg.points:
            target_poses.append(
                [
                    math.radians(point.positions[0]),
                    math.radians(point.positions[1]),
                    math.radians(point.positions[2]),
                    math.radians(point.positions[3]),
                    math.radians(point.positions[4]),
                    math.radians(point.positions[5]),
                    math.radians(point.positions[6])
                ]
            )
        
        # Launch movement
        self.execute_joint_sequence(target_poses=target_poses)

    def unity_cart_trajectory_callback(self, msg:PoseArray):
        self.get_logger().info(f"Received cartesian trajectory from unity :\n{msg}")
        target_poses = []
        point: Pose
        for point in msg.poses:
            target_poses.append(point)
        
        # Launch movement
        self.execute_cart_sequence(target_poses=target_poses)

    def unity_target_callback(self, msg:JointState):
        self.get_logger().info(f"Received target from unity :\n{msg}")
        target_poses = []
        target_poses.append(
            [
                math.radians(msg.position[0]),
                math.radians(msg.position[1]),
                math.radians(msg.position[2]),
                math.radians(msg.position[3]),
                math.radians(msg.position[4]),
                math.radians(msg.position[5]),
                math.radians(msg.position[6])
            ]
        )
        
        # Launch movement
        self.execute_joint_sequence(target_poses=target_poses)

    def _build_joint_motion_plan_request(self, target_joints: List[float], velocity_scaling_factor=1.0) -> MotionPlanRequest:
        req = MotionPlanRequest()

        # general config
        req.pipeline_id = "pilz_industrial_motion_planner"
        req.planner_id = "PTP"  # For Pilz PTP, LIN of CIRC
        req.allowed_planning_time = 10.0
        req.group_name = self.move_group_name
        req.max_acceleration_scaling_factor = 0.5
        req.max_velocity_scaling_factor = velocity_scaling_factor
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

    def execute_joint_sequence(self, target_poses: List[Pose]) -> None:
        goal = MoveGroupSequence.Goal()
        for target_pose in target_poses:
            goal.request.items.append(
                MotionSequenceItem(
                    blend_radius=0.01,
                    req=self._build_joint_motion_plan_request(target_pose),
                )
            )
        goal.request.items[0].blend_radius = 0.0  # last radius must be 0
        goal.request.items[-1].blend_radius = 0.0  # last radius must be 0
        future = self.move_group_action_client.send_goal_async(goal)

    def _build_cart_motion_plan_request(self, target_pose: Pose) -> MotionPlanRequest:
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

    def execute_cart_sequence(self, target_poses: List[Pose]) -> None:
        goal = MoveGroupSequence.Goal()
        for target_pose in target_poses:
            goal.request.items.append(
                MotionSequenceItem(
                    blend_radius=0.01,
                    req=self._build_cart_motion_plan_request(target_pose),
                )
            )
        goal.request.items[0].blend_radius = 0.0  # first radius must be 0 TBC
        goal.request.items[-1].blend_radius = 0.0  # last radius must be 0
        future = self.move_group_action_client.send_goal_async(goal)


def main(args: List = None) -> None:
    try:
        with rclpy.init(args=args):
            # Instantiate our node
            sequenced_motion = SequencedMotion(
                "sequenced_motion_node"
                )
            rclpy.spin(sequenced_motion)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    

if __name__ == "__main__":
    main()
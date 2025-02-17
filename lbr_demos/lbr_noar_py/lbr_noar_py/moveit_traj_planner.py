from typing import List

import rclpy
import math
from geometry_msgs.msg import Point, Pose, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState


class MoveGroupActionClientNode(Node):
    def __init__(self, node_name: str, namespace: str = "lbr") -> None:
        super().__init__(node_name, namespace=namespace)

        self.action_server = "/lbr/move_action"
        self.move_group_name = "arm"
        self.base = "lbr_link_0"
        self.end_effector = "lbr_link_ee"

        # MoveIt action client
        self.move_group_action_client = ActionClient(
            self, MoveGroup, self.action_server
        )
        if not self.move_group_action_client.wait_for_server(timeout_sec=1):
            raise RuntimeError(
                f"Couldn't connect to action server {self.action_server}."
            )

        # Inverse kinematics client
        self.ik_client = self.create_client(GetPositionIK, "compute_ik")
        if not self.ik_client.wait_for_service(timeout_sec=1):
            raise RuntimeError(
                f"Couldn't connect to service {self.ik_client.srv_name}."
            )

    def request_inverse_kinematics(self, pose: Pose) -> JointState:
        r"""Request inverse kinematics for a given pose."""
        request = GetPositionIK.Request()
        request.ik_request.group_name = self.move_group_name
        # tf_prefix = self.get_namespace().removeprefix("/")
        tf_prefix = ""
        # request.ik_request.pose_stamped.header.frame_id = f"{tf_prefix}/{self.base}"
        request.ik_request.pose_stamped.header.frame_id = f"{self.base}"
        request.ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        request.ik_request.pose_stamped.pose = pose
        request.ik_request.avoid_collisions = True
        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response is None:
            self.get_logger().error("Inverse kinematics service call failed")
            return
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(
                f"Failed to compute inverse kinematics: {response.error_code.val}"
            )
            return
        return response.solution.joint_state

    def move_to_pose(self, target: Pose):
        r"""Move the robot to a given pose. Do so by requesting inverse kinematics and then sending a move group action."""
        joint_state = self.request_inverse_kinematics(target)
        if joint_state is None:
            return

        # Prepare goal
        goal = MoveGroup.Goal()
        # goal.request.allowed_planning_time = 5.0
        # goal.request.num_planning_attempts = 10
        # goal.request.group_name = self.move_group_name
        # goal.request.max_acceleration_scaling_factor = 0.1
        # goal.request.max_velocity_scaling_factor = 0.05

        goal.request.allowed_planning_time = 10.0
        goal.request.num_planning_attempts = 50
        goal.request.group_name = self.move_group_name
        goal.request.max_acceleration_scaling_factor = 0.1
        goal.request.max_velocity_scaling_factor = 0.5

        # Set joint constraints
        goal.request.goal_constraints.append(
            Constraints(
                joint_constraints=[
                    JointConstraint(
                        joint_name=joint_state.name[i],
                        position=joint_state.position[i],
                        tolerance_above=0.001,
                        tolerance_below=0.001,
                        weight=1.0,
                    )
                    for i in range(len(joint_state.name))
                ]
            )
        )

        # Send goal
        goal_future = self.move_group_action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, goal_future)
        goal_handle = goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("MoveGroup goal rejected")
            return

        # Wait for result
        self.get_logger().info("MoveGroup goal accepted")
        self.get_logger().info("Waiting for result...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(f"MoveGroup action failed: {result.error_code.val}")
            return
        self.get_logger().info("MoveGroup action succeeded")


def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [w, x, y, z]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr
    print(f'r:{roll}, p:{pitch}, y:{yaw}')
    print(f'x:{q[1]}, y:{q[2]}, z:{q[3]}, w:{q[0]}')
    return Quaternion(x=q[1], y=q[2], z=q[3], w=q[0])

def main(args: List = None) -> None:

    rclpy.init(args=args)
    move_group_action_client_node = MoveGroupActionClientNode(
        "move_group_action_client_node"
    )

    # List of poses to move the robot to
    poses = [
        Pose(
            position=Point(x=0.4, y=0.0, z=0.4),
            orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
        ),
        Pose(
            position=Point(x=0.4, y=0.2, z=0.4),
            orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
        ),
        Pose(
            position=Point(x=0.4, y=0.2, z=0.8),
            orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
        ),
        Pose(
            position=Point(x=0.4, y=-0.2, z=0.8),
            orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
        ),
        Pose(
            position=Point(x=0.4, y=-0.2, z=0.4),
            orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
        ),
        Pose(
            position=Point(x=0.4, y=0.0, z=0.4),
            orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
        ),
        # Add more poses as needed
    ]

    for pose in poses:
        move_group_action_client_node.move_to_pose(pose)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
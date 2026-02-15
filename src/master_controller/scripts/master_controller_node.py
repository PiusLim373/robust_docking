#!/usr/bin/python3

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.action import ActionClient
import math
import time
import uuid
import numpy as np
from tf_transformations import euler_from_quaternion
from std_srvs.srv import Trigger
from gazebo_msgs.srv import GetEntityState
from custom_docker_msgs.srv import DockingGoal
from custom_docker_msgs.msg import DockingStatus
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


class MasterController(Node):
    def __init__(self):
        super().__init__("master_controller_node")
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])
        self.first_stage_offset = 0.9
        self.second_stage_offset = 0.4

        # subscriber
        self.docking_status = DockingStatus()
        self.create_subscription(
            DockingStatus, "/docking_status", self.docking_status_cb, 1, callback_group=MutuallyExclusiveCallbackGroup()
        )

        # services
        self.corner_docking_client = self.create_client(DockingGoal, "/dock_to_corner")
        self.get_robot_pose_client = self.create_client(GetEntityState, "/get_entity_state")
        self.create_service(
            Trigger,
            "/start_corner_docking_smach",
            self.start_corner_docking_smach_cb,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.create_service(
            Trigger,
            "/recover",
            self.recover_cb,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        self.nav2_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.get_logger().info("Master Controller Node has been started.")

    #### nav2 actions related
    def send_goal(self, x, y, yaw):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.nav2_client.wait_for_server()
        self._send_goal_future = self.nav2_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            return
        self.get_logger().info("Goal accepted")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info("Navigation finished")

    ##### nav2 actions related end

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

    def docking_status_cb(self, msg: DockingStatus):
        self.docking_status = msg

    def start_corner_docking_smach_cb(self, request, response):
        self.get_logger().info("Corner docking smach started")

        ##### First stage docking #####
        first_stage_id = str(uuid.uuid4())
        self.get_logger().info(f"First stage docking in progress..., docking_id: {first_stage_id}")
        data = DockingGoal.Request()
        data.activate = True
        data.offset_x = self.first_stage_offset
        data.docking_id = first_stage_id
        self.corner_docking_client.call(data)
        self.get_logger().info("First stage docking sent, waiting for completion...")
        while self.docking_status.docking_id != first_stage_id or self.docking_status.status != "DOCKED":
            if self.docking_status.docking_id == first_stage_id and self.docking_status.status == "FAILED":
                self.get_logger().error(
                    f"First stage docking failed, status: {self.docking_status.status}, docking_id: {self.docking_status.docking_id}"
                )
                self.get_logger().error("Sending robot to recovery point (0.75, 0.75, 45deg)")
                self.send_goal(0.75, 0.75, np.radians(45))
                response.success = False
                return response
            time.sleep(0.5)
        self.get_logger().info("First stage docking complete, transitioning to second stage")

        ##### Second stage docking #####
        second_stage_id = str(uuid.uuid4())
        self.get_logger().info(f"Second stage docking in progress..., id: {second_stage_id}")
        data = DockingGoal.Request()
        data.activate = True
        data.offset_x = self.second_stage_offset
        data.docking_id = second_stage_id
        self.corner_docking_client.call(data)
        self.get_logger().info("Second stage docking sent, waiting for completion...")
        while self.docking_status.docking_id != second_stage_id or self.docking_status.status != "DOCKED":
            if self.docking_status.docking_id == second_stage_id and self.docking_status.status == "FAILED":
                self.get_logger().error(
                    f"Second stage docking failed, status: {self.docking_status.status}, docking_id: {self.docking_status.docking_id}"
                )
                self.get_logger().error("Sending robot to recovery point (0.75, 0.75, 45deg)")
                self.send_goal(0.75, 0.75, np.radians(45))
                response.success = False
                return response
            time.sleep(0.5)
        self.get_logger().info("Second stage docking complete, transitioning to benchmarking")

        ##### Benchmarking #####
        self.get_logger().info("Benchmarking in progress...")
        get_robot_pose_req = GetEntityState.Request()
        get_robot_pose_req.name = "waffle_pi"
        robot_pose_data = self.get_robot_pose_client.call(get_robot_pose_req)
        if robot_pose_data.success:
            position = robot_pose_data.state.pose.position
            orientation = robot_pose_data.state.pose.orientation
            _, _, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
            offset_from_corner = math.sqrt(position.x**2 + position.y**2)
            self.get_logger().warn(
                "============================== Benchmarking completed =============================="
            )
            self.get_logger().warn(
                f"Robot true pose from Gazebo: ({position.x:.5f}, {position.y:.5f}), Offset from corner: {offset_from_corner:.5f} meters, target offset: {self.second_stage_offset:.2f} meters"
            )
            self.get_logger().warn(
                f"Robot true yaw from Gazebo: {np.degrees(yaw):.5f} degrees, target orientation: 45.0 degrees"
            )
        response.success = True
        return response

    def recover_cb(self, request, response):
        self.get_logger().info("Recover service called, sending robot to (0.75, 0.75, 45deg)")
        self.send_goal(0.75, 0.75, np.radians(45))
        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)
    master_controller_node = MasterController()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(master_controller_node)
    executor.spin()
    executor.shutdown()
    master_controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

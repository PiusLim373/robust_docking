#!/usr/bin/python3

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
import tf2_ros
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
import time
import random
import copy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool


class DisturbanceManager(Node):

    def __init__(self):
        super().__init__("disturbance_manager_node")
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.noisy_odom_enabled = False
        self.crippled_laser_enabled = False

        self.xy_sigma = 0.01    # noisy odom std dev in x and y direction (meters)
        self.yaw_sigma = 0.01   # noisy odom std dev in yaw (radians)

        # service clients
        self.create_service(
            SetBool,
            "/toggle_noisy_odom",
            self.toggle_noisy_odom_cb,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.create_subscription(Odometry, "/odom", self.odom_cb, 1, callback_group=MutuallyExclusiveCallbackGroup())

        self.dropout_probability = 0.1  # Drop 10% of scans
        self.freeze_probability = 0.05  # 5% chance to start freeze
        self.freeze_duration = 1.0  # seconds
        self.corrupt_probability = 0.2  # 20% chance to corrupt a scan
        self.corrupt_sector_size = 20  # number of beams to corrupt
        self.freeze_active = False
        self.freeze_end_time = 0.0
        self.frozen_scan = None
        self.create_service(
            SetBool,
            "/toggle_crippled_laser",
            self.toggle_crippled_laser_cb,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.create_subscription(LaserScan, "/scan", self.laser_cb, 1, callback_group=MutuallyExclusiveCallbackGroup())
        self.crippled_laser_pub = self.create_publisher(LaserScan, "/crippled_scan", 1)

        self.get_logger().info("Disturbance Manager Node has been started.")

    def odom_cb(self, msg: Odometry):
        transform_data = TransformStamped()
        transform_data.header.stamp = self.get_clock().now().to_msg()
        transform_data.header.frame_id = "odom"
        transform_data.child_frame_id = "base_footprint"
        transform_data.transform.translation.x = msg.pose.pose.position.x
        transform_data.transform.translation.y = msg.pose.pose.position.y
        transform_data.transform.translation.z = msg.pose.pose.position.z
        if self.noisy_odom_enabled:
            transform_data.transform.translation.x += np.random.normal(0, self.xy_sigma)
            transform_data.transform.translation.y += np.random.normal(0, self.xy_sigma)
            q = msg.pose.pose.orientation
            quat = [q.x, q.y, q.z, q.w]
            roll, pitch, yaw = euler_from_quaternion(quat)
            yaw += np.random.normal(0, self.yaw_sigma)
            q_noisy = quaternion_from_euler(roll, pitch, yaw)
            transform_data.transform.rotation.x = q_noisy[0]
            transform_data.transform.rotation.y = q_noisy[1]
            transform_data.transform.rotation.z = q_noisy[2]
            transform_data.transform.rotation.w = q_noisy[3]
        else:
            transform_data.transform.rotation.x = msg.pose.pose.orientation.x
            transform_data.transform.rotation.y = msg.pose.pose.orientation.y
            transform_data.transform.rotation.z = msg.pose.pose.orientation.z
            transform_data.transform.rotation.w = msg.pose.pose.orientation.w
        self.tf_broadcaster.sendTransform(transform_data)

    def laser_cb(self, msg: LaserScan):
        if self.crippled_laser_enabled:
            now = time.time()

            # dropout
            if random.random() < self.dropout_probability:
                self.get_logger().debug("Scan dropped")
                return

            # freeze
            if self.freeze_active:
                if now < self.freeze_end_time:
                    self.crippled_laser_pub.publish(self.frozen_scan)
                    return
                else:
                    self.freeze_active = False
            elif random.random() < self.freeze_probability:
                self.freeze_active = True
                self.freeze_end_time = now + self.freeze_duration
                self.frozen_scan = copy.deepcopy(msg)
                self.get_logger().debug("Scan freeze started")
                self.crippled_laser_pub.publish(self.frozen_scan)
                return

            # corrupt
            cripped_scan = copy.deepcopy(msg)
            if random.random() < self.corrupt_probability:
                total_beams = len(cripped_scan.ranges)
                start_index = random.randint(0, total_beams - self.corrupt_sector_size)
                for i in range(start_index, start_index + self.corrupt_sector_size):
                    cripped_scan.ranges[i] = float("inf")
                self.get_logger().debug(
                    f"Corrupted sector {start_index} to " f"{start_index + self.corrupt_sector_size}"
                )
            self.crippled_laser_pub.publish(cripped_scan)

        else:
            self.crippled_laser_pub.publish(msg)

    def toggle_noisy_odom_cb(self, request, response):
        self.noisy_odom_enabled = request.data
        self.get_logger().info(f"Toggle Noisy Odom: {request.data}")
        response.success = True
        return response

    def toggle_crippled_laser_cb(self, request, response):
        self.crippled_laser_enabled = request.data
        self.get_logger().info(f"Toggle Crippled Laser: {request.data}")
        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)
    disturbance_manager_node = DisturbanceManager()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(disturbance_manager_node)
    executor.spin()
    executor.shutdown()
    disturbance_manager_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

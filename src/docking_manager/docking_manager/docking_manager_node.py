#!/usr/bin/python3

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from tf_transformations import euler_from_quaternion
import tf2_ros
import math
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseStamped, Twist, Pose, TransformStamped
from custom_docker_msgs.srv import DockingGoal
from custom_docker_msgs.msg import DockingStatus


class DockingManager(Node):

    def __init__(self):
        super().__init__("docking_manager_node")
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])

        # Control parameters
        self.k_rho = 0.8
        self.k_alpha = 2.0
        self.k_theta = 2.5
        self.v_max = 0.2
        self.omega_max = 0.2
        self.goal_tolerance = 0.03  # 3cm
        self.angle_tolerance = 0.03  # rad (~2 deg)
        # Safety
        self.target_timeout = 0.5  # seconds
        self.last_target_time = None
        self.lost_target_starttime = 0.0
        self.lost_target_docking_failure_patience = 5.0  # seconds

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.x_t = None
        self.y_t = None
        self.theta_d = None

        # State
        self.start_docking = False
        self.offset_x = 0.0
        self.state = "IDLE"
        self.docking_id = ""

        self.create_service(
            DockingGoal, "/dock_to_corner", self.dock_to_corner_cb, callback_group=MutuallyExclusiveCallbackGroup()
        )

        # Subscriber
        self.subscription = self.create_subscription(
            PoseStamped, "/corner_pose", self.corner_pose_cb, 1, callback_group=MutuallyExclusiveCallbackGroup()
        )
        # Publisher
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 1)
        self.docking_status_pub = self.create_publisher(DockingStatus, "/docking_status", 1)

        # Timer
        self.timer = self.create_timer(0.05, self.control_loop, callback_group=MutuallyExclusiveCallbackGroup())

        self.get_logger().info("Docking controller started")

    def dock_to_corner_cb(self, request, response):
        self.get_logger().info(
            f"Received docking request: activate={request.activate}, offset_x={request.offset_x:.3f}, docking_id={request.docking_id}"
        )
        self.start_docking = request.activate
        self.docking_id = request.docking_id
        if not request.activate:
            self.offset_x = 0.0
            self.stop_robot()
        else:
            self.offset_x = request.offset_x
        self.state = "IDLE"
        response.success = True
        return response

    def corner_pose_cb(self, msg):
        if self.offset_x == 0.0 or not self.start_docking:
            return
        # transform the target: rotate 180deg about z axis and translate forward by offset_x
        baselink_T_corner = self.pose_to_matrix(msg.pose)
        offset_T = np.array(
            [
                [np.cos(np.radians(180.0)), -np.sin(np.radians(180.0)), 0, 0],
                [np.sin(np.radians(180.0)), np.cos(np.radians(180.0)), 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ]
        )
        offset_T[0, 3] = self.offset_x
        offset_baselink_T_corner = baselink_T_corner @ offset_T
        docking_target = PoseStamped()
        docking_target = msg
        docking_target.pose = self.matrix_to_pose(offset_baselink_T_corner)
        self.publish_frame(docking_target, "base_link", "docking_target")
        self.x_t = docking_target.pose.position.x
        self.y_t = docking_target.pose.position.y
        q = docking_target.pose.orientation
        quat = [q.x, q.y, q.z, q.w]
        _, _, yaw = euler_from_quaternion(quat)
        self.theta_d = yaw
        self.last_target_time = time.time()
        # start the state machine once first target is received
        if self.state == "IDLE":
            self.state = "APPROACH"

    def control_loop(self):
        docking_status = DockingStatus()
        docking_status.docking_id = self.docking_id
        docking_status.status = self.state
        self.docking_status_pub.publish(docking_status)

        if not self.start_docking or self.x_t is None or self.y_t is None or self.theta_d is None:
            return

        # Timeout check
        if self.last_target_time is None or (time.time() - self.last_target_time) > self.target_timeout:
            if self.lost_target_starttime != 0.0 and (
                (time.time() - self.lost_target_starttime) > self.lost_target_docking_failure_patience
            ):
                self.get_logger().warn("Docking target lost for too long, marking docking as failed")
                self.state = "FAILED"
                self.lost_target_starttime = 0.0
                self.start_docking = False
                self.stop_robot()
                return
            if self.lost_target_starttime == 0.0:
                self.lost_target_starttime = time.time()
                self.get_logger().warn("Docking target lost, starting timer...")
            self.get_logger().warn("Docking target timeout, stopping robot")
            self.stop_robot()
            return
        self.lost_target_starttime = 0.0

        # Compute polar error
        x = self.x_t
        y = self.y_t
        rho = math.sqrt(x * x + y * y)
        alpha = math.atan2(y, x)

        # State machine
        if self.state == "APPROACH":
            if x < 0.0:
                self.get_logger().warn("Target behind robot...")
                if rho < 3 * self.goal_tolerance:
                    self.get_logger().warn(f"But close enough with rho: {rho:.3f}, transit to rotation in place")
                    self.state = "ROTATE_IN_PLACE"
                    return
                else:
                    self.get_logger().warn("And too far, stopping robot")
                    self.stop_robot()
                    self.state = "FAILED"
                    return
            # Check if reached goal
            if rho < self.goal_tolerance:
                self.get_logger().info("Reached docking position, aligning...")
                self.state = "ROTATE_IN_PLACE"
                return
                # theta_error = wrap_to_pi(self.theta_d)
                # if abs(theta_error) < self.angle_tolerance:
                #     self.stop_robot()
                #     self.state = "DOCKED"
                #     self.get_logger().info("Docked successfully")
                #     self.get_logger().info(f"Final pose error: rho={rho:.3f}, theta_error={theta_error:.3f} rad")
                #     self.start_docking = False
                #     return
                # v = 0.0
                # omega = self.k_theta * theta_error
                # omega = max(min(omega, self.omega_max), -self.omega_max)
                # self.publish_cmd(v, omega)
                # return
            # Nonlinear control law
            v = self.k_rho * rho * math.cos(alpha)
            omega = self.k_alpha * alpha
            # Saturation
            v = max(min(v, self.v_max), -self.v_max)
            omega = max(min(omega, self.omega_max), -self.omega_max)
            # If heading very wrong, reduce forward motion
            if abs(alpha) > math.pi / 4:
                v = 0.0
            self.publish_cmd(v, omega)

        elif self.state == "ROTATE_IN_PLACE":
            theta_error = self.wrap_to_pi(self.theta_d)
            if abs(theta_error) < self.angle_tolerance:
                self.stop_robot()
                self.state = "DOCKED"
                self.get_logger().info("Docked successfully")
                self.get_logger().info(f"Final pose error: rho={rho:.3f}, theta_error={theta_error:.3f} rad")
                self.start_docking = False
                return
            v = 0.0
            omega = self.k_theta * theta_error
            omega = max(min(omega, self.omega_max), -self.omega_max)
            self.publish_cmd(v, omega)

        elif self.state == "WAIT_FOR_TARGET":
            self.stop_robot()

        elif self.state == "DOCKED":
            self.stop_robot()

    #######################################################  Helper functions

    def publish_cmd(self, v, omega):
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(omega)
        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

    def publish_frame(self, pose, parent_frame, child_frame):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        if isinstance(pose, PoseStamped):
            p = pose.pose
        elif isinstance(pose, Pose):
            p = pose
        else:
            raise TypeError("pose must be Pose or PoseStamped")
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
        t.transform.translation.x = p.position.x
        t.transform.translation.y = p.position.y
        t.transform.translation.z = p.position.z
        t.transform.rotation = p.orientation
        self.tf_broadcaster.sendTransform(t)

    def wrap_to_pi(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def pose_to_matrix(self, pose):
        t = np.array([pose.position.x, pose.position.y, pose.position.z])
        q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        rot = R.from_quat(q)
        R_mat = rot.as_matrix()
        T = np.eye(4)
        T[:3, :3] = R_mat
        T[:3, 3] = t
        return T

    def matrix_to_pose(self, T):
        if T.shape != (4, 4):
            raise ValueError("Input must be a 4x4 matrix")
        pose = Pose()
        pose.position.x = T[0, 3]
        pose.position.y = T[1, 3]
        pose.position.z = T[2, 3]
        R_mat = T[:3, :3]
        rot = R.from_matrix(R_mat)
        q = rot.as_quat()
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        return pose


def main(args=None):
    rclpy.init(args=args)
    docking_manager_node = DockingManager()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(docking_manager_node)
    executor.spin()
    executor.shutdown()
    docking_manager_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

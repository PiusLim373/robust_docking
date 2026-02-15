#!/usr/bin/python3
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import tf2_ros
import tf_transformations
from tf2_ros import TransformException
from tf2_geometry_msgs import do_transform_point
import numpy as np
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped, TransformStamped, Pose, PoseStamped


class FeatureDetector(rclpy.node.Node):
    def __init__(self):
        super().__init__("feature_detector_node")
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.filtered_corner = None
        self.filtered_bisector = None
        self.alpha = 1.0  # smoothing factor, 1.0 means no smoothing

        self.create_subscription(
            LaserScan, "/crippled_scan", self.scan_cb, 1, callback_group=MutuallyExclusiveCallbackGroup()
        )
        self.corner_pose_pub = self.create_publisher(PoseStamped, "/corner_pose", 1)
        self.get_logger().info("Feature Detector Node has been started.")

    def scan_cb(self, msg: LaserScan):
        # filter and convert polar scan to Cartesian coordinates,
        points = self.scan_to_points(msg)
        if len(points) < 5:
            # if there are too few points, skip processing
            return
        # RANSAC for line detection
        line1, inliers1 = self.ransac_line(points)
        if line1 is None:
            return
        # Remove inliers of the first line and run RANSAC again to find the second line
        remaining = np.array([p for p in points if not any(np.allclose(p, q) for q in inliers1)])
        if len(remaining) < 5:
            # if there are too few points left, skip processing
            return
        line2, inliers2 = self.ransac_line(remaining)
        if line2 is None:
            return

        # Check perpendicular
        angle_diff = self.angle_between_lines(line1, line2)
        if abs(abs(angle_diff) - np.radians(90.0)) > 0.1:
            return

        # do some process so that the x axis of corner detected is always pointing toward the robot
        corner = self.compute_intersection(line1, line2)
        if corner is None:
            return
        bisector = self.compute_bisector(line1, line2, corner)

        # low-pass filter the corner and bisector to reduce noise, mainly for tackle jitering detection
        if self.filtered_corner is None:
            self.filtered_corner = corner
            self.filtered_bisector = bisector
        else:
            self.filtered_corner = self.alpha * corner + (1 - self.alpha) * self.filtered_corner
            self.filtered_bisector = self.alpha * bisector + (1 - self.alpha) * self.filtered_bisector

        # populate the data and publish
        corner_pose = PoseStamped()
        corner_pose.header.frame_id = "base_link"
        corner_pose.header.stamp = self.get_clock().now().to_msg()
        corner_pose.pose.position.x = self.filtered_corner[0]
        corner_pose.pose.position.y = self.filtered_corner[1]
        corner_pose.pose.position.z = 0.0
        yaw = math.atan2(self.filtered_bisector[1], self.filtered_bisector[0])
        q = tf_transformations.quaternion_from_euler(0, 0, yaw)
        corner_pose.pose.orientation.x = q[0]
        corner_pose.pose.orientation.y = q[1]
        corner_pose.pose.orientation.z = q[2]
        corner_pose.pose.orientation.w = q[3]

        self.corner_pose_pub.publish(corner_pose)
        self.publish_frame(corner_pose, "base_link", "corner_pose")
        return

    #######################################################  Helper functions
    def compute_intersection(self, l1, l2):
        a1, b1, c1 = l1
        a2, b2, c2 = l2
        A = np.array([[a1, b1], [a2, b2]])
        B = np.array([[-c1], [-c2]])
        if abs(np.linalg.det(A)) < 1e-6:
            return None
        inter = np.linalg.inv(A) @ B
        return inter.flatten()

    def compute_bisector(self, l1, l2, corner):
        d1 = np.array([-l1[1], l1[0]])
        d2 = np.array([-l2[1], l2[0]])
        d1 /= np.linalg.norm(d1)
        d2 /= np.linalg.norm(d2)
        candidates = [d1 + d2, d1 - d2, -d1 + d2, -d1 - d2]
        candidates = [v / np.linalg.norm(v) for v in candidates]
        vector_to_robot = -corner
        best_bisector = max(candidates, key=lambda v: np.dot(v, vector_to_robot))
        return best_bisector

    def scan_to_points(self, scan: LaserScan):
        points = []
        for i, r in enumerate(scan.ranges):
            angle = scan.angle_min + i * scan.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))  # normalize
            # Filter out points outside of a 90 degree FOV in front of the robot
            if abs(angle) > math.radians(45.0):
                continue
            # Filter out invalid ranges
            if np.isinf(r) or np.isnan(r):
                continue
            # Filter out points that are too far away
            if r > 2.0:
                continue
            x_laser = r * math.cos(angle)
            y_laser = r * math.sin(angle)
            # Transform to base_link
            try:
                transform = self.tf_buffer.lookup_transform("base_link", scan.header.frame_id, rclpy.time.Time())
                point = PointStamped()
                point.header.frame_id = scan.header.frame_id
                point.point.x = x_laser
                point.point.y = y_laser
                point.point.z = 0.0
                point_bl = do_transform_point(point, transform)
                points.append([point_bl.point.x, point_bl.point.y])
            except TransformException:
                return []
        return np.array(points)

    def ransac_line(self, points, iterations=100, threshold=0.02):
        best_line = None
        best_inliers = []
        for _ in range(iterations):
            p1, p2 = points[np.random.choice(len(points), 2, replace=False)]
            a = p2[1] - p1[1]
            b = p1[0] - p2[0]
            c = -(a * p1[0] + b * p1[1])
            norm = math.sqrt(a * a + b * b)
            if norm < 1e-6:
                continue
            a, b, c = a / norm, b / norm, c / norm
            inliers = []
            for p in points:
                dist = abs(a * p[0] + b * p[1] + c)
                if dist < threshold:
                    inliers.append(p)
            if len(inliers) > len(best_inliers):
                best_line = (a, b, c)
                best_inliers = inliers
        if best_line is None:
            return None, None
        return best_line, np.array(best_inliers)

    def angle_between_lines(self, l1, l2):
        a1, b1, _ = l1
        a2, b2, _ = l2
        n1 = np.array([a1, b1])
        n2 = np.array([a2, b2])
        cosang = np.dot(n1, n2)
        return math.acos(np.clip(cosang, -1.0, 1.0))

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


def main(args=None):
    rclpy.init(args=args)
    feature_detector_node = FeatureDetector()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(feature_detector_node)
    executor.spin()
    executor.shutdown()
    feature_detector_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

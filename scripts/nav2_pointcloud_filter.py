#!/usr/bin/env python3
"""Filter FAST-LIO body-frame clouds into a 2D-costmap obstacle cloud."""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


class Nav2PointCloudFilter(Node):
    def __init__(self):
        super().__init__('nav2_pointcloud_filter')

        self.declare_parameter('input_topic', '/cloud_registered_body')
        self.declare_parameter('output_topic', '/costmap/points')
        self.declare_parameter('min_range', 0.15)
        self.declare_parameter('max_range', 8.0)
        self.declare_parameter('voxel_size', 0.05)
        self.declare_parameter('ground_distance', 0.08)
        self.declare_parameter('min_obstacle_height', 0.18)
        self.declare_parameter('max_obstacle_height', 1.8)
        self.declare_parameter('ransac_iterations', 60)
        self.declare_parameter('ransac_sample_size', 6000)
        self.declare_parameter('ground_candidate_percentile', 45.0)
        self.declare_parameter('max_ground_tilt_deg', 30.0)

        self.input_topic = self.get_parameter(
            'input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter(
            'output_topic').get_parameter_value().string_value
        self.min_range = self.get_parameter(
            'min_range').get_parameter_value().double_value
        self.max_range = self.get_parameter(
            'max_range').get_parameter_value().double_value
        self.voxel_size = self.get_parameter(
            'voxel_size').get_parameter_value().double_value
        self.ground_distance = self.get_parameter(
            'ground_distance').get_parameter_value().double_value
        self.min_obstacle_height = self.get_parameter(
            'min_obstacle_height').get_parameter_value().double_value
        self.max_obstacle_height = self.get_parameter(
            'max_obstacle_height').get_parameter_value().double_value
        self.ransac_iterations = self.get_parameter(
            'ransac_iterations').get_parameter_value().integer_value
        self.ransac_sample_size = self.get_parameter(
            'ransac_sample_size').get_parameter_value().integer_value
        self.ground_candidate_percentile = self.get_parameter(
            'ground_candidate_percentile').get_parameter_value().double_value
        max_ground_tilt_deg = self.get_parameter(
            'max_ground_tilt_deg').get_parameter_value().double_value
        self.min_ground_normal_z = math.cos(math.radians(max_ground_tilt_deg))

        self.sub = self.create_subscription(
            PointCloud2, self.input_topic, self.cloud_callback, 10)
        self.pub = self.create_publisher(PointCloud2, self.output_topic, 10)
        self.rng = np.random.default_rng()

        self.get_logger().info(
            f'Filtering {self.input_topic} -> {self.output_topic}')

    def cloud_callback(self, msg):
        points = point_cloud2.read_points_numpy(
            msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if points.size == 0:
            self.publish_points(msg.header, np.empty((0, 3), dtype=np.float32))
            return

        points = np.asarray(points, dtype=np.float32).reshape((-1, 3))

        distance_xy = np.linalg.norm(points[:, :2], axis=1)
        range_mask = (distance_xy >= self.min_range) & (distance_xy <= self.max_range)
        points = points[range_mask]
        if points.size == 0:
            self.publish_points(msg.header, np.empty((0, 3), dtype=np.float32))
            return

        points = self.voxel_downsample(points, self.voxel_size)
        plane = self.fit_ground_plane(points)
        if plane is None:
            height_mask = (
                (points[:, 2] >= self.min_obstacle_height) &
                (points[:, 2] <= self.max_obstacle_height)
            )
            obstacles = points[height_mask]
        else:
            normal, offset = plane
            signed_height = points @ normal + offset
            obstacles = points[
                (signed_height > self.min_obstacle_height) &
                (signed_height < self.max_obstacle_height)
            ]

        self.publish_points(msg.header, obstacles.astype(np.float32, copy=False))

    def fit_ground_plane(self, points):
        if points.shape[0] < 30:
            return None

        z_limit = np.percentile(points[:, 2], self.ground_candidate_percentile)
        candidates = points[points[:, 2] <= z_limit]
        if candidates.shape[0] < 30:
            return None

        if candidates.shape[0] > self.ransac_sample_size:
            sample_indices = self.rng.choice(
                candidates.shape[0], self.ransac_sample_size, replace=False)
            candidates = candidates[sample_indices]

        best_plane = None
        best_inliers = 0
        for _ in range(max(self.ransac_iterations, 1)):
            idx = self.rng.choice(candidates.shape[0], 3, replace=False)
            p1, p2, p3 = candidates[idx]
            normal = np.cross(p2 - p1, p3 - p1)
            norm = np.linalg.norm(normal)
            if norm < 1e-6:
                continue
            normal = normal / norm
            if normal[2] < 0.0:
                normal = -normal
            if normal[2] < self.min_ground_normal_z:
                continue
            offset = -float(normal @ p1)
            distances = np.abs(candidates @ normal + offset)
            inliers = int(np.count_nonzero(distances < self.ground_distance))
            if inliers > best_inliers:
                best_inliers = inliers
                best_plane = (normal, offset)

        return best_plane

    @staticmethod
    def voxel_downsample(points, voxel_size):
        if voxel_size <= 0.0 or points.shape[0] == 0:
            return points
        keys = np.floor(points / voxel_size).astype(np.int32)
        _, unique_indices = np.unique(keys, axis=0, return_index=True)
        return points[np.sort(unique_indices)]

    def publish_points(self, source_header, points):
        header = Header()
        header.stamp = source_header.stamp
        header.frame_id = source_header.frame_id
        cloud = point_cloud2.create_cloud_xyz32(header, points.tolist())
        self.pub.publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    node = Nav2PointCloudFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

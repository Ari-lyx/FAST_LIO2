#!/usr/bin/env python3
"""
FAST-LIO -> Nav2 Odometry Bridge

功能：
  1. 发布静态 TF: map->odom, map->camera_init (identity)
  2. 在 FAST-LIO 首帧到来前持续发布初始 identity TF: odom->base_footprint
  3. 读取 FAST-LIO 的 camera_init->body TF
     -> 投影为 2D base_footprint 位姿
     -> 更新动态 TF: odom->base_footprint
     -> 重发布为 /odom

TF 树:
  map --(static identity)--> odom --(dynamic 2D FAST-LIO pose)--> base_footprint
  base_footprint --(URDF fixed)--> base_link
  map --(static identity)--> camera_init --(FAST-LIO TF)--> body
"""

import math
from copy import deepcopy

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros


def yaw_from_quaternion(q):
    """Return yaw from a geometry_msgs Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw):
    """Return a geometry_msgs-style z/w tuple for a yaw-only quaternion."""
    half_yaw = yaw * 0.5
    return math.sin(half_yaw), math.cos(half_yaw)


class OdomBridge(Node):
    def __init__(self):
        super().__init__('odom_bridge')

        self.declare_parameter('input_odom_topic', '/Odometry')
        self.declare_parameter('output_odom_topic', '/odom')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('source_frame', 'camera_init')
        self.declare_parameter('source_child_frame', 'body')
        self.declare_parameter('project_to_2d', True)
        self.declare_parameter('publish_map_to_odom', True)
        self.declare_parameter('publish_map_to_source', True)
        self.declare_parameter('initial_tf_rate', 10.0)
        self.declare_parameter('tf_lookup_rate', 50.0)

        self.input_odom_topic = self.get_parameter(
            'input_odom_topic').get_parameter_value().string_value
        self.output_odom_topic = self.get_parameter(
            'output_odom_topic').get_parameter_value().string_value
        self.map_frame = self.get_parameter(
            'map_frame').get_parameter_value().string_value
        self.odom_frame = self.get_parameter(
            'odom_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter(
            'base_frame').get_parameter_value().string_value
        self.source_frame = self.get_parameter(
            'source_frame').get_parameter_value().string_value
        self.source_child_frame = self.get_parameter(
            'source_child_frame').get_parameter_value().string_value
        self.project_to_2d = self.get_parameter(
            'project_to_2d').get_parameter_value().bool_value
        self.publish_map_to_odom = self.get_parameter(
            'publish_map_to_odom').get_parameter_value().bool_value
        self.publish_map_to_source = self.get_parameter(
            'publish_map_to_source').get_parameter_value().bool_value
        initial_tf_rate = self.get_parameter(
            'initial_tf_rate').get_parameter_value().double_value
        tf_lookup_rate = self.get_parameter(
            'tf_lookup_rate').get_parameter_value().double_value

        # 订阅 FAST-LIO 的 /Odometry，仅复用 twist/covariance；位姿以 TF 为准。
        self.sub = self.create_subscription(
            Odometry, self.input_odom_topic, self.odom_callback, 10)

        # 发布 FAST-LIO odom 话题供 Nav2。
        self.odom_pub = self.create_publisher(Odometry, self.output_odom_topic, 10)

        # TF 广播器
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._have_pose = False
        self._first_pose_log = True
        self._last_odom = None

        # 先发布静态 TF，再持续发布初始 identity TF，直到 FAST-LIO 首帧到来。
        now = self.get_clock().now().to_msg()
        self._publish_static_transforms(now)
        self._publish_initial_odom_tf()
        timer_period = 1.0 / max(initial_tf_rate, 1.0)
        self.initial_tf_timer = self.create_timer(
            timer_period, self._publish_initial_odom_tf)
        tf_timer_period = 1.0 / max(tf_lookup_rate, 1.0)
        self.tf_timer = self.create_timer(
            tf_timer_period, self.publish_pose_from_fastlio_tf)

        self.get_logger().info('OdomBridge started')
        self.get_logger().info(
            f'  {self.input_odom_topic} -> {self.output_odom_topic}')
        self.get_logger().info(
            f'  Dynamic TF: {self.odom_frame}->{self.base_frame}')
        self.get_logger().info(
            f'  Pose source TF: {self.source_frame}->{self.source_child_frame}')
        self.get_logger().info(f'  Project to 2D: {self.project_to_2d}')

    def _publish_initial_odom_tf(self):
        """发布初始 identity TF odom->base_frame，让 Nav2 启动时不等待。"""
        if self._have_pose:
            return
        if not self._last_odom:
            stamp = self.get_clock().now().to_msg()
        else:
            stamp = self._last_odom.header.stamp
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

    def _publish_static_transforms(self, stamp):
        """发布静态 TF (只在启动时发布一次)"""
        transforms = []
        if self.publish_map_to_odom:
            transforms.append(self._make_identity_static(
                stamp, self.map_frame, self.odom_frame))
        if self.publish_map_to_source:
            transforms.append(self._make_identity_static(
                stamp, self.map_frame, self.source_frame))

        if transforms:
            self.static_broadcaster.sendTransform(transforms)

    @staticmethod
    def _make_identity_static(stamp, parent_frame, child_frame):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        return t

    def publish_pose_from_fastlio_tf(self):
        try:
            source_tf = self.tf_buffer.lookup_transform(
                self.source_frame,
                self.source_child_frame,
                Time())
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            return

        position = source_tf.transform.translation
        orientation = source_tf.transform.rotation
        yaw = yaw_from_quaternion(orientation)
        yaw_z, yaw_w = quaternion_from_yaw(yaw)

        if self._first_pose_log:
            nav_z = 0.0 if self.project_to_2d else position.z
            self.get_logger().info(
                f'First {self.source_frame}->{self.source_child_frame} received: '
                f'fast_lio_pos=({position.x:.2f}, {position.y:.2f}, {position.z:.2f}), '
                f'nav2_pos=({position.x:.2f}, {position.y:.2f}, {nav_z:.2f}), '
                f'yaw={yaw:.3f}')
            self._first_pose_log = False
            self._have_pose = True
            self.initial_tf_timer.cancel()

        # --- 发布 TF: odom -> base_frame ---
        t = TransformStamped()
        t.header.stamp = source_tf.header.stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = position.x
        t.transform.translation.y = position.y
        t.transform.translation.z = 0.0 if self.project_to_2d else position.z
        if self.project_to_2d:
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = yaw_z
            t.transform.rotation.w = yaw_w
        else:
            t.transform.rotation = orientation
        self.tf_broadcaster.sendTransform(t)

        # --- 发布 FAST-LIO odom 话题 ---
        odom = Odometry()
        odom.header.stamp = source_tf.header.stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        if self._last_odom is not None:
            odom.pose = deepcopy(self._last_odom.pose)
            odom.twist = deepcopy(self._last_odom.twist)
        odom.pose.pose.position.x = position.x
        odom.pose.pose.position.y = position.y
        odom.pose.pose.position.z = position.z
        odom.pose.pose.orientation = deepcopy(orientation)
        if self.project_to_2d:
            odom.pose.pose.position.z = 0.0
            odom.pose.pose.orientation.x = 0.0
            odom.pose.pose.orientation.y = 0.0
            odom.pose.pose.orientation.z = yaw_z
            odom.pose.pose.orientation.w = yaw_w
        self.odom_pub.publish(odom)

    def odom_callback(self, msg):
        """保存 FAST-LIO /Odometry 的 twist/covariance，位姿发布以 TF 为准。"""
        self._last_odom = msg


def main(args=None):
    rclpy.init(args=args)
    node = OdomBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

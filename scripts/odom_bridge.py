#!/usr/bin/env python3
"""
FAST-LIO → Nav2 Odometry Bridge (v2 - Robust)

功能：
  1. 启动时立即发布静态 TF: map→odom, map→camera_init (identity)
  2. 启动时立即发布初始 identity TF: odom→base_link (防止 Nav2 等待)
  3. 订阅 FAST-LIO 的 /Odometry (camera_init→body)
     → 更新动态 TF: odom→base_link
     → 重发布为 /odom (odom→base_link)

TF 树:
  map ──(static identity)──→ odom ──(dynamic, FAST-LIO pose)──→ base_link
  map ──(static identity)──→ camera_init ──(FAST-LIO TF)──→ body

注意：body→base_link 的连接通过 robot_state_publisher 的 URDF 自动完成
      (body 与 imu_link 对齐, imu_link→base_link 由 URDF 定义)
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros


class OdomBridge(Node):
    def __init__(self):
        super().__init__('odom_bridge')

        # 订阅 FAST-LIO 的 /Odometry
        self.sub = self.create_subscription(
            Odometry, '/Odometry', self.odom_callback, 10)

        # 发布 /odom 话题供 Nav2
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # TF 广播器
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # 先发布初始 identity TF，再发布静态 TF
        now = self.get_clock().now().to_msg()
        self._publish_initial_odom_tf(now)
        self._publish_static_transforms(now)

        self.get_logger().info('OdomBridge v2 started')
        self.get_logger().info('  Static TFs: map->odom, map->camera_init')
        self.get_logger().info('  Dynamic: odom->base_footprint from /Odometry')

        # 用于第一次接收 /Odometry 后的日志
        self._first_odom = True

    def _publish_initial_odom_tf(self, stamp):
        """发布初始 identity TF odom→base_footprint，让 Nav2 启动时不等待"""
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info('  Initial identity TF: odom->base_footprint')

    def _publish_static_transforms(self, stamp):
        """发布静态 TF (只在启动时发布一次)"""
        # 1. map → odom (identity)
        t1 = TransformStamped()
        t1.header.stamp = stamp
        t1.header.frame_id = 'map'
        t1.child_frame_id = 'odom'
        t1.transform.translation.x = 0.0
        t1.transform.translation.y = 0.0
        t1.transform.translation.z = 0.0
        t1.transform.rotation.w = 1.0

        # 2. map → camera_init (identity)
        t2 = TransformStamped()
        t2.header.stamp = stamp
        t2.header.frame_id = 'map'
        t2.child_frame_id = 'camera_init'
        t2.transform.translation.x = 0.0
        t2.transform.translation.y = 0.0
        t2.transform.translation.z = 0.0
        t2.transform.rotation.w = 1.0

        self.static_broadcaster.sendTransform([t1, t2])

    def odom_callback(self, msg):
        """收到 FAST-LIO 的 /Odometry → 更新 TF 和 /odom"""
        # --- 日志: 第一次收到 ---
        if self._first_odom:
            self.get_logger().info(
                f'First /Odometry received: '
                f'pos({msg.pose.pose.position.x:.2f}, '
                f'{msg.pose.pose.position.y:.2f}, '
                f'{msg.pose.pose.position.z:.2f})')
            self._first_odom = False

        # --- 发布 TF: odom → base_footprint ---
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

        # --- 发布 /odom 话题 ---
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose = msg.pose
        odom.twist = msg.twist
        self.odom_pub.publish(odom)


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

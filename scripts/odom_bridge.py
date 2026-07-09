#!/usr/bin/env python3
"""
FAST-LIO → Nav2 Odometry Bridge

Subscribes to FAST-LIO's /Odometry (frame: camera_init → body)
Publishes:
  - /odom topic for Nav2 (frame: odom → base_link)
  - TF: odom → base_link (from FAST-LIO pose)
  - Static TF: map → odom (identity)
  - Static TF: map → camera_init (identity)
  - Static TF: body → imu_link (identity, connects FAST-LIO body to URDF tree)

This bridges FAST-LIO's SLAM output into Nav2's expected TF tree:
  map ──(id)──→ odom ──(FAST-LIO)──→ base_link
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros


class OdomBridge(Node):
    def __init__(self):
        super().__init__('odom_bridge')

        # Subscriber to FAST-LIO's /Odometry
        self.sub = self.create_subscription(
            Odometry, '/Odometry', self.odom_callback, 10)

        # Publisher for Nav2 /odom
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # TF broadcasters
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # Publish static transforms (only once)
        self.publish_static_transforms()

        self.get_logger().info('OdomBridge started: '
                               'FAST-LIO /Odometry → /odom + TF')

    def publish_static_transforms(self):
        now = self.get_clock().now().to_msg()

        # 1. Static TF: map → odom (identity)
        #    Since FAST-LIO is drift-free SLAM, map and odom are the same
        t1 = TransformStamped()
        t1.header.stamp = now
        t1.header.frame_id = 'map'
        t1.child_frame_id = 'odom'
        t1.transform.translation.x = 0.0
        t1.transform.translation.y = 0.0
        t1.transform.translation.z = 0.0
        t1.transform.rotation.w = 1.0

        # 2. Static TF: map → camera_init (identity)
        t2 = TransformStamped()
        t2.header.stamp = now
        t2.header.frame_id = 'map'
        t2.child_frame_id = 'camera_init'
        t2.transform.translation.x = 0.0
        t2.transform.translation.y = 0.0
        t2.transform.translation.z = 0.0
        t2.transform.rotation.w = 1.0

        # 3. Static TF: body → imu_link (identity)
        #    FAST-LIO's body frame is the IMU frame, URDF has imu_link
        t3 = TransformStamped()
        t3.header.stamp = now
        t3.header.frame_id = 'body'
        t3.child_frame_id = 'imu_link'
        t3.transform.translation.x = 0.0
        t3.transform.translation.y = 0.0
        t3.transform.translation.z = 0.0
        t3.transform.rotation.w = 1.0

        self.static_broadcaster.sendTransform([t1, t2, t3])

    def odom_callback(self, msg):
        """Convert FAST-LIO /Odometry to Nav2 /odom + TF"""
        # --- Republish odometry topic ---
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose = msg.pose
        odom.twist = msg.twist
        self.odom_pub.publish(odom)

        # --- Publish TF: odom → base_link ---
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)


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

#!/usr/bin/env python3
"""
vertical velocity estimator using a complementary filter.

subscribes to:
    /imu/data  (sensor_msgs/Imu)    - IMU linear acceleration in m/s^2
    /depth     (std_msgs/Float32)       - depth reading in meters

publishes to:
    /vertical_velocity (std_msgs/Float32) - estimated vertical velocity in m/s
                                            positive = descending
                                            negative = ascending

how it works:
    depth differentiation alone is noisy. IMU integration alone drifts over time.
    a complementary filter combines both, making use of each's advantages, 
    while disregarding their disadvantages, since IMU handles fast changes, and
    depth derivative corrects long term drift.

        v_depth = (depth_now - depth_prev) / dt
        v_imu   = v_prev + (a_z - 9.81) * dt
        v_fused = alpha * v_imu + (1 - alpha) * v_depth

parameters:
    alpha   (float, default 0.98) - filter coefficient/constant. a high alpha means we trust the IMU more at fast timescales.
                                    a low alpha means we trust the depth derivative more (this would be laggy but drift free).
    max_dt  (float, default 1.0)  - reject gaps larger than these many seconds.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32

#defining acceleration due to gravity. we'll subtract this from the IMU's Z axis later.
GRAVITY = 9.81


class FusedDataNode(Node):

    def __init__(self):
        super().__init__("fused_data")
        
        #some of the tunable parameters
        self.declare_parameter("alpha", 0.98)
        self.declare_parameter("max_dt", 1.0)
        self.alpha = self.get_parameter("alpha").value
        self.max_dt = self.get_parameter("max_dt").value

        #the filter state
        self.v_fused = 0.0
        self.prev_depth = None
        self.prev_time = None

        #the latest IMU reading will be stored here until a depth message arrives
        self.latest_imu = None

        #the subscribers and publisher
        self.create_subscription(Imu, "/imu/data", self.imu_callback, 10)
        self.create_subscription(Float32, "/depth", self.depth_callback, 10)
        self.vel_pub = self.create_publisher(Float32, "/vertical_velocity", 10)

        self.get_logger().info("fused_data node started.")

    def imu_callback(self, msg):
        #we'll just store the latest IMU message. the sensor fusion happens in depth_callback.
        self.latest_imu = msg

    def depth_callback(self, msg):
        #we'll need an IMU reading before we can fuse anything
        if self.latest_imu is None:
            return

        current_depth = msg.data
        current_time = self.get_clock().now().nanoseconds * 1e-9

        #the first message. we initialise the state and wait for the next one
        if self.prev_time is None:
            self.prev_depth = current_depth
            self.prev_time = current_time
            return

        dt = current_time - self.prev_time

        #we'll skip readings if time gap is invalid or too large (eg. after a sensor dropout)
        if dt <= 0.0 or dt > self.max_dt:
            self.get_logger().warn(f"Skipping: dt={dt:.3f}s out of range. Resetting.")
            self.prev_depth = current_depth
            self.prev_time = current_time
            self.v_fused = 0.0
            return

        #the depth derived velocity (noisy but drift free)
        v_depth = (current_depth - self.prev_depth) / dt

        #the IMU derived velocity (smooth but drifts if not corrected)
        #we'll subtract gravity because the IMU reads 9.81 m/s^2 at rest
        a_z = self.latest_imu.linear_acceleration.z - GRAVITY
        v_imu = self.v_fused + a_z * dt

        #the complementary filter
        self.v_fused = self.alpha * v_imu + (1.0 - self.alpha) * v_depth

        #we'll then publish
        out = Float32()
        out.data = self.v_fused
        self.vel_pub.publish(out)

        self.get_logger().debug(
            f"v_depth={v_depth:.3f}  v_imu={v_imu:.3f}  v_fused={self.v_fused:.3f} m/s")

        self.prev_depth = current_depth
        self.prev_time = current_time


def main(args=None):
    rclpy.init(args=args)
    node = FusedDataNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

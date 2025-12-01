#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float64MultiArray, Bool
from mavros_msgs.msg import Imu
import math                                                                                                                                                                                                        
from math import degrees

from tf_transformations import (
    euler_from_quaternion, quaternion_from_euler, quaternion_multiply
)
import geometry_msgs
                                                                                                                                                                                                                                                                                                                                                                                           


import time
import threading
from my_gyro_pkg.fog import State, reader_thread, CALIB_SECS

class FogNode(Node):
    def __init__(self):
        super().__init__('fog_node')
        self.get_logger().info("Starting FOG Node...")
        self.subscription = self.create_subscription(
            Bool,
            'fog/zero',
            self.listener_callback,
            10)
        self.pose_sub = self.create_subscription(
            Imu,
            '/mavros/IMU/Data',
            self.imu_callback,
            10)

        # Shared state
        self.state = State()

        # Start reader thread
        self.reader_thread = threading.Thread(target=reader_thread, args=("/dev/ttyUSB0", 912600, self.state), daemon=True)
        self.reader_thread.start()

        # Publishers
        #self.heading_pub = self.create_publisher(Float32, '/mavros/setpoint_raw/attitude', 10)
        self.heading_pub = self.create_publisher(Float32, 'fog/heading', 10)
        self.temp_pub = self.create_publisher(Float32, 'fog/temp', 10)
        self.vsup_pub = self.create_publisher(Float32, 'fog/vsup', 10)
        self.mag_pub = self.create_publisher(Float32, 'mavros/heading_degrees', 10)

        self.diff_pub = self.create_publisher(Float32, 'fog/heading_diff', 10)


        # Optional combined publisher
        self.all_pub = self.create_publisher(Float64MultiArray, 'fog/data', 10)

        # Timer for publishing at 20Hz
        self.create_timer(0.05, self.publish_data)

        self.offset = 0
        self.mag_heading = 0

    def imu_callback(self, msg):
        q = msg.orientation
        q_meas = [q.x, q.y, q.z, q.w]

        # OPTIONAL: apply a fixed mounting correction (here: 180° about X)
        # If you don't need this, just set q_corr = q_meas.
        q_fix  = quaternion_from_euler(math.pi, 0.0, 0.0)  # roll=180°
        q_corr = quaternion_multiply(q_meas, q_fix)        # change of body frame

        roll, pitch, yaw = euler_from_quaternion(q_corr)

        self.mag_heading= degrees(yaw)
        self.mag_pub.publish(Float32(data=self.mag_heading))

        self.get_logger().info(
            f"RPY (rad): roll={roll:.3f}, pitch={pitch:.3f}, yaw={yaw:.3f} | "
            f"deg: R={math.degrees(roll):.1f}, P={math.degrees(pitch):.1f}, Y={math.degrees(yaw):.1f}")
            
        self.get_logger().info(
            f"RPY (rad): Roll={roll:.3f}, Pitch={pitch:.3f}, Yaw={yaw:.3f}")

    def listener_callback(self, data):
        self.offset = -self.state.heading
        
    def publish_data(self):
        with self.state.lock:
            if not self.state.cal_done:
                self.get_logger().info(f"Calibrating FOG... {CALIB_SECS:.1f}s")
                return

            heading = self.state.heading + self.offset
            temp = self.state.temp_c
            vsup = self.state.v_sup
            #mag_heading = self.mag_heading
        # Individual messages
        self.heading_pub.publish(Float32(data=heading))
        self.temp_pub.publish(Float32(data=temp))
        self.vsup_pub.publish(Float32(data=vsup))
        self.diff_pub.publish(Float32(data=(self.mag_heading-heading)))
        
        # Combined data array
        msg = Float64MultiArray()
        msg.data = [heading, temp, vsup]
        self.all_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FogNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

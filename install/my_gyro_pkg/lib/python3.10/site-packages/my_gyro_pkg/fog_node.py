#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float64MultiArray, Bool
import math                                                                                                                                                                                                        
from math import degrees
import geometry_msgs
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, ReliabilityPolicy
                                                                                                                                                                                                                                                                                                                                                                                           
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
        
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pose_sub = self.create_subscription(
            Imu,
            '/mavros/imu/data',
            self.imu_callback,
            qos_profile)

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

        self.mag_heading = 0

    def imu_callback(self, msg):
        q = msg.orientation
        heading = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        msg = Float32()
        msg.data = math.degrees(heading)
        self.mag_pub.publish(msg)



    def listener_callback(self, data):
        pass
        
    def publish_data(self):
        with self.state.lock:
            if not self.state.cal_done:
               return

            heading = self.state.heading
            temp = self.state.temp_c
            vsup = self.state.v_sup
            #mag_heading = self.mag_heading
        # Individual messages
        self.heading_pub.publish(Float32(data=heading))
        self.temp_pub.publish(Float32(data=temp))
        self.vsup_pub.publish(Float32(data=vsup))
        self.diff_pub.publish(Float32(data=(abs(self.mag_heading-heading))))
        
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

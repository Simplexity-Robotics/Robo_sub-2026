import time

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool, SetMode
from geographic_msgs.msg import GeoPoseStamped

from rclpy.qos import QoSProfile, ReliabilityPolicy

class DepthHoldTest(Node):
    def __init__(self):
        super().__init__('depth_hold_test')
        time.sleep(5)

        # Define best effort with queue size of 1
        qos_profile = QoSProfile(
            depth=1,  # Queue depth
            reliability=ReliabilityPolicy.BEST_EFFORT  # Set reliability to best effort
        )

        # Create service clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # Publisher for depth setpoint
        self.sp_pub = self.create_publisher(GeoPoseStamped, "/mavros/setpoint_position/global", qos_profile)

        # Wait for services
        self.arming_client.wait_for_service()
        self.mode_client.wait_for_service()

        # Change mode to ALT_HOLD
        mode_req = SetMode.Request()
        mode_req.custom_mode = "ALT_HOLD"
        self.mode_client.call_async(mode_req)

        # Arm the vehiclehttps://discordapp.com/channels/1093601696887873546/1093601697529597994ro
        arm_req = CommandBool.Request()
        arm_req.value = False
        self.arming_client.call_async(arm_req)

        # Start sending depth setpoint at 100 Hz
        self.timer = self.create_timer(0.01, self.send_depth_sp)

        # Depth in meters (positive down)
        self.target_depth_m = -0.5 # 2 feet

        self.get_logger().info(f"Setting target depth to {self.target_depth_m}")

        self.subscription = self.create_subscription(
            GeoPoseStamped,
            'target/depth',
            self.listener_callback,
            10)

    def listener_callback(self,data):
        self.get_logger().info("CALLBACK")
        self.target_depth_m = data.pose.position.altitude
        self.get_logger().info(f"setting depth target to{data.pose.position.altitude}")

    def send_depth_sp(self):
        msg = GeoPoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg() # Set the timestamp to the current time
        msg.header.frame_id = 'map'
        msg.pose.position.altitude = self.target_depth_m
        self.sp_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DepthHoldTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
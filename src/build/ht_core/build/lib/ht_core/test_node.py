
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import PositionTarget

class TestNode(Node):
    def __init__(self):
        super().__init__('depth_hold_test')

        print('Create service clients') #Create service clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # Publisher for depth setpoint
        self.sp_pub = self.create_publisher(PositionTarget, '/mavros/setpoint_raw/local', 10)

        print( 'Wait for services')

        self.arming_client.wait_for_service()
        self.mode_client.wait_for_service()

        # Change mode to DEPTH_HOLD
        mode_req = SetMode.Request()
        mode_req.custom_mode = "ALT_HOLD"
        print('setDepthHold')
        self.mode_client.call_async(mode_req)

        # Arm the vehicle
        arm_req = CommandBool.Request()
        print('arming')
        arm_req.value = True
        self.arming_client.call_async(arm_req)

        # Start sending depth setpoint at 10 Hz
        self.timer = self.create_timer(0.1, self.send_depth_sp)

        # Depth in meters (positive down)
        self.target_depth_m = 0.61  # 2 feet

    def send_depth_sp(self):
        sp = PositionTarget()
        sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        # Ignore everything except Z position
        sp.type_mask = (
            PositionTarget.IGNORE_VX |
            PositionTarget.IGNORE_VY |
            PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW |
            PositionTarget.IGNORE_YAW_RATE
        )
        sp.position.x = 0.0  # Not moving in X
        sp.position.y = 0.0  # Not moving in Y
        sp.position.z = self.target_depth_m  # Positive down in NED
        self.sp_pub.publish(sp)

def main(args=None):
    rclpy.init(args=args)
    node = TestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
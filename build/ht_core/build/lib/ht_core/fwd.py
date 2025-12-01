import rclpy
from rclpy.node import Node
from mavros_msgs.msg import OverrideRCIn
import time
from mavros_msgs.srv import CommandBool, SetMode

#from mavros_msgs.msg import R

class RCOverrideNode(Node):
    def __init__(self):
        super().__init__('rc_override_node')
        self.pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.arming_client.wait_for_service()

        time.sleep(1)
        #self.arm()
        self.timer = self.create_timer(0.05, self.tick)  # 10 Hz
        # 18 channels; 0 means "ignore"
        self.channels = [1500]*18
        # self.channels[3:6] = [1650, 1650, 1650]        
        self.channels[2:3] = [1400]
        #self.channels[4:5] = [1650]
        
        
        arm_req = CommandBool.Request()
        arm_req.value = True
        self.arming_client.call_async(arm_req)
        # self.get_logger().info()

    def tick(self):
        msg = OverrideRCIn()        
        msg.channels = self.channels
        self.pub.publish(msg)
        self.get_logger().info(f'RC In: {msg.channels}')
   
def main():
    rclpy.init()
    node = RCOverrideNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        self.channels = [1500]*18
        msg = OverrideRCIn()        
        msg.channels = self.channels
        self.pub.publish(msg)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
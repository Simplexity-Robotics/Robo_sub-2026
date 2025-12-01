import time

import rclpy

from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from mavros_msgs.srv import CommandBool, CommandLong
from mavros_msgs.msg import OverrideRCIn

from geometry_msgs.msg import Twist, PoseStamped
from geographic_msgs.msg import GeoPoseStamped
# State machine class for handling various actions

SUCCESS = True
FAILED = False



class StateMachine:
    def __init__(self, mp_node):
        self.logger = mp_node.get_logger()
        self.node = mp_node
        self.cmd = Twist() # Used to control the velocities

    def print_success(self, message = "SUCCESS"):
        self.logger.info("\033[32m" + message + "\033[0m")

    def print_fail(self, message = "FAILED"):
        self.logger.info("\033[31m" + message + "\033[0m")
    
    def print_unknown(self, message):
        self.logger.info("\033[33m" + message + "\033[0m")
    
    def print(self, message):
        self.logger.info("\033[34m" + message + "\033[0m")


    def wait(self, time_s):
        self.print(f"WAITING FOR {time_s} SECONDS")
        time.sleep(time_s)
        self.print_success()
        return SUCCESS

    def arm(self):
        # Call the arm service
        self.logger.info("ARMING SUBMARINE")
        req = CommandBool.Request()
        req.value = True

        # Call service synchronously
        response = self.node.arm_service.call_async(req)
        rclpy.spin_until_future_complete(self.node, response, timeout_sec = 5)
        if response.result():
            self.print_success()
            return SUCCESS
        else:
            self.print_fail()
            return FAILED

    def fire_torpedo_left(self):
        self.logger.info("FIRING LEFT TORPEDO")
        req = CommandLong.Request()
        req.broadcast = False
        req.command  = 181
        req.confirmation = 0
        req.param1 = 5.0 #pin 16
        req.param2 = 1.0
        req.param3 = 0.0
        req.param4 = 0.0
        req.param5 = 0.0
        req.param6 = 0.0
        req.param7 = 0.0
        response = self.node.relay_service.call_async(req)
        rclpy.spin_until_future_complete(self.node, response, timeout_sec=5)
        if response.result():
            self.print_success()
            return SUCCESS
        else:
            self.print_fail()
            return FAILED




    def fire_torpedo_right(self):
        pass

    def dropper_left(self):
        pass

    def dropper_right(self):
        pass

    def dive(self,
             target_depth_m, 
             timeout_sec=15.0, 
             tolerance=0.1):
        self.print(f"SETTING DEPTH TARGET TO {target_depth_m}")

        msg = GeoPoseStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg() # Set the timestamp to the current time
        msg.header.frame_id = 'map'
        msg.pose.position.altitude = target_depth_m
        self.node.depth_pub.publish(msg)
        # NEED TO IMPLEMENT MAVROS DEPTH TARGET HERE

        # while (time.time() - start_time) < timeout_sec:
        #     if abs(self.node.depth - target_depth_m) < tolerance:
        #         self.print_success(f"TARGET DEPTH OF {target_depth_m} REACHED")
        #         self.print_success()
        #         return SUCCESS

        return SUCCESS
    
    
    # def target_buoy():
    #     dist = Float32()


    def move_forward(self, time_s):
        start_time = time.time()
        self.print(f"MOVING FORWARD FOR {time_s} SECONDS")    
        channels = [1500]*18
        channels[4:5] = [1650]
        msg = OverrideRCIn()        
        msg.channels = channels
        
        self.node.get_logger().info(f'RC In: {msg.channels}')
        while (time.time() - start_time) < time_s:
            self.node.override_pub.publish(msg)
            time.sleep(0.025)
        msg.channels = [1500]*18
        self.node.override_pub.publish(msg)
        



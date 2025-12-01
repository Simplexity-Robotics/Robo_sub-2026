import threading

from rclpy.executors import MultiThreadedExecutor
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from mavros_msgs.srv import CommandBool, CommandLong
from geographic_msgs.msg import GeoPoseStamped
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.msg import VfrHud

# Import Mission Here
from ht_core.missions.best_mission import best_mission
from ht_core.missions.alpha import alpha_mission
from ht_core.missions.bravo import bravo_mission
from ht_core.missions.bravo2 import bravo2_mission
from ht_core.missions.charlie import charlie_mission
from ht_core.missions.charlie2 import charlie2_mission
from ht_core.missions.delta import delta_mission

MISSION_LOOKUP = {
    "best_mission": best_mission,
}

class MissionNode(Node):
    def __init__(self):
        super().__init__("mission_planner_node")
        
        ##########################
        ### DECLARE PARAMETERS ###
        ##########################

        # Declare parameters for launch files here
        self.declare_parameter("do_our_best", True)
        self.declare_parameter("ideal_depth_m", 1.5)  # Meters
        self.declare_parameter("mission", "best_mission")  # Meters
 
        # Read Parameters
        self.do_our_best = self.get_parameter("do_our_best").get_parameter_value().bool_value
        self.ideal_depth_m = self.get_parameter("ideal_depth_m").value
        global MISSION
        MISSION = self.get_parameter("mission").get_parameter_value().string_value
        
        #################
        ### VARIABLES ###
        #################

        self.heading = 0  # Init variable
        self.depth = 0
        self.is_armed = False

        ##########################
        ### DECLARE PUBLISHERS ###
        ##########################
        
        # Zero FOG publisher


        # Trigger analog pins publisher

        # Depth target publisher
        self.depth_pub = self.create_publisher(GeoPoseStamped, '/target/depth', 10)
        self.override_pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)
        # publisher
        # Other publishers

        ###########################
        ### DECLARE SUBSCRIBERS ###
        ###########################
        
        # Fog subscriber
        self.fog_subscriber = self.create_subscription(Float32, '/fog/heading', self.fog_callback, 10)

        # Depth subscriber
        self.depth_subscriber = self.create_subscription(Float32, '/mavros/vfr_hud', self.depth_callback, 10)

        self.arm_subscriber = self.create_subscription(Float32, '/mavros/state', self.arm_callback, 10)

        #distance from target subscriber
 #       self.dist_subscriber = self.create_subscription(Float32, '/mavros/heading', self.dist_callback, 10)

        ########################
        ### DECLARE SERVICES ###
        ########################
        #arming service
        self.arm_service = self.create_client(CommandBool, '/mavros/cmd/arming')
        if not self.arm_service.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('/mavros/cmd/arming service not available')
        #Relay service
        self.relay_service = self.create_client(CommandLong, '/mavros/cmd/command')
        if not self.relay_service.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('/mavros/cmd/command service not available')
           
        
    def fog_callback(self, msg):
        # Update our self.heading variable so other things can access
        self.heading = msg.data 

    def depth_callback(self, msg):    
        self.depth = msg.data

    def arm_callback(self, msg):
        self.is_armed = msg.armed.data



def main(args=None):

    rclpy.init(args=args)
    node = MissionNode()

    # Use executer so we can call services in other threads
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # Start the executor in its own thread
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    # Create mission thread and pass node into it so it has access to all of
    # the subcriber and publishers and various variables such a ideal depth
    node.get_logger().info(f"STARTING MISSION: {MISSION}")
    mission_thread = threading.Thread(target=MISSION_LOOKUP[MISSION], args=(node,))
    mission_thread.start()

    try:
        # Wait for mission thread to finish
        mission_thread.join()
    except KeyboardInterrupt:
        sm.disarm()
    finally:
        try:
            executor.shutdown()
            node.destroy_node()
            rclpy.shutdown()
        except:
            pass


if __name__ == "__main__":
    main()

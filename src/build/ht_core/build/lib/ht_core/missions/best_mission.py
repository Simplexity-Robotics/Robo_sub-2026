from ht_core.state_machine import StateMachine

#  Create the mission with an argument 
def best_mission(mp_node):

    #assert mp_node.do_our_best == True

    sm = StateMachine(mp_node=mp_node)

    sm.wait(1)

    sm.arm()

    sm.dive(target_depth_m=-1.0) # Needs to be float

    sm.move_forward(3)
    
    sm.dive(0.0)
     
    mp_node.get_logger().info("MISSION COMPLETED")

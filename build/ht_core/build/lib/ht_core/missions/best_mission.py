from ht_core.state_machine import StateMachine


#  Create the mission with an argument 
def best_mission(mp_node):
    
    # self.node.logger=mp_node.get_logger()
    #assert mp_node.do_our_best == True

    sm = StateMachine(mp_node=mp_node)
    sm.wait(5)

    for i in range(10):
        sm.arm()
        sm.wait(0.5)
        mp_node.get_logger().info("ATTEMPTING ARM")
        if(mp_node.is_armed):
            mp_node.get_logger().info("ARM SUCCESS!!")
            break

    sm.wait(10)

    # sm.dive(target_depth_m=0.5) # Needs to be float

    # sm.fire_torpedo_left()

    sm.move_forward(13, 0)

    sm.turn(23)

    sm.move_forward(35, 0)

    # sm.dive(target_depth_m=-0.01)

    # sm.wait(10)
    

    # sm.roll(5)

    # sm.wait(2)

    # sm.dive(target_depth=0.0)

    # sm.yaw_right(90)

    sm.disarm()


    mp_node.get_logger().info("MISSION COMPLETED")

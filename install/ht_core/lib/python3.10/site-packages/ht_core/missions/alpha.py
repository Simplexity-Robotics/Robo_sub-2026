from ht_core.state_machine import StateMachine

def alpha_mission(mp_node):
    
    sm = StateMachine(mp_node=mp_node)

    sm.wait(1)

    #sm.arm()

    sm.wait(1)

    sm.fire_torpedo_left()

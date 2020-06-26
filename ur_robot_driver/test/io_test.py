#!/usr/bin/env python

import rospy
from ur_msgs.srv import SetIO
from ur_msgs.msg import IOStates

def main():
    rospy.init_node('io_testing_client')
    service_client = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)
    service_client.wait_for_service()

    maximum_messages = 5
    pin = 0

    service_client(1, pin, 0)
    messages = 0
    pin_state = True

    while(pin_state):
        if messages >= 5:
            return False
        io_state = rospy.wait_for_message('/ur_hardware_interface/io_states', IOStates)
        pin_state = io_state.digital_out_states[pin].state
        messages += 1

    service_client(1, pin, 1)
    messages = 0
    pin_state = False

    while(not pin_state):
        if messages >= 5:
            return False
        io_state = rospy.wait_for_message('/ur_hardware_interface/io_states', IOStates)
        pin_state = io_state.digital_out_states[pin].state
        messages += 1

    return True

if __name__ == '__main__':
    print(main())

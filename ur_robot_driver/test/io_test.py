#!/usr/bin/env python
import sys
import time
import unittest


import rospy
import rostopic


PKG = 'ur_robot_driver'
NAME = 'io_test'


from ur_msgs.srv import SetIO, SetIORequest
from ur_msgs.msg import IOStates


class IOTest(unittest.TestCase):
    def __init__(self, *args):
        super(IOTest, self).__init__(*args)
        rospy.init_node('io_test')

        timeout = 10

        self.service_client = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)
        try:
            self.service_client.wait_for_service(timeout)
        except rospy.exceptions.ROSException as err:
            self.fail(
                "Could not reach SetIO service. Make sure that the driver is actually running."
                " Msg: {}".format(err))

    def test_set_io(self):
        """Test to set an IO and check whether it has been set."""

        maximum_messages = 5
        pin = 0
        self.assertEqual(maximum_messages, 5)

        self.service_client(1, pin, 0)
        messages = 0
        pin_state = True

        while(pin_state):
            if messages >= maximum_messages:
                self.fail("Could not read desired state after {} messages.".format(maximum_messages))
            io_state = rospy.wait_for_message('/ur_hardware_interface/io_states', IOStates)
            pin_state = io_state.digital_out_states[pin].state
            messages += 1
        self.assertEqual(pin_state, 0)

        self.service_client(SetIORequest.FUN_SET_DIGITAL_OUT, pin, 1)
        messages = 0
        pin_state = False

        while(not pin_state):
            if messages >= maximum_messages:
                self.fail("Could not read desired state after {} messages.".format(maximum_messages))
            io_state = rospy.wait_for_message('/ur_hardware_interface/io_states', IOStates)
            pin_state = io_state.digital_out_states[pin].state
            messages += 1
        self.assertEqual(pin_state, 1)


if __name__ == '__main__':
    import rostest
    rostest.run(PKG, NAME, IOTest, sys.argv)

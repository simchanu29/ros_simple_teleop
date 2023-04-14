#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from .Interpreter import Interpreter


class Interpreter_string(Interpreter):
    def __init__(self, interpreter_info):
        super(Interpreter_string, self).__init__(interpreter_info)
        self.val = ''
        self.pub = rospy.Publisher(self._config['topic'], String, queue_size=1)

    # Override
    def process_input(self, val, cmd_type):
        self.cmd.val = str(val)

        # Send message
        self.send_msg()
        self.cmd.send = True

    def send_msg(self):
        msg = String()
        msg.data = self.cmd.val

        self.pub.publish(msg)
        self.cmd.send = False

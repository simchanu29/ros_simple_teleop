#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Vector3
# import Interpreter.Interpreter as Interpreter
from .Interpreter import Interpreter
# import Interpreter
import numpy as np


class Interpreter_twistVector3(Interpreter):
    def __init__(self, interpreter_info):
        super(Interpreter_twistVector3, self).__init__(interpreter_info)

        print('Initializing interpreter twistVector3')
        self.cmd.val = [0.0, 0.0, 0.0]
        self.pub = rospy.Publisher(self._config['topic'], Vector3, queue_size=1)
        print('created publisher on', self._config['topic'])

    # Override
    def process_input(self, val, cmd_type):
        if cmd_type == self.SLIDER:
            self.cmd.val[val[0]] = val[1]

        if cmd_type == self.BUTTON:
            if val[0] == self.ALL:
                for i in range(len(self.cmd.val)):
                    self.handle_key(i, val[1])
            else:
                self.handle_key(val[0], val[1])

        # Saturation
        self.cmd.val = np.clip(self.cmd.val, -1.0, 1.0)

    def handle_key(self, i, val):
        # BACK keyword
        if val == self.BACK and self.cmd.val[i] != 0.0:
            self.cmd.val[i] = max(min(-self.cmd.val[i] / abs(self.cmd.val[i]), 1.0), -1.0)
        # STOP keyword
        elif val == self.STOP:
            self.cmd.val[i] = 0.0
        # Cas classique
        else:
            self.cmd.val[i] += val * self._config['key_precision']

    def send_msg(self):
        msg = Vector3()

        msg.x = self.cmd.val[0] * self._config['range_lin']
        msg.y = self.cmd.val[1] * self._config['range_lin']
        msg.z = self.cmd.val[2] * self._config['range_ang']

        self.pub.publish(msg)

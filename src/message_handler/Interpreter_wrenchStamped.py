#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float64
from .Interpreter import Interpreter
import numpy as np


class Interpreter_wrenchStamped(Interpreter):
    def __init__(self, interpreter_info):
        super(Interpreter_wrenchStamped, self).__init__(interpreter_info)
        self.cmd.val = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.pub = rospy.Publisher(self._config['topic'], WrenchStamped, queue_size=1)
        self.pub_range_lin = rospy.Publisher("/range_lin",Float64,queue_size=1)

    # Override
    def process_input(self, val, cmd_type):
        if val[1] != self.NULL:
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

        if val[2] != self.NULL:
            self._config['range_lin'] = np.clip(self._config['range_lin'] + val[2], self._config['range_lin_min'], self._config['range_lin_max'])
            rospy.loginfo('Wrench range_lin is {}'.format(self._config['range_lin']))
            self.pub_range_lin.publish(self._config['range_lin'])
        if val[3] != self.NULL:
            self._config['range_ang'] = np.clip(self._config['range_ang'] + val[3], self._config['range_ang_min'], self._config['range_ang_max'])
            rospy.loginfo('Wrench range_ang is {}'.format(self._config['range_ang']))

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
        msg = WrenchStamped()

        msg.wrench.force.x = self.cmd.val[0] * self._config['range_lin']
        msg.wrench.force.y = self.cmd.val[1] * self._config['range_lin']
        msg.wrench.force.z = self.cmd.val[2] * self._config['range_lin']

        msg.wrench.torque.x = self.cmd.val[3] * self._config['range_ang']
        msg.wrench.torque.y = self.cmd.val[4] * self._config['range_ang']
        msg.wrench.torque.z = self.cmd.val[5] * self._config['range_ang']

        msg.header.stamp = rospy.get_rostime()

        self.pub.publish(msg)

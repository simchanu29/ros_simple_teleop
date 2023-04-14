#!/usr/bin/env python3

# coding=utf-8

import rospy
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Joy


class Enabler:
    def __init__(self, input_switch_teleop):
        self._input_switch_teleop = input_switch_teleop
        self._enable = False
        rospy.logwarn('TELEOP UNARMED')


    def key_cb(self, msg):
        # rospy.loginfo("msg={}".format(msg.data))
        # rospy.loginfo("self._input_switch_teleop['key']={}".format(self._input_switch_teleop['key']))
        if msg.data in self._input_switch_teleop['key']:
            self.toogle_enable()

    def joy_cb(self, msg):
        for i, val in enumerate(msg.axes):
            if i in self._input_switch_teleop['joy_axes'] and val != 0:
                self.toogle_enable()

        for i, val in enumerate(msg.buttons):
            if i in self._input_switch_teleop['joy_buttons'] and val != 0:
                self.toogle_enable()

    def toogle_enable(self):
        self._enable = not self._enable

        if self._enable:
            rospy.logwarn('TELEOP ARMED')
        else:
            rospy.logwarn('TELEOP UNARMED')

        enable_pub.publish(Bool(self._enable))


def extract_input():
    # Config
    interpreter_kw = 'switch_teleop'
    called_interpreter_kw = 'called_interpreter'
    # config_prefix = 'teleop_config/'
    config_prefix = ''

    input_switch_teleop = {'key': [], 'joy_axes': [], 'joy_buttons': []}

    for key in rospy.get_param(config_prefix+'teleop_key_map'): # find switch teleop key
        if isinstance(rospy.get_param(config_prefix+'teleop_key_map'), dict):
            if rospy.get_param(config_prefix+'teleop_key_map')[key][called_interpreter_kw] == interpreter_kw:
                input_switch_teleop['key'].append(key)

    for axe in rospy.get_param(config_prefix+'teleop_joy_map/axes'): # find switch teleop key
        if isinstance(rospy.get_param(config_prefix+'teleop_joy_map/axes'), dict):
            if rospy.get_param(config_prefix+'teleop_joy_map')['axes'][axe][called_interpreter_kw] == interpreter_kw:
                input_switch_teleop['joy_axes'].append(int(axe))

    for button in rospy.get_param(config_prefix+'teleop_joy_map/buttons'): # find switch teleop key
        if isinstance(rospy.get_param(config_prefix+'teleop_joy_map/buttons'), dict):
            if rospy.get_param(config_prefix+'teleop_joy_map')['buttons'][button][called_interpreter_kw] == interpreter_kw:
                input_switch_teleop['joy_buttons'].append(int(button))

    return input_switch_teleop


if __name__ == '__main__':
    rospy.init_node('joy_interpreter')

    # Dynamic import
    enabler = Enabler(extract_input())

    # Pub sub
    enable_pub = rospy.Publisher('enable', Bool, queue_size=5, tcp_nodelay=True)
    key_sub = rospy.Subscriber('keys', String, enabler.key_cb, tcp_nodelay=True)
    joy_sub = rospy.Subscriber('joy', Joy, enabler.joy_cb, tcp_nodelay=True)

    rospy.spin()

#!/usr/bin/env python
# coding=utf-8

import rospy
from std_msgs.msg import String, Bool

class Enabler:
    def __init__(self, key_switch_teleop):
        self._key_switch_teleop = key_switch_teleop
        self._enable = True

    def enable_cb(self, msg):
        if msg.data == self._key_switch_teleop:
            self._enable = not self._enable

            if self._enable:
                rospy.logwarn('TELEOP ARMED')
            else:
                rospy.logwarn('TELEOP UNARMED')

            enable_pub.publish(Bool(self._enable))

if __name__ == '__main__':
    rospy.init_node('joy_interpreter')

    # Param server

    key_switch_teleop = ''
    for key in rospy.get_param('key_config/key_map'): # find switch teleop key
        if rospy.get_param('key_config/key_map')[key]['called_interpreter'] == 'switch_teleop':
            key_switch_teleop = key

    # Dynamic import
    enabler = Enabler(key_switch_teleop)

    # Pub sub
    enable_pub = rospy.Publisher('enable', Bool, queue_size=5)
    rospy.Subscriber('keys', String, enabler.enable_cb)

    rospy.spin()

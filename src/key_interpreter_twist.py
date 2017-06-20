#!/usr/bin/env python
# coding=utf-8
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import interpreter_callback.interpreter_callback as interpreter_callback


class key_interpreter():
    def __init__(self):
        self.key_map = rospy.get_param('key_map')
        self.topic_map = rospy.get_param('topic_map')
        self.cmd = interpreter_callback.Command()

    def keys_cb(self, msg, twist_pub):
            send, self.cmd = interpreter_callback.process(msg, self.key_map, 'twist', self.cmd)

            if send:
                # Creation du message twist
                print 'cmd motors :  ', self.cmd.wrench
                t = Twist()

                t.angular.x = 0
                t.angular.y = 0
                t.angular.z = self.cmd.twist[2] * self.topic_map['twist']['range_ang']

                t.linear.x = self.cmd.twist[0] * self.topic_map['twist']['range_lin']
                t.linear.y = self.cmd.twist[1] * self.topic_map['twist']['range_lin']
                t.linear.z = 0

                # Publish
                twist_pub.publish(t)


if __name__ == '__main__':
    ki = key_interpreter()
    rospy.init_node('keys_to_twist')

    twist_pub = rospy.Publisher('cmd_twist', Twist, queue_size=1)

    rospy.Subscriber('keys', String, ki.keys_cb, twist_pub)

    rospy.spin()

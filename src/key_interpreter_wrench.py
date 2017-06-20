#!/usr/bin/env python
# coding=utf-8
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int8
from geometry_msgs.msg import Wrench
import interpreter_callback.interpreter_callback as interpreter_callback

class key_interpreter():
    def __init__(self):
        self.key_map = rospy.get_param('key_map')
        self.topic_map = rospy.get_param('topic_map')
        self.cmd = interpreter_callback.Command()

    def keys_cb(self, msg, wrench_pub):
            send, self.cmd = interpreter_callback.process(msg, self.key_map, 'wrench', self.cmd)

            if send:
                # Creation du message twist
                print 'cmd motors :  ', self.cmd.wrench
                t = Wrench()

                t.torque.x = 0
                t.torque.y = 0
                t.torque.z = self.cmd.wrench[2] * self.topic_map['wrench']['range_ang']

                t.force.x = self.cmd.wrench[0] * self.topic_map['wrench']['range_lin']
                t.force.y = self.cmd.wrench[1] * self.topic_map['wrench']['range_lin']
                t.force.z = 0

                # Publish
                wrench_pub.publish(t)

if __name__ == '__main__':
    ki = key_interpreter()
    rospy.init_node('keys_to_wrench')

    wrench_pub = rospy.Publisher('cmd_wrench', Wrench, queue_size=1)

    rospy.Subscriber('keys', String, ki.keys_cb, wrench_pub)

    rospy.spin()

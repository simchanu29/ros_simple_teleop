#!/usr/bin/env python
# coding=utf-8
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from tf.transformations import quaternion_from_euler


class joy_interpreter():
    def __init__(self, interpreter_info):
        self.config = interpreter_info
        self.twist = Twist()

    def joy_cb(self, msg):
            buttons = msg.buttons
            axes = msg.axes

            # Gestion des axes
            for i in range(len(axes)):
                i_str = str(i)
                if self.config['axes'][i_str]['type'] == "axes_button":
                    cmd_pub.publish(self.config['axes'][i_str]['key'])

                # Pour chaque config de stick
                for stick in self.config['config']:

                    # Si cette config correspond au type de l'entree examinee
                    if self.config['axes'][i_str]['type'] == stick:
                        config_stick = self.config['config'][stick]
                        for axis in config_stick:

                            # Si l'entree correspond au bon axe
                            if self.config['axes'][i_str]['axe'] == axis:

                                twist_axis_to_fill = config_stick[axis]['twist_axis']
                                config_gain = self.config['config'][stick][axis]['gain']

                                # print("filling axis {} on entry {}".format(twist_axis_to_fill, i_str))

                                if twist_axis_to_fill == 'x':
                                    self.twist.linear.x = axes[i] * float(config_gain)
                                if twist_axis_to_fill == 'y':
                                    self.twist.linear.y = axes[i] * float(config_gain)
                                if twist_axis_to_fill == 'z':
                                    self.twist.linear.z = axes[i] * float(config_gain)
                                if twist_axis_to_fill == 'roll':
                                    self.twist.angular.x = axes[i] * float(config_gain)
                                if twist_axis_to_fill == 'pitch':
                                    self.twist.angular.y = axes[i] * float(config_gain)
                                if twist_axis_to_fill == 'yaw':
                                    self.twist.angular.z = axes[i] * float(config_gain)

            # Gestion des boutons
            for i in range(len(buttons)):
                if buttons[i] == 1:
                    pass
                    cmd_pub.publish(self.config['buttons'][str(i)]['key'])

            # print(self.twist)
            twist_pub.publish(self.twist)


if __name__ == '__main__':
    rospy.init_node('joy_interpreter')

    # Param server
    joy_map = rospy.get_param('key_config/joystick')
    stick_topic = rospy.get_param('~stick_topic', 'cmd_vel')
    print 'joy_map :', joy_map

    ji = joy_interpreter(joy_map)

    # Pub sub
    cmd_pub = rospy.Publisher('keys', String, queue_size=100)
    twist_pub = rospy.Publisher(stick_topic, Twist, queue_size=10)
    rospy.Subscriber('joy', Joy, ji.joy_cb)

    rospy.spin()

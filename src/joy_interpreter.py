#!/usr/bin/env python
# coding=utf-8
import rospy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Joy
from tf.transformations import quaternion_from_euler

class joy_interpreter():
    def __init__(self, interpreter_info):
        self.config = interpreter_info
        self.cmd = Msg_class()
        self._enable = True

    def extract_cmd(self):
        cmd_lin = None
        cmd_ang = None
        if out_type == 'Wrench':
            cmd_lin = self.cmd.force
            cmd_ang = self.cmd.torque
        elif out_type == 'Twist':
            cmd_lin = self.cmd.linear
            cmd_ang = self.cmd.angular
        elif out_type == 'WrenchStamped':
            cmd_lin = self.cmd.wrench.force
            cmd_ang = self.cmd.wrench.torque
        elif out_type == 'TwistStamped':
            cmd_lin = self.cmd.twist.linear
            cmd_ang = self.cmd.twist.angular

        return cmd_lin, cmd_ang

    def update_cmd(self, cmd_lin, cmd_ang):
        if out_type == 'Wrench':
            self.cmd.force = cmd_lin
            self.cmd.torque = cmd_ang
        elif out_type == 'Twist':
            self.cmd.linear = cmd_lin
            self.cmd.angular = cmd_ang
        elif out_type == 'WrenchStamped':
            self.cmd.header.stamp = rospy.Time.now()
            self.cmd.wrench.force = cmd_lin
            self.cmd.wrench.torque = cmd_ang
        elif out_type == 'TwistStamped':
            self.cmd.header.stamp = rospy.Time.now()
            self.cmd.twist.linear = cmd_lin
            self.cmd.twist.angular = cmd_ang

    def enable_cb(self, msg):
        self._enable = msg.data

        # RAZ de la commande au cas où
        if not self._enable:
            cmd_lin = Vector3()
            cmd_ang = Vector3()
            self.update_cmd(cmd_lin, cmd_ang)
            twist_pub.publish()

    def joy_cb(self, msg):
        # Extractiondu message
        buttons = msg.buttons
        axes = msg.axes

        # Gestion de wrench ou twist
        cmd_lin, cmd_ang = self.extract_cmd()

        # Gestion des axes
        if self._enable:
            for i in range(len(axes)):
                i_str = str(i)
                if self.config['axes'][i_str]['type'] == "axes_button":
                    cmd_pub.publish(self.config['axes'][i_str]['key'])

                # Pour chaque config de stick
                for stick in self.config['sticks']:

                    # Si cette config correspond au type de l'entree examinee
                    if (self.config['axes'][i_str]['type'] == stick):
                        config_stick = self.config['sticks'][stick]
                        for axis in config_stick:

                            # Si l'entree correspond au bon axe
                            if self.config['axes'][i_str]['axe'] == axis:

                                cmd_axis_to_fill = config_stick[axis]['cmd_axis']
                                config_gain = self.config['sticks'][stick][axis]['gain']

                                # print("filling axis {} on entry {}".format(cmd_axis_to_fill, i_str))

                                if cmd_axis_to_fill == 'x':
                                    cmd_lin.x = axes[i] * float(config_gain)
                                if cmd_axis_to_fill == 'y':
                                    cmd_lin.y = axes[i] * float(config_gain)
                                if cmd_axis_to_fill == 'z':
                                    cmd_lin.z = axes[i] * float(config_gain)
                                if cmd_axis_to_fill == 'roll':
                                    cmd_ang.x = axes[i] * float(config_gain)
                                if cmd_axis_to_fill == 'pitch':
                                    cmd_ang.y = - axes[i] * float(config_gain)
                                if cmd_axis_to_fill == 'yaw':
                                    cmd_ang.z = - axes[i] * float(config_gain)

            # Stockage des valeurs
            self.update_cmd(cmd_lin, cmd_ang)

            # print(self.twist)
            twist_pub.publish(self.cmd)

        # Gestion des boutons
        for i in range(len(buttons)):
            if buttons[i] == 1:
                pass
                cmd_pub.publish(self.config['buttons'][str(i)]['key'])


if __name__ == '__main__':
    rospy.init_node('joy_interpreter')

    # Param server
    joy_map = rospy.get_param('joystick_config')
    stick_topic = rospy.get_param('~stick_topic', 'cmd_vel')

    # Dynamic import
    out_type = joy_map['sticks_config']['topic_type']
    Msg_module = __import__('geometry_msgs.msg', fromlist=[out_type])
    Msg_class = getattr(Msg_module, out_type)

    ji = joy_interpreter(joy_map)

    # Pub sub
    cmd_pub = rospy.Publisher('keys', String, queue_size=100)
    twist_pub = rospy.Publisher(stick_topic, Msg_class, queue_size=10)
    rospy.Subscriber('joy', Joy, ji.joy_cb)
    rospy.Subscriber('enable', Bool, ji.enable_cb)

    rospy.spin()

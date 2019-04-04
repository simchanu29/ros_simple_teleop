#!/usr/bin/env python
# coding=utf-8
import rospy
from pydoc import locate
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Joy
import numpy as np

class InterpreterInterface:
    def __init__(self, config):
        self._config = config
        self.interpreter = self.import_filler()

    def import_filler(self):
        interpreter_type = self._config['type']

        # Dynamic import
        # Ros is bound to import only
        obj_module = __import__('message_handler.Interpreter_'+interpreter_type, fromlist=['Interpreter_'+interpreter_type])
        Interpreter = getattr(obj_module, 'Interpreter_'+interpreter_type)
        print('import result :', Interpreter)

        interpreter_instance = Interpreter(self._config)
        # print('Return interpreter instance : ', interpreter_instance.name)
        return interpreter_instance

    def process_input(self, val, cmd_type):
        # Gère et publie la valeur sur le bon topic
        self.interpreter.process_input(val, cmd_type)


class InputRouter:
    def __init__(self, interpreters_config, key_config, joy_config):

        self._key_config = key_config
        self._joy_config = joy_config
        self._enable = True

        self.interpreters = {}
        # Ces interpreteurs sont associés à un script python. Il peut y avoir plusieurs instances nommées séparéments

        for interpreter_config in interpreters_config:
            # Crée l'objet interpreter et lui fournis sa config
            self.interpreters[interpreter_config] = InterpreterInterface(interpreters_config[interpreter_config])

    def route_key(self, key_msg):
        if self._enable:
            key = key_msg.data

            if key in self._key_config:
                called_interpreter = self._key_config[key]['called_interpreter']
                if called_interpreter != 'switch_teleop':
                    val = self._key_config[key]['value']

                    in_if = self.interpreters[called_interpreter]
                    in_if.process_input(val, in_if.interpreter.BUTTON)

    def route_joy(self, joy_msg):
        if self._enable:
            # print('enabled')
            joy_axes = joy_msg.axes
            joy_buttons = joy_msg.buttons
            for i, val in enumerate(joy_axes):
                i_str = str(i)
                if i_str in self._joy_config['axes']:
                    called_interpreter = self._joy_config['axes'][i_str]['called_interpreter']
                    if called_interpreter != 'switch_teleop':
                        in_if = self.interpreters[called_interpreter]
                        # Check if config specified axe as a button giving a single value
                        if 'input_type' in self._joy_config['axes'][i_str] and self._joy_config['axes'][i_str]['input_type'] == 'button':
                            if val != 0:
                                val = self._joy_config['axes'][i_str]['value']
                            in_if.process_input(val, in_if.interpreter.BUTTON)
                        else:
                            if 'value' in self._joy_config['axes'][i_str]:
                            #     mask_val = self._joy_config['axes'][i_str]['value']
                            #     tmp_val = [val]*len(mask_val)
                            #     val = np.multiply(mask_val, tmp_val)
                                tmp_val = val
                                val = list(self._joy_config['axes'][i_str]['value'])
                                val[1] = tmp_val*val[1]
                            in_if.process_input(val, in_if.interpreter.SLIDER)

            for i, val in enumerate(joy_buttons):
                i_str = str(i)
                if val != 0 and i_str in self._joy_config['buttons']:
                    called_interpreter = self._joy_config['buttons'][i_str]['called_interpreter']
                    if called_interpreter != 'switch_teleop':
                        val = self._joy_config['buttons'][i_str]['value']

                        in_if = self.interpreters[called_interpreter]
                        in_if.process_input(val, in_if.interpreter.BUTTON)

    def enable(self, msg):
        self._enable = msg.data


if __name__ == '__main__':
    rospy.init_node('input_interpreter')

    # lit les interpreteurs
    # interpreters_config = rospy.get_param('teleop_config/teleop_interpreters')
    # key_config = rospy.get_param('teleop_config/teleop_key_map')
    # joy_config = rospy.get_param('teleop_config/teleop_joy_map')
    interpreters_config = rospy.get_param('teleop_interpreters')
    key_config = rospy.get_param('teleop_key_map')
    joy_config = rospy.get_param('teleop_joy_map')

    router = InputRouter(interpreters_config, key_config, joy_config)

    rospy.Subscriber('keys', String, router.route_key)
    rospy.Subscriber('joy', Joy, router.route_joy)
    rospy.Subscriber('enable', Bool, router.enable)

    rospy.spin()

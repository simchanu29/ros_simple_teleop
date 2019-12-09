import rospy
from std_msgs.msg import Float32
from Interpreter import Interpreter
import numpy as np


class Interpreter_position_speed(Interpreter):
    def __init__(self, interpreter_info):
        super(Interpreter_position_speed, self).__init__(interpreter_info)
        self.cmd.val = 0.0  # position
        self.speed = 0.0
        self.pub = rospy.Publisher(self._config['topic'], Float32, queue_size=1)
        print('created publisher on', self._config['topic'])

    # Override
    def process_input(self, val, cmd_type):
        if cmd_type == self.SLIDER:
            self.speed = val

        if cmd_type == self.BUTTON:
            # BACK keyword
            if val == self.BACK and self.speed != 0.0:
                self.speed = max(min(-self.speed / abs(self.speed), 1.0), -1.0)
            # STOP keyword
            elif val == self.STOP:
                self.speed = 0.0
            # Cas classique
            else:
                self.speed += val * self._config['key_precision']

        # Saturation
        self.speed = np.clip(self.speed, -1.0, 1.0)

    def send_msg(self):

        cmd_val_tmp = self.cmd.val + self.speed * self._config['gain_speed']

        if cmd_val_tmp != self.cmd.val:
            self.cmd.val = cmd_val_tmp
            msg = Float32()
            min_cmd = float(self._config['min'])
            max_cmd = float(self._config['max'])
            range_cmd = (max_cmd - min_cmd)/2.0  # car de -1 a 1 ca fait range = 2
            offset = range_cmd + min_cmd

            msg.data = np.clip(self.cmd.val * range_cmd + offset, min_cmd, max_cmd)

            self.pub.publish(msg)

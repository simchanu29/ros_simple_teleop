import rospy
from std_msgs.msg import Float32
from Interpreter import Interpreter
import numpy as np


class Interpreter_position_incremental(Interpreter):
    def __init__(self, interpreter_info):
        super(Interpreter_position_incremental, self).__init__(interpreter_info)
        self.cmd.val = 0.0  # position
        self.pub = rospy.Publisher(self._config['topic'], Float32, queue_size=1)
        print('created publisher on', self._config['topic'])

    # Override
    def process_input(self, val, cmd_type):
        # BACK keyword
        if val == self.BACK and self.cmd.val != 0.0:
            self.cmd.val = max(min(-self.cmd.val / abs(self.cmd.val), 1.0), -1.0)
        # STOP keyword
        elif val == self.STOP:
            self.cmd.val = 0.0
        # Cas classique
        else:
            self.cmd.val += val * self._config['key_precision']

        # Saturation
        self.cmd.val = np.clip(self.cmd.val, -1.0, 1.0)

        self.cmd.send = True

    def send_msg(self):
        msg = Float32()
        min_cmd = float(self._config['min'])
        max_cmd = float(self._config['max'])
        range_cmd = (max_cmd - min_cmd)/2.0  # car de -1 a 1 ca fait range = 2
        offset = range_cmd + min_cmd

        msg.data = np.clip(self.cmd.val * range_cmd + offset, min_cmd, max_cmd)

        self.pub.publish(msg)
        self.cmd.send = False

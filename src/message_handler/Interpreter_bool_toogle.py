import rospy
from std_msgs.msg import Bool
from Interpreter import Interpreter


class Interpreter_bool_toogle(Interpreter):
    def __init__(self, interpreter_info):
        super(Interpreter_bool_toogle, self).__init__(interpreter_info)
        self.cmd.val = bool(self._config['init_val'])
        self.pub = rospy.Publisher(self._config['topic'], Bool, queue_size=1)

    # Override
    def process_input(self, val, cmd_type):
        self.cmd.val = not bool(self.cmd.val)
        self.cmd.send = True

    def send_msg(self):
        msg = Bool()
        msg.data = self.cmd.val

        self.pub.publish(msg)
        self.cmd.send = False

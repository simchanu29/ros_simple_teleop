import rospy
from std_msgs.msg import Int8
from Interpreter import Interpreter


class Interpreter_int8(Interpreter):
    def __init__(self, interpreter_info):
        super(Interpreter_int8, self).__init__(interpreter_info)
        print('Initializing interpreter int8')
        self.val = 0
        self.pub = rospy.Publisher(self._config['topic'], Int8, queue_size=1)
        print('created publisher on', self._config['topic'])

    # Override
    def process_input(self, val, cmd_type):
        print('val=', val)
        self.cmd.val = int(val)

        # Send message
        self.send_msg()

    def send_msg(self):
        msg = Int8()
        msg.data = self.cmd.val

        self.pub.publish(msg)

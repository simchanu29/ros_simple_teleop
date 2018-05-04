from std_msgs.msg import Int8
from interpreter_callback import CommandParent


class Command(CommandParent):
    def __init__(self):
        CommandParent.__init__(self)
        self.val = 0


def process_key(val, cmd, topic_info):
    return val


def fill_msg(cmd, topic_info):
    msg = Int8()
    msg.data = cmd.val

    return msg
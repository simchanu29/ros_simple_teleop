from std_msgs.msg import String
from interpreter_callback import CommandParent


class Command(CommandParent):
    def __init__(self):
        CommandParent.__init__(self)
        self.val = ""


def process_key(val, cmd, topic_info):
    return val


def fill_msg(cmd, topic_info):
    msg = String()
    msg.data = cmd.val

    return msg
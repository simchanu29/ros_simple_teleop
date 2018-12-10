from std_msgs.msg import Int8
from interpreter_callback import CommandParent


class Command(CommandParent):
    def __init__(self, interpreter_info):
        CommandParent.__init__(self)
        self.val = 0


def process_key(val, cmd, interpreter_info):
    return int(val)


def fill_msg(cmd, interpreter_info):
    msg = Int8()
    msg.data = cmd.val

    return msg

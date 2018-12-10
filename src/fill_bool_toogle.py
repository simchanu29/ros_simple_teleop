from std_msgs.msg import Bool
from interpreter_callback import CommandParent


class Command(CommandParent):
    def __init__(self, interpreter_info):
        CommandParent.__init__(self)
        self.val = interpreter_info['init_val']


def process_key(val, cmd, interpreter_info):
    """
    Inversion de la commande
    """
    return not bool(cmd)


def fill_msg(cmd, interpreter_info):
    msg = Bool()
    msg.data = cmd.val

    return msg

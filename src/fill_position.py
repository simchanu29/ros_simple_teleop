from std_msgs.msg import Float32
from interpreter_callback import CommandParent


class Command(CommandParent):
    def __init__(self):
        CommandParent.__init__(self)
        self.val = 0.0


def process_key(val, cmd, topic_info):
    if val is 'BACK':
        if cmd != 0.0:
            cmd = max(min(-cmd / abs(cmd), 1.0), -1.0)
        elif val is 'STOP':
            cmd = 0.0
    else:
        cmd += val / topic_info['precision']

    cmd = max(min(cmd, 1.0), -1.0)

    return cmd


def fill_msg(cmd, topic_info):
    msg = Float32()
    msg.data = cmd.val * topic_info['range']

    return msg

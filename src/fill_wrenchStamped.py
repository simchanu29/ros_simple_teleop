from geometry_msgs.msg import WrenchStamped
from interpreter_callback import CommandParent

class Command(CommandParent):
    def __init__(self, interpreter_info):
        CommandParent.__init__(self)
        self.val = [0.0, 0.0, 0.0]


def process_key(val, cmd, interpreter_info):
    for i in range(len(val)):
        if val[i] == 'BACK':
            if cmd[i] != 0.0:
                cmd[i] = max(min(-cmd[i] / abs(cmd[i]), 1.0), -1.0)
        elif val[i] == 'STOP':
            cmd[i] = 0.0
        else:
            cmd[i] += val[i] * interpreter_info['precision']

        cmd[i] = max(min(cmd[i], 1.0), -1.0)

    return cmd


def fill_msg(cmd, interpreter_info):
    msg = WrenchStamped()

    msg.wrench.torque.x = 0
    msg.wrench.torque.y = 0
    msg.wrench.torque.z = cmd.val[2] * interpreter_info['range_ang']

    msg.wrench.force.x = cmd.val[0] * interpreter_info['range_lin']
    msg.wrench.force.y = cmd.val[1] * interpreter_info['range_lin']
    msg.wrench.force.z = 0

    msg.header.stamp = cmd.time

    return msg

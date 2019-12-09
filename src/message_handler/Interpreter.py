

class Interpreter(object):
# class Interpreter:
    def __init__(self, config):
        # print('Initializing Parent Interpreter')
        self._config = config
        self.name = 'None'
        self.cmd = Command()

        # Global types for commands
        self.SLIDER = 0
        self.BUTTON = 1
        self.ALL = 'ALL'

        # Global keywords for config
        self.BACK = 'BACK'
        self.STOP = 'STOP'
        self.NULL = 'NULL'

    def process_input(self, val, cmd_type):
        print('Error : No process function defined for', val)

    def send_msg(self):
        print('Error : No send_msg function defined')


class Command:
    def __init__(self):
        # print('Initializing command')
        self.send = True
        self.val = None

# coding=utf-8


class CommandParent:
    def __init__(self):
        self.send = True
        self.time = 0


def get_val(msg, called_interpreter, key_map):

    # Gestion des erreurs : le message est vide, non reconnu ou pas du bon called_interpreter
    if len(msg.data) == 0 or not key_map.has_key(msg.data):
        print 'len(msg.data) == 0 : ', len(msg.data) == 0
        print 'key_map.has_key(msg.data) : ', key_map.has_key(msg.data)
        print 'unknown key : ', msg.data
        return None  # unknown key
    elif (key_map[msg.data]['called_interpreter'] != called_interpreter) \
            and (key_map[msg.data]['called_interpreter'] != 'switch_teleop'):
        # print "key that shouldn't be translated in this instance :", key_map[msg.data]['called_interpreter'], called_interpreter
        return None  # key that shouldn't be translated in this instance
    elif key_map[msg.data]['called_interpreter'] == 'switch_teleop':
        # Si c'est un message interne
        return 'switch_teleop'

    # Récupération des données
    val = key_map[msg.data]['value']
    print "received mgs.data : ", msg.data
    print 'vals : ', val

    return val

# coding=utf-8


class Command:

    def __init__(self):
        self.twist = [0.0, 0.0, 0.0]
        self.wrench = [0.0, 0.0, 0.0]
        self.string = ''
        self.manual = True


def process_string(val, cmd):
    pass

def process_vec3(val, cmd):
    for i in range(len(val)):
        if val[i] is 'r':
            if cmd[i] != 0.0:
                cmd[i] = max(min(-cmd[i] / abs(cmd[i]), 1), -1)
        else:
            cmd[i] += val[i] / 20.0

        cmd[i] = max(min(cmd[i], 1), -1)

    return cmd

def process_twist(val, cmd):
    return process_vec3(val, cmd.twist)


def process_wrench(val, cmd):
    return process_vec3(val, cmd.wrench)


def process(msg, key_map, topic, cmd):

    # Gestion des erreurs : le message est vide, non reconnu ou pas du bon topic
    if len(msg.data) == 0 or not key_map.has_key(msg.data):
        print 'len(msg.data) == 0 : ', len(msg.data) == 0
        print 'key_map.has_key(msg.data) : ', key_map.has_key(msg.data)
        print 'unknown key : ', msg.data
        return  # unknown key
    elif key_map[msg.data]['topic'] is not topic \
            and key_map[msg.data]['topic'] is not 'state':
        return  # key that shouldn't be translated in this instance

    # Récupération des données
    val = key_map[msg.data]['value']
    topic = key_map[msg.data]['topic']
    print "received mgs.data : ", msg.data
    print 'vals : ', val

    # Gestion des entrées
    if topic is "twist":
        cmd = process_twist(val, cmd)
    elif topic is "wrench":
        cmd = process_wrench(val, cmd)
    elif topic is "state":
        cmd.manual = val

    # Envoi de la commande si en manuel
    if cmd.manual:
        return True, cmd
    return False, None

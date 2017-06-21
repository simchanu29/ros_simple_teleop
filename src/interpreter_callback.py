# coding=utf-8


class CommandParent:
    def __init__(self):
        self.send = True


def get_val(msg, topic, key_map):

    # Gestion des erreurs : le message est vide, non reconnu ou pas du bon topic
    if len(msg.data) == 0 or not key_map.has_key(msg.data):
        print 'len(msg.data) == 0 : ', len(msg.data) == 0
        print 'key_map.has_key(msg.data) : ', key_map.has_key(msg.data)
        print 'unknown key : ', msg.data
        return None # unknown key
    elif (key_map[msg.data]['topic'] != topic) \
            and (key_map[msg.data]['topic'] != 'switch_teleop'):
        print "key that shouldn't be translated in this instance :", key_map[msg.data]['topic'], topic
        return None # key that shouldn't be translated in this instance
    elif key_map[msg.data]['topic'] == 'switch_teleop':
        # Si c'est un message interne
        return 'switch_teleop'

    # Récupération des données
    val = key_map[msg.data]['value']
    print "received mgs.data : ", msg.data
    print 'vals : ', val

    return val

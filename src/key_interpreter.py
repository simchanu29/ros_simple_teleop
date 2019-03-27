#!/usr/bin/env python
# coding=utf-8
import rospy
from pydoc import locate
from std_msgs.msg import String, Bool
import interpreter_callback


class key_interpreter():
    def __init__(self):
        self.cmd = Command(interpreter_info)
        self.cmd.time = rospy.Time.now()

    def enable_cb(self, msg):
        self.cmd.send = msg.data

    def keys_cb(self, msg, twist_pub):
        if self.cmd.send:
            val = interpreter_callback.get_val(msg, interpreter_name, key_map)
            print 'val :', val

            if val is not None and val is not 'switch_teleop':
                # Fonction process du fichier importé automatiquement a partir de la config
                self.cmd.val = process_key(val, self.cmd.val, interpreter_info)

                self.cmd.time = rospy.Time.now()
                msg = fill_msg(self.cmd, interpreter_info)

                # Publish
                cmd_pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('key_interpreter')

    # Param server
    topic_name = rospy.get_param('~topic_name')
    interpreter_name = rospy.get_param('~interpreter_name')
    print 'topic_name :', topic_name
    print 'interpreter_name :', interpreter_name
    key_map = rospy.get_param('key_config/key_map')
    interpreter_config_map = rospy.get_param('key_config/interpreter_config_map')
    print 'interpreter_config_map :', interpreter_config_map

    # Extraction des parametres
    interpreter_info = interpreter_config_map[interpreter_name]
    interpreter_types = rospy.get_param('key_config/interpreter_types')
    interpreter_type = interpreter_info['type']
    print 'interpreter_type :', interpreter_type
    import_str_msg_module = interpreter_types[interpreter_type]['import'][0]
    print 'import_str_msg_module :', import_str_msg_module
    import_str_msg_class = interpreter_types[interpreter_type]['import'][1]
    print 'import_str_msg_class :', import_str_msg_class
    import_str_filler = interpreter_type
    print 'import_str_filler :', import_str_filler

    # Dynamic import
    # Ros is bound to import only
    Msg_module = __import__(import_str_msg_module, fromlist=[import_str_msg_class])
    Msg_class = getattr(Msg_module, import_str_msg_class)
    print 'import result Cmd_Msg :', Msg_class
    # Locate works only here
    fill_msg = locate('fill_'+import_str_filler+'.fill_msg')
    print 'import result fill_msg :', fill_msg
    Command = locate('fill_' + import_str_filler + '.Command')
    print 'import result Command :', Command
    process_key = locate('fill_' + import_str_filler + '.process_key')
    print 'import result process_key :', process_key

    ki = key_interpreter()

    # Dynamic publisher
    prefix = rospy.get_param('teleop_prefix','')  # Global variable as a prefix
    localPrefix = rospy.get_param('~teleop_prefix', '')  # Private variable as a prefix
    cmd_pub = rospy.Publisher(prefix + localPrefix + topic_name, Msg_class, queue_size=100)

    rospy.Subscriber('keys', String, ki.keys_cb, cmd_pub)
    rospy.Subscriber('enable', Bool, ki.enable_cb)

    rospy.spin()

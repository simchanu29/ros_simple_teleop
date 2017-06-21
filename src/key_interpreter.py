#!/usr/bin/env python
# coding=utf-8
import rospy
from pydoc import locate
from std_msgs.msg import String
import interpreter_callback


class key_interpreter():
    def __init__(self):
        self.cmd = Command()

    def keys_cb(self, msg, twist_pub):
            val = interpreter_callback.get_val(msg, topic_name, key_map)
            if val is 'switch_teleop':
                self.cmd.send = not self.cmd.send
            elif val is not None:
                self.cmd.val = process_key(val, self.cmd.val, topic_info)

                if self.cmd.send:
                    msg = fill_msg(self.cmd, topic_info)

                    # Publish
                    cmd_pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('keys_to_twist')

    # Param server
    topic_name = rospy.get_param('~topic_name')
    key_map = rospy.get_param('key_map')
    topic_map = rospy.get_param('topic_map')

    # Extraction des parametres
    topic_info = topic_map[topic_name]
    topic_types = rospy.get_param('topic_types')
    topic_type = topic_info['type']
    print 'topic_type :', topic_type
    import_str_msg = topic_types[topic_type]['import']
    print 'import_str_msg :', import_str_msg
    import_str_filler = topic_type
    print 'import_str_filler :', import_str_filler

    # Dynamic import
    Cmd_Msg = locate(import_str_msg)
    print 'import result Cmd_Msg :', Cmd_Msg
    fill_msg = locate('fill_'+import_str_filler+'.fill_msg')
    print 'import result fill_msg :', fill_msg
    Command = locate('fill_' + import_str_filler + '.Command')
    print 'import result Command :', Command
    process_key = locate('fill_' + import_str_filler + '.process_key')
    print 'import result process_key :', process_key

    ki = key_interpreter()

    # Dynamic publisher
    cmd_pub = rospy.Publisher('cmd_'+topic_name, Cmd_Msg, queue_size=1)

    rospy.Subscriber('keys', String, ki.keys_cb, cmd_pub)

    rospy.spin()

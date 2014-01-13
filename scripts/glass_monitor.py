#!/usr/bin/env python
import rospy, rostopic
from std_msgs.msg import String

class GlassMonitor:


    def __init__(self):
      #init nodes and services
        rospy.init_node("glass_monitor")
        self.warn_pub =  rospy.Publisher('/glass_warn', String)
        #read lists
        input_list =  rospy.get_param('~monitors', None)
        self.monitor_list = self.process_input(input_list)
        


    def process_input(self, input_list):
        sub_list = []
        for monitor in input_list:
            msg_type = rostopic.get_topic_class(monitor['topic'])[0]
            if msg_type != None:
                sub_list.append(rospy.Subscriber(monitor['topic'],msg_type, self.callback, monitor))
                rospy.logerr('Started for topic %s', monitor['topic'])
            else:
                rospy.logerr("Cannot determine type of topic %s", monitor['topic'])
                
    def callback(self, msg, info):
        data = float(self.get_msg_data(msg, info['field']) )
        op = info['op']
        check_val = float(info['val'])
        if op == 'lt':
            if data < check_val:
                st = String(info['msg'])
                self.warn_pub.publish(st)

        elif op == 'gt':
            if data > check_val:
                st = String(info['msg'])
                self.warn_pub.publish(st)


        elif op == 'ne':
            if data != check_val:
                st = String(info['msg'])
                self.warn_pub.publish(st)

        elif op == 'eq':
            if data == check_val:
                st = String(info['msg'])
                self.warn_pub.publish(st)
        else:
            rospy.logerr("Invalid operation %s", info['op'])

        
    def get_msg_data(self, msg, key):
        '''Have to slpit the keys up and recursivly call to get
        the desired information out of all the messages'''
        idx = key.find('/')
        if idx  == -1: 
            return msg.__getattribute__(key)
        else:
        #split the key
            sub_key = key[idx+1:]
            key = key[:idx]
            msg = msg.__getattribute__(key)
            return self.get_msg_data(msg, sub_key)



if __name__ == '__main__':
    rm =GlassMonitor() 
    rospy.spin();

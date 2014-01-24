#!/usr/bin/env python
# copyright 2014 UNL Nimbus Lab 
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#     you may not use this file except in compliance with the License.
#    You may obtain a copy of the License at
#  
#        http://www.apache.org/licenses/LICENSE-2.0
#  
#    Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#   See the License for the specific language governing permissions and
#   limitations under the License.
import rospy, rostopic
from std_msgs.msg import String

class GlassMonitor:
    '''Class to monitor desired topics and feilds and send out messages
        when the value on that topic exceeds a defined threshold value
        in order to build the monitor it requires the needed information 
        to be input as a parameter when the node is launched
        The input contains a list of dictonaries that have each of the following defined

        topic: The rostopic to monitor
        field: The fully named field within the message to check
        op: Comparison to make can be one of the following: gt, lt, eq, ne,within, outside 
        val: The value to check against
        msg: The message to display when the check has been violated
       
        And example .launch input is below: 

        <node name="monitor" type="glass_monitor.py" pkg="ros_glass_tools" output="s
            <rosparam param="monitors">
            [
            { topic: '/a/pose', field: 'translation/x', op: 'gt', val : 2 , msg: 'UAV is too far to the left'},
            { topic: '/a/pose', field: 'translation/z', op: 'gt', val : 2 , msg: 'UAV is too high!'}
            ]
            </rosparam>
        </node>
    '''

    def __init__(self):
        #init nodes and services
        rospy.init_node("glass_monitor")
        self.warn_pub =  rospy.Publisher('/glass_warn', String)
        #read lists
        input_list =  rospy.get_param('~monitors', None)
        self.monitor_list = self.process_input(input_list)
        


    def process_input(self, input_list):
        '''this method takes the input parameter to create the monitors
            the topic must already be published so that this method 
            can determine the message type
        '''
        sub_list = []
        for monitor in input_list:
            msg_type = rostopic.get_topic_class(monitor['topic'])[0]
            #check to see we can get a valid message class
            #if so we create a subscriver and pass along the extra information
            #diectonary so comparisions can be made
            if msg_type != None:
                sub_list.append(rospy.Subscriber(monitor['topic'],msg_type, self.callback, monitor))
                rospy.logerr('Started for topic %s', monitor['topic'])
            else:
                rospy.logerr("Cannot determine type of topic %s", monitor['topic'])
        return sub_list
                
    def callback(self, msg, info):
        '''generic callback for the monitor topics
            this will get the desired comparison and check that value
            if it is violated than the node will publish warning message
            to the glass topic'''
        #get data from the message
        data = float(self.get_msg_data(msg, info['field']) )
        op = info['op']
        check_val = float(info['val'])
        #comparisons
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
        elif op == 'within':
            if abs(data) > check_val:
                st = String(info['msg'])
                self.warn_pub.publish(st)
        elif op == 'outside':
            if abs(data) < check_val:
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

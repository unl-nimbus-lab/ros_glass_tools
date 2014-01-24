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
import json, sys, re
import rospy, rostopic
from ros_glass_tools.srv import Topics




class ListTopics:
    '''Node to return a list of all running topics when service is called'''
    def __init__(self):
        rospy.init_node("list_topics_server")
        self.serv = rospy.Service("list_topics", Topics, self.list)




    def list(self, request):
        '''request for service handler return the list'''
        topics = self.get_ros_topics()
        return topics

    def get_ros_topics(self):
        '''return list of published topics'''
        try:
            topics = rospy.get_published_topics()
            return ' ,'.join((topic[0] for topic in topics))            
        except:
            rospy.logerr("no core active")
            return False 




if __name__ == '__main__':
    lt = ListTopics()
    rospy.spin()

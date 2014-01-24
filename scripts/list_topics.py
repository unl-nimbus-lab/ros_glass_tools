#!/usr/bin/env python
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

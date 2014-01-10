#!/usr/bin/env python
import roslib; roslib.load_manifest('list_topics')
import json, sys, re
import rospy, rostopic
from list_topics.srv import Topics




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
            return ' ,'.join(topics)            
        except:
            rospy.logerr("no core active")
            return None




if __name__ == '__main__':
    lt = ListTopics()
    rospy.spin()

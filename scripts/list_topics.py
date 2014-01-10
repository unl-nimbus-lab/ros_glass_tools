#!/usr/bin/env python
import roslib; roslib.load_manifest('list_topics')
import json, sys, re
import rospy, rostopic
from list_topics.srv import Topics

class ListTopics:
    def __init__(self):
        rospy.init_node("list_topics_server")
        self.serv = rospy.Service("list_topics", Topics, self.list)

    def list(self, request):

        topics = self.get_ros_topics()

        return topics

    def get_ros_topics(self):
        try:
            topics = rospy.get_published_topics()
            
            string = ""
            for topic in topics:
                string = string + topic[0] + " ,"

            return string[0:-2]

        except:
            print "no core active"
            return None




if __name__ == '__main__':
    lt = ListTopics()
    rospy.spin()

# Summary
ros_glass_tools provides the ros side actions for interacting with the Google
Glass.  Eaxmple applications include GlassTopicViewer, RobotVoiceControl, and GlassROSMonitor.

Running these scripts allows for information to be sent from
a computer running ros and [RosBridge](http://www.rosbridge.org/).

#Scripts

* list_topics.py - This provides a service to send the glass a list of currently publishing topics

* glass_monitor.py - Allows a user to specify specific topic message fields to inspect and report
a message if the data examined violates the defined constraint.  

* voice_command.py - Provides a service that will publish a blank message on a specified field when 
it receives a string message.  

For further information please see the ROS wiki [here](http://wiki.ros.org/ros_glass_tools).

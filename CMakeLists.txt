cmake_minimum_required(VERSION 2.8.3)
project(ros_glass_tools)

find_package(catkin REQUIRED
                COMPONENTS 
                rospy
                message_generation
                std_msgs)


## Generate services in the 'srv' folder
 add_service_files(
  FILES
  Command.srv
  Topics.srv
 )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs  
 )

catkin_package(
    CATKIN_DEPENDS message_runtime std_msgs
)


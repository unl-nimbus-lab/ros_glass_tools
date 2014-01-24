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
import rospy
from ros_glass_tools.srv import * 
from std_msgs.msg import Empty

class VoiceCommandControl:
    '''Node to translate a voice/text command into a ROS message command
      This node takes in a list of robots prefixes, a list of global commands and topics, and a list of robot
      commands and topics. 
      
        When a global command is received in a request it publishes a blank message on that topic.

        When a robot command is received which means robot #, or all. It will publish a command on that topic
        with the robot prefex added to the namespace. 
        and example launch file to set this up is shown below.

          <node name="voice_commands" type="voice_command.py" pkg="ros_glass_tools">
            <rosparam param="global_commands">
              [ [stop, stop],[go, go] ]
           
            </rosparam>
            <rosparam param="robots">
              [a, b]
            </rosparam>
            <rosparam param="robot_commands">
              [
                [forward, forward],
                [backward, backward]
              ]
            </rosparam>
           </node>
        '''



    def __init__(self):
      #init nodes and services
        rospy.init_node("voice_command_trans")
        self.serv = rospy.Service("glass_voice_command", Command, self.command)
        
        #read lists
        input_global_commands =  rospy.get_param('~global_commands', None)
        input_robots =  rospy.get_param('~robots', None)
        input_robot_commands=  rospy.get_param('~robot_commands', None)

        #create commands
        self.global_commands =  create_command_dict(input_global_commands)
        self.robot_commands =  create_robot_commands(input_robot_commands, input_robots)

    
        

    def command(self, request):
        '''
          Handle a service request at the moment handles requests of the following syntax for one word commands
          execute `command` -> Publish to the global command topic if defined
          all `command` -> Publsih the command on all robots topics if defined 
          robot # `command` -> Publsih the command on the # robot topics if robot exists and command defined
          Returns the warning message or success string
        '''
        robot_number = 'All' 
        command_arr = request.command.split(' ')
        result = 'Fail'
        #are they commanding a single robot
        if command_arr[0] == 'robot':
            try:
                #see if it parsed the speech as an int
                robot_number = int(command_arr[1])
            except:
                #otherwise use this to get the correct value
                number = command_arr[1]
                #to and too and four cause problems and must be replaced
                if number == 'to' or number == 'too':
                  number = 'two'
                if number == 'for':
                  number = 'four'
                robot_number = text2int(number)
            cmd = command_arr[2]
            #send the commands
            result = send_robot_command(robot_number, cmd, self.robot_commands)

        #are they command all of the robots
        elif command_arr[0] == 'all':
            cmd = command_arr[1]
            result = send_robot_command(robot_number, cmd, self.robot_commands)

        #or is it a global command
        elif command_arr[0] == 'execute':
            cmd = command_arr[1]
            result = send_command(cmd, self.global_commands)

        #or are they talking gibberish
        else:
            result = "INVALID COMMAND MUST BEGIN WITH All, Robot #, or execute"

        rospy.logerr(result)
        return CommandResponse(result)

def send_command(cmd, cmd_dict):
    '''publish an Empty message on the cmd if defined in the dictonary of command topics or 
      just return that we don't have a defined command
    '''
    if cmd not in cmd_dict:
        return "Command %s Not Defined"%cmd
    else:
        msg = Empty()
        cmd_dict[cmd].publish(msg)
        return "Sent Command: " + cmd  

def send_robot_command(robot_number, cmd, robot_list):
    '''publish the command on the specific robot
        If all is selected it will publish it on every robot on the list
        otherwise it is based on which robot number is defined
    '''
    cmd = cmd.lower()
    if robot_number == 'All':
        results = []
         #send the command on all of them
        for rob in robot_list:
            res = send_command(cmd, rob)
            results.append(res)
        return  '\n'.join(results)
            
    else:
        #only send it on one
        if robot_number > len(robot_list):
            return "Robot Number invalid"
        else:
            return send_command(cmd, robot_list[robot_number-1])


    

def create_command_dict(command_arr, prefix = ''):
    '''create a dictonary that maps command word to a publisher that will get 
        an empty message when the command is received 
    '''
    command_dict = {}
    for pair in command_arr:
        topic_name = prefix + pair[1]
        publisher = rospy.Publisher(topic_name, Empty)
        command_dict[pair[0]] = publisher   
    return command_dict

def create_robot_commands(commands, robots):
    '''with the list of commands and the list of robots make the robot array so
      we can publish to specific robots
     '''
    robot_list = []
    for prefix in robots:
       #create the dictonary with the robot prefix appended to the namespace
        robot_list.append(create_command_dict(commands,  prefix + '/'))
    return robot_list

        
def text2int(textnum):
        '''simple text number represnetation to integer value'''
        nums = [
        "zero", "one", "two", "three", "four", "five", "six", "seven", "eight",
        "nine", "ten", "eleven", "twelve", "thirteen", "fourteen", "fifteen",
        "sixteen", "seventeen", "eighteen", "nineteen"
        ]
        val = nums.index(textnum.lower())
        return val




if __name__ == '__main__':
    vcc = VoiceCommandControl()
    rospy.spin()

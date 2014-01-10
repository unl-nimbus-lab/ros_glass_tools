#!/usr/bin/env python
import rospy
from ros_glass_tools.srv import * 
from std_msgs.msg import Empty

class VoiceCommandControl:
    def __init__(self):
        rospy.init_node("voice_command_trans")
        self.serv = rospy.Service("glass_voice_command", Command, self.command)
        
        input_global_commands =  rospy.get_param('~global_commands', None)
        input_robots =  rospy.get_param('~robots', None)
        input_robot_commands=  rospy.get_param('~robot_commands', None)

        self.global_commands =  create_command_dict(input_global_commands)
        self.robot_commands =  create_robot_commands(input_robot_commands, input_robots)

    
        

    def command(self, request):
        #for now I'm just going to take the first word after the thing 
        #but I think it would be cool to process the whole list as commands
        robot_number = 'All' 
        command_arr = request.command.split(' ')
        result = 'Fail'
        if command_arr[0] == 'robot':
            try:
                robot_number = int(command_arr[1])
            except:
                robot_number = text2int(command_arr[1])
            cmd = command_arr[2]
            result = send_robot_command(robot_number, cmd, self.robot_commands)
        elif command_arr[0] == 'all':
            cmd = command_arr[1]
            result = send_robot_command(robot_number, cmd, self.robot_commands)
        elif command_arr[0] == 'execute':
            cmd = command_arr[1]
            result = send_command(cmd, self.global_commands)
        else:
            result = "INVALID COMMAND MUST BEGIN WITH All, Robot #, or execute"

        rospy.logerr(result)
        return CommandResponse(result)

def send_command(cmd, cmd_dict):
    if cmd not in cmd_dict:
        return "Command %s Not Defined"%cmd
    else:
        msg = Empty()
        cmd_dict[cmd].publish(msg)
        return "Sent Command: " + cmd  

def send_robot_command(robot_number, cmd, robot_list):
    if robot_number == 'All':
        results = []
        for rob in robot_list:
            res = send_command(cmd, rob)
            results.append(res)
        return  '\n'.join(results)
            
    else:
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
    robot_list = []
    for prefix in robots:
        robot_list.append(create_command_dict(commands,  prefix + '/'))
    return robot_list

        
def text2int(textnum, numwords={}):
    if not numwords:
      units = [
        "zero", "one", "to", "three", "four", "five", "six", "seven", "eight",
        "nine", "ten", "eleven", "twelve", "thirteen", "fourteen", "fifteen",
        "sixteen", "seventeen", "eighteen", "nineteen",
      ]

      tens = ["", "", "twenty", "thirty", "forty", "fifty", "sixty", "seventy", "eighty", "ninety"]

      scales = ["hundred", "thousand", "million", "billion", "trillion"]

      numwords["and"] = (1, 0)
      for idx, word in enumerate(units):    numwords[word] = (1, idx)
      for idx, word in enumerate(tens):     numwords[word] = (1, idx * 10)
      for idx, word in enumerate(scales):   numwords[word] = (10 ** (idx * 3 or 2), 0)

    current = result = 0
    for word in textnum.split():
        if word not in numwords:
          raise Exception("Illegal word: " + word)

        scale, increment = numwords[word]
        current = current * scale + increment
        if scale > 100:
            result += current
            current = 0

    return result + current




if __name__ == '__main__':
    vcc = VoiceCommandControl()
    rospy.spin()

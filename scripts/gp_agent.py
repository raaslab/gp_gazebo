#!/usr/bin/python

import numpy as np
import rospy
import time
import random
import math
import std_msgs.msg
import upDate_transition
import matplotlib.pyplot as plt
import actionlib
import matplotlib.pyplot as plt
import gp_gazebo.msg
import mavros
#import update_transition_class
from mavros.utils import *
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, StreamRate, StreamRateRequest, CommandBool, CommandTOL
from geometry_msgs.msg import *
from global_var import initialTrainingEpisodes, GRID
#currentState = 0
action_value = 0
old_state = (0,0)
next_state = (0,0)
#plannerObj = None
record = []
def agent_client():

    global action_value
    global old_state
    global next_state
    global record
    transition = upDate_transition.update_transition_class()
    #set the publisher for sending the goals
    action_client = actionlib.SimpleActionClient('env1',gp_gazebo.msg.agentAction)
    print "action client init"
    #r = rospy.Rate(20)
    action_client.wait_for_server()
	# Some Random Number
    for j in range (0,initialTrainingEpisodes):
        action_value = random.randint(0,3)
        goal = gp_gazebo.msg.agentGoal(action=action_value)
        action_client.send_goal(goal,done_cb= done)
        #action_client.send_goal(goal)
        print "GOAL SENT --> " + str(goal) 
        action_client.wait_for_result()

        if j == 5:
            action_client = actionlib.SimpleActionClient('env2',gp_gazebo.msg.agentAction)
            print "=============="
            print "Changing environment to 2"
            print "=============="
            action_client.wait_for_server()
        elif j == 10:
            action_client = actionlib.SimpleActionClient('env3',gp_gazebo.msg.agentAction)
            print "=============="
            print "Changing environment to 3"
            print "=============="
            action_client.wait_for_server()

    T = transition.upDate_transition(record)
    plt.show()

def done(integer,result):
    global action_value
    global old_state
    global next_state
    global record
    action_value_append = ()

    if integer == 3:

        if result.terminal == False:
            if result.reward != 0:
                next_state = (result.state[0],result.state[1])
                if action_value == 0:
                    action_value_append = (0,1)
                elif action_value == 1:
                    action_value_append = (-1,0)
                elif action_value == 2:
                    action_value_append = (0,-1)
                elif action_value == 3:
                    action_value_append = (1,0)
                record.append( [old_state, action_value_append, next_state] )
                old_state = next_state


if __name__ == '__main__':
    try:
        rospy.init_node('agent', anonymous=True)
        agent_client()
        #rospy.spin()
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
        pass

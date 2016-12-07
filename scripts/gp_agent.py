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
import planner
#import update_transition_class
from mavros.utils import *
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, StreamRate, StreamRateRequest, CommandBool, CommandTOL
from geometry_msgs.msg import *
from global_var import initialTrainingEpisodes, GRID
import global_var
from collections import deque
#currentState = 0
action_value = 0
old_state = (-GRID,GRID)
next_state = (0,0)
#plannerObj = None
record = []

envList = ['env3','env2','env1']
states1 = [ (i , j) for i in xrange(-GRID,GRID+1,1) for j in xrange(-GRID,GRID+1,1)]
states2 = [ (i , j) for i in xrange(-GRID,GRID+1,2) for j in xrange(-GRID,GRID+1,2)]
states3 = [ (i , j) for i in xrange(-GRID,GRID+1,4) for j in xrange(-GRID,GRID+1,4)]
recordCounter = 0
devQueue = deque(10*[0],10)

global_var.sigmaDict = {}
global_var.delta_t = 4
 
currentEnv = envList[0]
transition = upDate_transition.update_transition_class()
updateObj = planner.gprmax()

def currentStates(currentEnvironmet):
    global states1
    global states2
    global states3

    if currentEnvironmet == 'env1':
        return states1
    elif currentEnvironmet == 'env2':
        return states2
    elif currentEnvironmet == 'env3':
        return states3

def check(curr,currentEnvironment):
    global envList
    return curr in currentStates(envList[envList.index(currentEnvironment)-1])


def agent_client():

    global action_value
    global old_state
    global next_state
    global record
    global currentEnv
    global transition
    global updateObj
    global envList
    global recordCounter
    sigma_sum_thresh = 100
    sigmaThresh = 10
    actionList = [(0,1),(1,0),(0,-1),(-1,0)]
    #set the publisher for sending the goals
    action_client = actionlib.SimpleActionClient(currentEnv,gp_gazebo.msg.agentAction)
    print "action client init"
    #r = rospy.Rate(20)
    action_client.wait_for_server()
	# Some Random Number
    '''
    Initialize T
    '''
    for j in range (0,initialTrainingEpisodes):
        action_value = random.randint(0,3)
        goal = gp_gazebo.msg.agentGoal(action=action_value)
        action_client.send_goal(goal,done_cb= done)
        #action_client.send_goal(goal)
        print "GOAL SENT --> " + str(goal) 
        action_client.wait_for_result()

    T = transition.upDate_transition(record,currentStates(currentEnv))
    U = updateObj.value_iteration ( T ,currentStates(currentEnv))
    policy = updateObj.best_policy( U, T ,currentStates(currentEnv))
    old_state = (-GRID,-GRID)
    

    '''
    #GP-MFRL Algorithm
    '''

    for i in range(0,20):

        while True:
            print "-----------------------------------"
            print sum(devQueue)
            print "-----------------------------------"

            if old_state != (GRID, GRID):
                actionValue = policy [old_state]
            else : actionValue = actionList[random.randint(0,3)]
            #actionValue = policy [oldState]
            if envList.index(currentEnv) != 0:
                if check(old_state,currentEnv) and global_var.sigmaDict.get((old_state,actionValue),99) > sigmaThresh:
                    currentEnv = envList[envList.index(currentEnv)-1]
                    print "*************** PREVIOUS TRANSITION ***************"        
                    action_client = actionlib.SimpleActionClient(currentEnv,gp_gazebo.msg.agentAction)
                    print "action client init"
                    action_client.wait_for_server()
            if actionValue == (0,1):
                action_value = 0
            elif actionValue == (-1,0):
                 action_value = 1
            elif actionValue == (0,-1):
                 action_value = 2
            elif actionValue == (1,0):
                 action_value = 3

            goal = gp_gazebo.msg.agentGoal(action=action_value)
            action_client.send_goal(goal,done_cb= done)
            action_client.wait_for_result()
            
            currSigma = global_var.sigmaDict.get((next_state,actionValue),99)
            devQueue.appendleft(currSigma)
            recordCounter = recordCounter + 1

            if recordCounter == 10:
                T = transition.upDate_transition(record,currentStates(currentEnv))
                U = updateObj.value_iteration ( T ,currentStates(currentEnv))
                policy = updateObj.best_policy( U, T ,currentStates(currentEnv))        
                recordCounter = 0
                if sum(devQueue) < sigma_sum_thresh:
                    break
        
        print record
        print len(record)
        #FIND THE SIGMA VALUE and ADD
        if (envList.index(currentEnv) + 1) != len(envList):
            print 'MAKING TRANSITION'
            currentEnv = envList[envList.index(currentEnv) + 1]
            action_client = actionlib.SimpleActionClient(currentEnv,gp_gazebo.msg.agentAction)
            print "action client init"
            action_client.wait_for_server()

            T = transition.upDate_transition(record,currentStates(currentEnv))
            U = updateObj.value_iteration ( T ,currentStates(currentEnv))
            policy = updateObj.best_policy( U, T ,currentStates(currentEnv))
        #plot()
            #T = transition.upDate_transition(record,currentStates(currentEnv))
            #U = updateObj.value_iteration ( T ,currentStates(currentEnv))
        print "==========="
        print currentEnv
        print "==========="

    print T

    '''
    #END OF ALGORITHM
    '''
    

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
                velocity =  ((next_state[0] - old_state[0])/global_var.delta_t, (next_state[1] - old_state[1])/global_var.delta_t)
                record.append( [old_state, action_value_append, next_state] )
                old_state = next_state


if __name__ == '__main__':
    try:
        rospy.init_node('agent', anonymous=True)
        agent_client()
        #rospy.spin()
    except rospy.ROSInterruptException:
        print "program interrupted before completion"

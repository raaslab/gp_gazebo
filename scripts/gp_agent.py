#!/usr/bin/python

import numpy as np
import rospy
import time
import random
import math
import std_msgs.msg
import update_gp_new
import matplotlib.pyplot as plt
import actionlib
import matplotlib.pyplot as plt
import gp_gazebo.msg
import mavros
import planner_new
#import update_transition_class
from mavros.utils import *
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, StreamRate, StreamRateRequest, CommandBool, CommandTOL
from geometry_msgs.msg import *
from global_var import initialTrainingEpisodes, GRID, current_state_for_grid_world_reference
import global_var
from collections import deque
#currentState = 0
action_value = 0
old_state = (-GRID + 2, -GRID + 2)
next_state = (0,0)
#plannerObj = None
record = []

envList = ['gazebo']
states1 = [ (i , j) for i in xrange(-GRID,GRID+1,1) for j in xrange(-GRID,GRID+1,1)]
states2 = [ (i , j) for i in xrange(-GRID,GRID+1,2) for j in xrange(-GRID,GRID+1,2)]
states3 = [ (i , j) for i in xrange(-GRID,GRID+1,4) for j in xrange(-GRID,GRID+1,4)]
recordCounter = 0

global_var.sigmaDictX = {}
global_var.sigmaDictY = {}
global_var.delta_t = 1
 
currentEnv = envList[0]
transition = update_gp_new.update_transition_class()
updateObj = planner_new.gprmax()

#fig = plt.figure(2)
#plt.ion()
# for x in xrange(-GRID, GRID + 1, global_var.delta_t):
#     for y in xrange(-GRID, GRID + 1, global_var.delta_t):
#         plt.scatter(x,y, marker='o', s=100, color='red')

def currentStates(currentEnvironmet):
    global states1
    global states2
    global states3
    '''
    if currentEnvironmet == 'grid':
        return states2
    elif currentEnvironmet == 'gazebo':
        return states1
    
    elif currentEnvironmet == 'env3':
        return states3
    '''
    
    return states1

def check(curr, currentEnvironment):
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
    sigma_sum_threshX = [0.5, 1.0]
    sigmaThreshX = [0.1, 0.2]
    sigma_sum_threshY = [.5, 1.0]
    sigmaThreshY = [0.1, 0.2]
    actionList = [(0,1),(1,0),(0,-1),(-1,0)]

    devQueueX = deque([], 5)
    devQueueY = deque([], 5)
    #set the publisher for sending the goals
    action_client = actionlib.SimpleActionClient(currentEnv,gp_gazebo.msg.agentAction)
    print "action client init"
    #r = rospy.Rate(20)
    action_client.wait_for_server()
    # Some Random Number
    '''
    Initialize T
    '''
    # Inti
    current_state_for_grid_world_reference = old_state
    for j in range (0,initialTrainingEpisodes):
        action_value = random.randint(0,3)
        goal = gp_gazebo.msg.agentGoal(action=action_value)
        action_client.send_goal(goal,done_cb= done)
        #action_client.send_goal(goal)
        #print "GOAL SENT --> " + str(goal) 
        action_client.wait_for_result()

    T = transition.upDate_transition(record,currentStates(currentEnv)) 
    U = updateObj.value_iteration ( T ,currentStates(currentEnv),currentEnv)
    policy = updateObj.best_policy( U, T ,currentStates(currentEnv),currentEnv)
    old_state = (-GRID, -GRID)
    list_of_samples_gathered = []
    '''
    #GP-MFRL Algorithm
    '''
    tracking = 1
    f = open("reward_gp_single_mfrl.txt", "w")
    samples_in_second_simulator = 0
    while True:
    	#print tracking
    	tracking += 1
    	print '\n' +  str(old_state)
    	print devQueueX
        actionValue = actionList[random.randint(0,3)]
        #actionValue = policy [old_state]
        
        if envList.index(currentEnv) > 0 and check(old_state, currentEnv) and global_var.sigmaDictX.get((int(old_state[0]),actionValue[0]), 99) > sigmaThreshX[envList.index(currentEnv)-1] and global_var.sigmaDictY.get((int(old_state[1]),actionValue[1]),99) > sigmaThreshY[envList.index(currentEnv)-1]:
            currentEnv = envList[envList.index(currentEnv)-1]
            print "*************** PREVIOUS TRANSITION ***************"        
            devQueueX = deque([], 5)
            devQueueY = deque([], 5)
            # New action client init
            action_client = actionlib.SimpleActionClient(currentEnv,gp_gazebo.msg.agentAction)
            #print "action client init"
            action_client.wait_for_server()
        
        no_of_samples = 0
        if (sum(devQueueX) < sigma_sum_threshX[envList.index(currentEnv)] and sum(devQueueY) < sigma_sum_threshY[envList.index(currentEnv)]) and len(devQueueY) > 4 and envList.index(currentEnv) < len(envList) - 1:      
            currentEnv = envList[envList.index(currentEnv) + 1]
            print '++++++++NEXT Transition+++++++'
            action_client = actionlib.SimpleActionClient(currentEnv,gp_gazebo.msg.agentAction)
            #print "action client init"
            action_client.wait_for_server()
            devQueueX = deque([], 5)
            devQueueY = deque([], 5)

        reward_in_second_simulator = 0    
        if envList.index(currentEnv) < 2:
            samples_in_second_simulator += 1
            print samples_in_second_simulator
            print reward_in_second_simulator
            if samples_in_second_simulator % 25 == 0 : 
                U = updateObj.value_iteration (T, currentStates(currentEnv), currentEnv)
                policy = updateObj.best_policy( U, T ,currentStates(currentEnv),currentEnv)
                # while old_state != Goal_state
                # reward_in_second_simulator += reward_dynmaics(old_state, actionValue, currentEnv) 
                f.write( str(U[(-8,-8)])  )
                f.write('\n')
                print "WRITTEN TO A FILE"

        # no_of_samples += 1
        if actionValue == (0,1):
            action_value = 0
        elif actionValue == (-1,0):
            action_value = 1
        elif actionValue == (0,-1):
            action_value = 2
        elif actionValue == (1,0):
            action_value = 3

        goal = gp_gazebo.msg.agentGoal(action = action_value)
        action_client.send_goal(goal, done_cb= done)
        action_client.wait_for_result()
        currSigmaX = global_var.sigmaDictX.get((int(next_state[0]),actionValue[0]),999)
        currSigmaY = global_var.sigmaDictY.get((int(next_state[1]),actionValue[1]),999)
        devQueueX.appendleft(currSigmaX)
        devQueueY.appendleft(currSigmaY)

        recordCounter = recordCounter + 1

        if recordCounter  == 5:
            T = transition.upDate_transition(record,currentStates(currentEnv))
                # U = updateObj.value_iteration ( T ,currentStates(currentEnv),currentEnv)
                # policy = updateObj.best_policy( U, T ,currentStates(currentEnv),currentEnv)        
            recordCounter = 0
                #plt.quiver(next_state[0],next_state[1],actionValue[0],actionValue[1])
                #plt.scatter(next_state[0],next_state[1], marker='o', s=500, color='blue')
                #plt.scatter(old_state[0],old_state[1], marker='o', s=100, color='red')
                #plt.pause(0.001)
        # list_of_samples_gathered.append(no_of_samples)        
        # print  'Samples gathered in\t' + str(envList.index(currentEnv)) + '\t is \t' + str(no_of_samples)
        # FIND THE SIGMA VALUE and ADD

        if next_state == (GRID, GRID):
            tmp = U
            U = updateObj.value_iteration (T, currentStates(currentEnv), currentEnv)
            difference, maximum = 0, -999
            for key in U:
                difference = max(difference, abs(U[key] - tmp[key]))
                if U[key] > maximum : maximum = U[key]
                
            if difference < 0.05 * abs(maximum) and envList.index(currentEnv) == 1: break

            
            action_client = actionlib.SimpleActionClient(currentEnv,gp_gazebo.msg.agentAction)
            #print "action client init"
            action_client.wait_for_server()

            T = transition.upDate_transition(record,currentStates(currentEnv))
            policy = updateObj.best_policy( U, T ,currentStates(currentEnv),currentEnv)
        # print "==========="
        # print currentEnv
        # print "==========="

    # U = updateObj.value_iteration ( T ,currentStates(currentEnv),currentEnv)
    # policy = updateObj.best_policy( U, T ,currentStates(currentEnv),currentEnv)
    
    # print 'samples in grid world' + str(sum(list_of_samples_gathered[0: len(list_of_samples_gathered):2]))
    # print 'samples in gazebo' + str(sum(list_of_samples_gathered[1: len(list_of_samples_gathered):2]))
        #plot()
            #T = transition.upDate_transition(record,currentStates(currentEnv))
            #U = updateObj.value_iteration ( T ,currentStates(currentEnv))


    for x in range(-GRID, GRID + 1):
    	for y in range(-GRID, GRID + 1):
    		a, b = policy[x, y]
    		plt.quiver(x, y, a, b)		
    '''
    #END OF ALGORITHM
    '''
    plt.xlim(-GRID - 1, GRID + 1)
    plt.ylim(-GRID - 1, GRID + 1)
    plt.show()
    print policy
    f.close()

			



def done(integer,result):
    global action_value
    global old_state
    global next_state
    global record
    action_value_append = ()

    if integer == 3:
        if result.terminal == False:
            if result.reward != 0:
                next_state = (int(result.state[0]),int(result.state[1]))
                #next_state = (max(min(result.state[0],GRID),-GRID) , max(min(result.state[1],GRID),-GRID))
                if action_value == 0:
                    action_value_append = (0,1)
                elif action_value == 1:
                    action_value_append = (-1,0)
                elif action_value == 2:
                    action_value_append = (0,-1)
                elif action_value == 3:
                    action_value_append = (1,0)

                # print '\n'
                #print action_value_append
                # print next_state
                # print '\n'

                velocity =  ((next_state[0] - old_state[0])/global_var.delta_t, (next_state[1] - old_state[1])/global_var.delta_t)
                record.append( [old_state, action_value_append, velocity] )
                # print record
                old_state = next_state
                global_var.current_state_for_grid_world_reference = old_state


if __name__ == '__main__':
    try:
        rospy.init_node('agent', anonymous=True)
        agent_client()
        #rospy.spin()
    except rospy.ROSInterruptException:
        print "program interrupted before completion"
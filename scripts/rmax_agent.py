#!/usr/bin/python

import numpy as np
import rospy
import time
import random
import math
import std_msgs.msg
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
old_state = (-GRID, -GRID)
next_state = (0,0)
#plannerObj = None
record = []
gamma = 0.9
epsilon = 0.05
envList = ['gazebo']
states1 = [ (i , j) for i in xrange(-GRID,GRID+1,1) for j in xrange(-GRID,GRID+1,1)]
states2 = [ (i , j) for i in xrange(-GRID,GRID+1,2) for j in xrange(-GRID,GRID+1,2)]
states3 = [ (i , j) for i in xrange(-GRID,GRID+1,4) for j in xrange(-GRID,GRID+1,4)]
recordCounter = 0

global_var.sigmaDictX = {}
global_var.sigmaDictY = {}
global_var.delta_t = 1
 
currentEnv = envList[0]

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
    global epsilon
    global gamma
    global action_value
    global old_state
    global next_state
    global record
    global currentEnv
    
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
    def reward_dynmaics(state, action, env):
        actual_state = (max(min(state[0] + action[0], GRID), -GRID) , max(min(state[1] + action[1], GRID), -GRID))
        # wall1 = [(0, i) for i in range(-10, -2)]
        # wall2 = [(0, i) for i in range(3, 11)]
        # wall3 = [(i, 0) for i in range(-10, -2)]
        # wall4 = [(i, 0) for i in range(3, 11)]

        # wall1 = [(i, 0) for i in range(-7, -3)]
        # wall2 = [(i, 4) for i in range(-5, 8)]
        # wall3 = [(-7, i) for i in range(0, 7)]
        # wall4 = [(i, 6) for i in range(-10, -6)]

        if env == 'grid' :
            if actual_state == (GRID, GRID) : return 30
            #elif state in wall1 or state in wall2 : return -20 
            else : return -3

        else:
            #if state in wall1 or state in wall2 or state in wall3 or state in wall4 : return -20
            if actual_state == (GRID, GRID) : return 30 
            else : return -3
    current_state_for_grid_world_reference = old_state
    Q = dict([((s, a), 0) for s in currentStates(currentEnv) for a in actionList]) 
    N = dict([((s, a), 0) for s in currentStates(currentEnv) for a in actionList])
    N_next = dict([((s, a, sn), 0) for s in currentStates(currentEnv) for a in actionList for sn in currentStates(currentEnv)])

    
    #policy = updateObj.best_policy(U, T, currentStates(currentEnv),currentEnv)
    old_state = (-GRID, -GRID)
    list_of_samples_gathered = []
    '''
    #GP-MFRL Algorithm
    '''
    tracking = 1



    samples_in_second_simulator = 0
    reward_in_second_simulator = 0
    f = open("reward_one_sim_rmax.txt", "w")

    while True:

        
        # Choose action
        # actionValue = actionList[random.randint(0,3)]
        if random.random() < epsilon:
            actionValue = random.choice(actionList)
        else:
            q = [Q.get((old_state, a), 0.0) for a in actionList]
            maxQ = max(q)
            count = q.count(maxQ)
            if count > 1:
                best = [i for i in range(len(actionList)) if q[i] == maxQ]
                i = random.choice(best)
            else:
                i = q.index(maxQ)
            actionValue = actionList[i]


    	#if envList.index(currentEnv) == 1:
            # samples_in_second_simulator += 1
            # reward_in_second_simulator += reward_dynmaics(old_state, actionValue, currentEnv)
            # print samples_in_second_simulator
            # print reward_in_second_simulator
            # if samples_in_second_simulator % 25 == 0 : f.write( str(samples_in_second_simulator) + '\t' + str(reward_in_second_simulator)  )    	       
    	# print '\n' +  str(old_state)
    	# print devQueueX
        # actionValue = actionList[random.randint(0,3)]
        # actionValue = policy [old_state]
     	if envList.index(currentEnv) < 1:
            samples_in_second_simulator += 1
            print samples_in_second_simulator
            print reward_in_second_simulator
            if samples_in_second_simulator % 5 == 0 : 
                # U = updateObj.value_iteration (T, currentStates(currentEnv), currentEnv)
                # policy = updateObj.best_policy( U, T ,currentStates(currentEnv),currentEnv)
                # while old_state != Goal_state
                # reward_in_second_simulator += reward_dynmaics(old_state, actionValue, currentEnv) 
                f.write( str(max(Q[(-8,-8),aa] for aa in actionList)) )
                f.write('\n')
                print "WRITTEN TO FILE"
        

        if envList.index(currentEnv) > 0 and check(old_state, currentEnv) and N[(old_state, actionValue)] < 5 :
            currentEnv = envList[envList.index(currentEnv)-1]
            print "*************** PREVIOUS TRANSITION ***************"        
            action_client = actionlib.SimpleActionClient(currentEnv, gp_gazebo.msg.agentAction)
            action_client.wait_for_server()
            # New action client init
            
        
        
        if N[(old_state, actionValue)] > 4 and envList.index(currentEnv) < len(envList) - 1:      
            currentEnv = envList[envList.index(currentEnv) + 1]
            print '++++++++NEXT Transition+++++++'
            action_client = actionlib.SimpleActionClient(currentEnv, gp_gazebo.msg.agentAction)
            action_client.wait_for_server()

        

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
        
        N[(old_state, actionValue)] = N.get((old_state, actionValue)) + 1
        N_next[(s, a, next_state)] = N_next.get((old_state, actionValue, next_state)) + 1
        tmp = Q
        if N.get((old_state, actionValue)) == 5:
            for times in range(0, 20):
                for s in currentStates(currentEnv):
                    for a in actionList:
                        if N.get((s, a)) > 4 :
                            Q[(s, a)] = sum([ N_next.get((s, a, sn))/(N.get((s, a)) + 0.01) * ( reward_dynmaics(s, a, currentEnv) + gamma * max( [Q.get((sn, a)) for a in actionList] )) for sn in currentStates(currentEnv)])     
                               
            difference, maximum = 0, -999
            if next_state == (GRID, GRID):
                for key in Q:
                    difference = max(difference, abs(Q[key] - tmp[key]))
                    if Q[key] > maximum : maximum = Q[key]
                
                if difference < 0.05 * abs(maximum) and envList.index(currentEnv) == 1: break
            
            action_client = actionlib.SimpleActionClient(currentEnv,gp_gazebo.msg.agentAction)
            #print "action client init"
            action_client.wait_for_server()
        # print "==========="
        # print currentEnv
        # print "==========="
    f.close()
    # U = updateObj.value_iteration ( T ,currentStates(currentEnv),currentEnv)
    # policy = updateObj.best_policy( U, T ,currentStates(currentEnv),currentEnv)
    
    # print 'samples in grid world' + str(sum(list_of_samples_gathered[0: len(list_of_samples_gathered):2]))
    # print 'samples in gazebo' + str(sum(list_of_samples_gathered[1: len(list_of_samples_gathered):2]))
        #plot()
            #T = transition.upDate_transition(record,currentStates(currentEnv))
            #U = updateObj.value_iteration ( T ,currentStates(currentEnv))


    # for x in range(-GRID, GRID + 1):
    # 	for y in range(-GRID, GRID + 1):
    # 		a, b = policy[x, y]
    # 		plt.quiver(x, y, a, b)		
    # '''
    # #END OF ALGORITHM
    # '''
    # plt.xlim(-GRID - 1, GRID + 1)
    # plt.ylim(-GRID - 1, GRID + 1)
    # plt.show()
    # print policy

			

    print Q

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
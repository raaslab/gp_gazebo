#!/usr/bin/python -W ignore
import random
import numpy as np
import os
#import update
import matplotlib.pyplot as plt
import environment
import env2
import env3
import planner
import update_gp
import itertools
from global_var import GRID,trainingEpisodes
import global_var
from collections import deque

import time
start_time = time.time()

updateObj = planner.gprmax()
transition = update_gp.update_transition_class()

initialTrainingEpisodes = 5
record = []
actionList = [(0,1),(1,0),(0,-1),(-1,0)]
oldState = (-GRID,-GRID)
recordCounter = 0
devQueueX = deque([], 5)
devQueueY = deque([], 5)


envList = [env3, env2, environment]

states1 = [ (i , j) for i in xrange(-GRID,GRID+1,1) for j in xrange(-GRID,GRID+1,1)]
states2 = [ (i , j) for i in xrange(-GRID,GRID+1,2) for j in xrange(-GRID,GRID+1,2)]
states3 = [ (i , j) for i in xrange(-GRID,GRID+1,4) for j in xrange(-GRID,GRID+1,4)]

global_var.sigmaDictX = {}
global_var.sigmaDictY = {}

sigma_sum_threshX = 0.3
sigmaThreshX = 1.2
sigma_sum_threshY = 1.2
sigmaThreshY = 0.3
'''
Initializing current and previous environment
'''
currentEnv = envList[0]
global_i = 0

def currentStates(currentEnvironmet):
	if currentEnvironmet == environment:
		return states1
	elif currentEnvironmet == env2:
		return states2
	elif currentEnvironmet == env3:
		return states3

def check(curr,currentEnvironment):
	return curr in currentStates(envList[envList.index(currentEnvironment)-1])


def plot(policy):
	g = plt.figure(2)
	for x in xrange(-GRID, GRID + 1, global_var.delta_t):
		for y in xrange(-GRID, GRID + 1, global_var.delta_t):
			plt.scatter(x,y, marker='o', s=30, color='yellow')
			a,b = policy[x,y]
			plt.quiver(x,y,a,b)
			

	# plt.scatter(-GRID,-GRID, marker='o', s=200, color='blue')	
	# plt.scatter(GRID,GRID, marker='o', s=200, color='green')	


	# plt.scatter(GRID,GRID-1, marker='o', s=100, color='red')	
	# plt.scatter(-GRID+1,GRID, marker='o', s=100, color='red')	
	#figName = str(counter) + "_" + ".png"
	#directory = os.path.abspath("data")
	#if not os.path.exists(directory):
	#	os.makedirs(directory)
	#plt.savefig(os.path.join(directory,figName))
	plt.xlim(-GRID - 2, GRID + 2)
	plt.ylim(-GRID - 2, GRID + 2)
	g.show()

'''
Initialize transition dynamics
'''
for i in range(0,initialTrainingEpisodes):
	actionValue = actionList[random.randint(0,3)]
	currentState = currentEnv.environment( oldState , actionValue )
	velocity = ((currentState[0] - oldState[0])/global_var.delta_t, (currentState[1] - oldState[1])/global_var.delta_t)
	record.append( [oldState, actionValue, velocity] )
	oldState = currentState

#print currentStates(currentEnv)
#print record
#print '\n'


'''
f = plt.figure(1)
for elements in record:
			plt.scatter(elements[0][0], elements[0][1], marker='o', s=30, color='yellow')
			plt.quiver(elements[0][0], elements[0][1], elements[1][0], elements[1][1])

plt.xlim(-GRID - 2, GRID + 2)
plt.ylim(-GRID - 2, GRID + 2)
f.show()
'''


T = transition.upDate_transition(record, currentStates(currentEnv))
#print record
#print '\n'
U = updateObj.value_iteration ( T , currentStates(currentEnv))
#print U
#print '\n'
policy = updateObj.best_policy( U, T ,currentStates(currentEnv))
oldState = (-GRID,-GRID)

# print policy
# print '\n'
# print global_var.sigmaDictX
# print '\n'
# print global_var.sigmaDictY

#print global_var.sigmaDictX
#print global_var.sigmaDictY
count = 0
environmentkilist = []
for i in range(0, 35):
		# print '\n'
		# print i
		# print oldState
		# print check(oldState, currentEnv)
		# print global_var.sigmaDictX.get((oldState[0],actionValue[0]),99)
		# print global_var.sigmaDictY.get((oldState[1],actionValue[1]),99)
		# print '\n'
		
		actionValue = actionList[random.randint(0,3)]

		# if oldState != (GRID, GRID):
		# 	actionValue = policy [ oldState ]
		# else : actionValue = actionList[random.randint(0,3)]
		# actionValue = policy [oldState]

		environmentkilist.append(envList.index(currentEnv))
		if envList.index(currentEnv) > 0 and check(oldState, currentEnv) and global_var.sigmaDictX.get((oldState[0],actionValue[0]),99) > sigmaThreshX and global_var.sigmaDictY.get((oldState[1],actionValue[1]),99) > sigmaThreshY:
				currentEnv = envList[envList.index(currentEnv)-1]
				print "@@@Previous Transtion@@@"

		environmentkilist.append(envList.index(currentEnv))
		
		recordCounter = 0
		while (sum(devQueueX) > sigma_sum_threshX or sum(devQueueY) > sigma_sum_threshY) or len(devQueueY) < 5:     
			actionValue = actionList[random.randint(0,3)]
			# actionValue = policy [ oldState ]
			currentState = currentEnv.environment( oldState , actionValue )
			velocity = ((currentState[0] - oldState[0])/global_var.delta_t, (currentState[1] - oldState[1])/global_var.delta_t)
			record.append( [oldState, actionValue, velocity] )
			# print global_var.sigmaDictX
			# print global_var.sigmaDictY
			currSigmaX = global_var.sigmaDictX.get((currentState[0],actionValue[0]),999)
			currSigmaY = global_var.sigmaDictY.get((currentState[1],actionValue[1]),999)
			
			devQueueX.appendleft(currSigmaX)
			devQueueY.appendleft(currSigmaY)
			
			oldState = currentState
			recordCounter = recordCounter + 1
			if recordCounter == 7:
				recordCounter = 0
				T = transition.upDate_transition(record, currentStates(currentEnv))
				#U = updateObj.value_iteration (T , currentStates(currentEnv))
				#policy = updateObj.best_policy( U, T ,currentStates(currentEnv))
				recordCounter = 0
		# 	if sum(devQueue) < sigma_sum_thresh:
		# 		break
		print 'MAKING UP TRANSITION'
		devQueueX = deque([], 5)
		devQueueY = deque([], 5)
		if envList.index(currentEnv) < 2 : currentEnv = envList[ envList.index(currentEnv) + 1]
		T = transition.upDate_transition(record, currentStates(currentEnv))
U = updateObj.value_iteration ( T , currentStates(currentEnv))
policy = updateObj.best_policy( U, T ,currentStates(currentEnv))	
	#plot()
		#T = transition.upDate_transition(record,currentStates(currentEnv))
		#U = updateObj.value_iteration ( T ,currentStates(currentEnv))
# print len(policy)
plot(policy)

plt.show()
print environmentkilist
print("--- %s seconds ---" % (time.time() - start_time))

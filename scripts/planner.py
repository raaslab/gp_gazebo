#!/usr/bin/python -W ignore
import math 
import random
import numpy as np
from global_var import GRID

class gprmax:

    def __init__(self, actions=[(1,0), (-1,0), (0,1), (0,-1)], gamma=0.9, epsilon = 0.9, alpha = 0.2):
        self.q = {}

        self.epsilon = epsilon
        self.alpha = alpha
        self.gamma = gamma
        self.actions = actions
        #self.states= [(x,y) for x in range(-GRID,GRID+1) for y in range(-GRID,GRID+1)]
    
    def value_iteration( self, T ,states):
            
        U1 = dict([(s, 0) for s in states ])
        li = []
        li_final = []
        # print U1
        """Reward function modeled as Gaussian Distribution with mean and covariance matrix defined as below""" 
        # T, gamma = mdp.T , mdp.gamma
        while True:
            #U = U1.copy()
            delta = 0.01
            for s in states:
                temp = U1 [s]
                #Implementation of Update Step
                li_final[:] = []
                for a in self.actions:
                    li[:] = []
                    pList = T [( s , a )]
                    #p,s1 = T [( s , a )]
                    for INDEX in range(0,len(pList)):
                        li.append(self.gamma * U1[pList[INDEX][0]] * pList[INDEX][1] + pList[INDEX][1] * self.reward_dynmaics(pList[INDEX][0]))    
                
                    li_final.append(sum(li))
                    #li.append(self.gamma * U1[s1] * p + self.reward_dynmaics(s1))    
                U1[s] = round(max(li_final),3)
                delta = max(delta, abs(U1[s] - temp))
            #print U1
            #print "\n" 
            if delta < .1: #self.epsilon * (1 - self.gamma) / self.gamma:
                 return U1


    def best_policy(self, U, T,states):
        pi = {}
        for s in states:
            li=[]
            li_final = []
            for a in self.actions:
                li[:] = []
                pList = T [( s , a )]
                for INDEX in range(0,len(pList)):
                    li.append(self.gamma * U[pList[INDEX][0]] * pList[INDEX][1] + pList[INDEX][1]* self.reward_dynmaics(pList[INDEX][0]) )
                li_final.append(sum(li))
                #p,s1 = T [( s , a )]
                #li.append(self.gamma * U[s1] * p + self.reward_dynmaics(s1) )
            maxU = max(li_final)
            count = li.count(maxU)
            if count > 1:
                best = [i for i in range(len(self.actions)) if li_final[i] == maxU]
                index = random.choice(best)
            else:
                index = li_final.index(maxU)
            if   index == 0 : pi[s] =  (1, 0)
            elif index == 1 : pi[s] =  (-1, 0)
            elif index == 2 : pi[s] =  (0, 1)
            else            : pi[s] =  (0, -1)
        return pi
        

                 # Still has to define
    def reward_dynmaics(self , state):
        if state == (GRID,GRID) : return 50
        #elif state == (2, -1) : return -30  
        #elif state == (2, 1) : return -30 
        #elif state == (-1, 2) : return -30 
        #elif state == (0, -1) : return -30 
        #elif state == (1, 0) : return -30 
        else : return -1

    """Given an MDP and a utility function U, determine the best policy,
    as a mapping from state to action. """

    def expected_utility(self , a , s , U , mdp):
        """The expected utility of doing a in state s, according to the MDP and U."""
        return sum([p * U[s1] for (p, s1) in mdp.T(s, a)])


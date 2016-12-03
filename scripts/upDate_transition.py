#!/usr/bin/python

import numpy as np
import math
import random
print(__doc__)

from matplotlib import pyplot as plt

from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, ConstantKernel as C
from global_var import GRID,numOfActions
import global_var
from scipy.stats import multivariate_normal
from scipy.integrate import dblquad

np.random.seed(1)

numOfStates = (GRID*2 + 1) * (GRID*2 + 1)

class update_transition_class:

	def upDate_transition(self,record,states):

		T = {}
		probList = []
		p = []

		actions = [(1, 0), (-1, 0), (0, 1), (0, -1) ]
		#states = [ (i , j) for i in range(-GRID,GRID+1) for j in range(-GRID,GRID+1)]

		training, target = [], []
		for elements in record:
			training.append( [elements[0][0] , elements[0][1] , elements[1][0], elements[1][1]] )
			target.append( [elements[2][0], elements[2][1]] )  

		D , t = np.array( training ), np.array( target )
		kernel = C(1.0, (1e-3, 1e3)) * RBF(10, (1e-2, 1e2))
		gp = GaussianProcessRegressor(kernel=kernel, n_restarts_optimizer=9,alpha=1e-2)
		# Fit to data using Maximum Likelihood Estimation of the parameters
		gp.fit(D, t)
		# visualize policy
		muListX = []
		muListY = []
		muListX2 = []
		muListY2 = []
		sigmaListX = []
		sigmaListY = []
		prevList = []
		nextList = []
		array = np.arange(-GRID-4,GRID+4,0.1)
		state = np.array(states)
		varSum1 = 0
		varSum2 = 0
		varSumoverall = 0
		#print state
		'''
		fig = plt.figure()

		plot1 = fig.add_subplot(211)
		plot2 = fig.add_subplot(212)
		'''
		for i in array:
			mu, sigma_square = gp.predict( np.array( [i, 0, 1, 0,]),return_std=True, return_cov=False)
			sigma = np.sqrt(sigma_square)
			muListX.append(mu[0][0])
			sigmaListX.append(sigma[0])

			if i >= -2 and i <= 2:
				#plot1.scatter(i,mu[0][0],marker='o', s=100, color='blue')
				varSum1 = varSum1 + sigma[0]
			'''
			print "============"
			print mu[0][0]
			print sigma
			print "============"
			'''
			mu2, sigma2_square = gp.predict( np.array( [0, i, 0, 1,]),return_std=True, return_cov=False)
			sigma2 = np.sqrt(sigma2_square)
			muListY.append(mu2[0][1])
			sigmaListY.append(sigma2[0])

			if i >= -GRID and i <= GRID:
				#plot2.scatter(i,mu2[0][1],marker='o', s=100, color='blue')
				varSum2 = varSum2 + sigma2[0]
			prevList.append(i)

		#f.write(str(varSum1) + "\n")
		#f.close()

		#f2.write(str(varSum2) + "\n")
		#f2.close()
		'''
		plot1.set_ylabel('$x_{t+1}$')
		plot1.set_xlabel('$x_t$')

		plot2.set_ylabel('$y_{t+1}$')
		plot2.set_xlabel('$y_t$')

		plot1.plot(prevList,muListX,'r')
		plot2.plot(prevList,muListY,'r')
		'''

		#print muListX

		#print sigmaListX

		mu = np.array(muListX)
		sigmaVal = np.array(sigmaListX)

		mu2 = np.array(muListY)
		sigmaVal2 = np.array(sigmaListY)
		'''
		plot1.fill_between(prevList,mu + sigmaVal,mu-sigmaVal, facecolor='green', interpolate=True)
		plot2.fill_between(prevList,mu2 + sigmaVal2,mu2-sigmaVal2, facecolor='green', interpolate=True)		
		plt.show(block=False)
		'''
		for i in states:
			for k in actions:
				mu, sigma = gp.predict( np.array( [i[0], i[1], k[0], k[1]] ), return_std=True, return_cov=False)
				p[:] = []
				temp = {((i[0],i[1]),(k[0],k[1])):sigma[0]}
				global_var.sigmaDict = dict(global_var.sigmaDict.items() + temp.items())				
				
				stateValue = np.array([ i[0] +  mu[0][0] * global_var.delta_t, i[1] + mu[0][1] * global_var.delta_t])
				
				for j in states:
					#prob = self.probability_of_states(j[0],j[1],mu[0],sigma*np.eye(2))
					prob = self.probability_of_states(j[0],j[1],stateValue,sigma*np.eye(2))
					p.append(round(prob,3))
			
				list_of_prob = zip(states,p)
				temporary = {(i , k): list_of_prob}
				
				T = dict(T.items() + temporary.items() )
				'''
				a = np.around(mu[0][0])
				b = np.around(mu[0][1])
				temporary = {(i , k): (1, (max(min(a,GRID),-GRID) , max(min(b,GRID),-GRID) ) ) }
				T = dict(T.items() + temporary.items() )
				'''
		#print T
		return T
	'''
	Integrate Gaussian over the rectangle.

	'''		
			
	def probability_of_states(self, x, y, mu, var):
		var = multivariate_normal (mean = mu, cov = var)
		z = 0.5 * global_var.delta_t
		area = dblquad(lambda X, Y: var.pdf( [X , Y]), y - z, y + z, lambda X: x - z, lambda X: x + z)
		return area[0]

	def coVariance(self,vector1, vector2 ):
		vector1 = np.matrix(vector1)
		vector2 = np.matrix(vector2)
		difference = vector1 - vector2
		lambdaa=np.matrix([[1,.7],[.5,1]])
		#print lambdaa
		if difference.all() == False : return math.exp(-0.5 * difference * np.linalg.inv(lambdaa) * np.transpose(difference)) + 1
		else : return math.exp(-0.5 * difference * np.linalg.inv(lambdaa) * np.transpose(difference)) 

	def calc_kerNal(self, record):
		K_X, K_Y = [] , []
		for data in record:
			for data_copy in record:
				K_X.append(self.coVariance( (data[0][0] , data[1]) , (data_copy[0][0] , data_copy[1])  ))
				K_Y.append(self.coVariance( (data[0][1] , data[1]) , (data_copy[0][1] , data_copy[1])  ))
		return np.resize(np.matrix([K_X]), (len(record) , len(record))) , np.resize(np.matrix([K_Y]), (len(record) , len(record)))

	def prediction(self,record, state, action ):
		lix, liy, target_x, target_y = [], [] , [], []
		for elements in record:
			lix.append( [elements[0][0] , elements[1]] )
			liy.append( [elements[0][1] , elements[1]] )
		# 	k_x.append(self.coVariance( (state[0] , action) , (elements[0][0] , elements[1])  ))
		# 	k_y.append(self.coVariance( (state[1] , action) , (elements[0][1] , elements[1])  ))
		# k_x, k_y           = np.matrix ([k_x]), np.matrix ([k_y])	
			target_x.append( [elements[2][0]] )
			target_y.append( [elements[2][1]] ) 
		X , Y , x , y = np.array( lix ), np.array( liy ), target_x, target_y
		kernel = C(1.0, (1e-3, 1e3)) * RBF(10, (1e-2, 1e2))
		gpx = GaussianProcessRegressor(kernel=kernel, n_restarts_optimizer=9)
		# Fit to data using Maximum Likelihood Estimation of the parameters
		gpx.fit(X, x)

		gpy = GaussianProcessRegressor(kernel=kernel, n_restarts_optimizer=9)
		# Fit to data using Maximum Likelihood Estimation of the parameters
		gpy.fit(Y, y)
		mu_x, sigma_x = gpx.predict(np.array([state[0] , action]), return_std=True)
		mu_y, sigma_y = gpy.predict(np.array([state[1] , action]), return_std=True)
		a = int(round(mu_x))
		b = int(round(mu_y))
		return (max(min(a,GRID),-GRID) , max(min(b,GRID),-GRID))

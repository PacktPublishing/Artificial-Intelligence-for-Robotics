# -*- coding: utf-8 -*-
"""
Created on Sun May 20 21:26:58 2018

@author: Francis
"""
import numpy as np
from math import *
import matplotlib.pyplot as mp


#from keras.optimizers import Adam, SGD,Adadelta,Adagrad
#from keras.utils import to_categorical
#from keras.models import Sequential
#from keras.layers.core import Activation
#from keras.layers.core import Flatten
#from keras.layers.core import Dense
#from keras import backend as K


# functions and classes
# action matrix - all possible combinations of actions of the three motors
ACTIONMAT = np.array([[0,0,-1],[0,0,0],[0,0,1],
       [0,-1,-1],[0,-1,0],[0,-1,1],
       [0,1,-1],[0,1,0],[0,1,1],
       [-1,0,-1],[-1,0,0],[-1,0,1],
       [-1,-1,-1],[-1,-1,0],[-1,-1,1],
       [-1,1,-1],[-1,1,0],[-1,1,1],
       [1,0,-1],[1,0,0],[1,0,1],
       [1,-1,-1],[1,-1,0],[1,-1,1],
       [1,1,-1],[1,1,0],[1,1,1]])

COLLISIONVOLUMES=[[0,0,95,180],[0,0,60,130]]

class RobotArm():
    def __init__(self):
        self.state = [0,0,0]
        
    def setState(self,st):
        self.state = st
        self.position = calHandPosition(st)
        
    def setGoal(self,newGoal):
        self.goal = newGoal
        
    def calcReward(self):
        dx = self.goal[0] - self.position[0]
        dy = self.goal[1] - self.position[1]
        dist2goal = sqrt(dx*dx + dy*dy)
        self.dist2goal = dist2goal
        # we want the reqard to be 100 if the goal is met
        # and porportional to the distance from goal otherwise
        # the arm is 340mm long, so that is as far away as we can get
        #
        reward = (340.0-dist2goal)/340.0 * 100.0
        # check for collisions with robot body or the floor
        colide = isWithin(self.position)
        if colide:
            reward=0
            print ("COLLISION WARNING")
        self.reward = reward
        return reward
        

    def step(self,act,learningRate):
        newState = self.state + (act * learningRate)
        # range check
        for ii in range(3):
            newState[ii]=max(newState[ii],0)
            newState[ii]=min(newState[ii],255.0)
        
        self.setState(newState)
        reward = self.calcReward()
        
        return self.state,reward
# for a given action, return the new state 

def isBetween(pnt,minpt,maxpt):
    betw = False
    if pnt > minpt and pt<maxpt:
        betw == True
    return betw
        
def isWithin(point):
    # is this point within the given area
    # used for collision detection
    collision=False
    for area in COLLISIONVOLUMES:
        x1,y1,x2,y2=area
        x,y = point
        if x < x2 and x > x1 and y<y2 and y>y1:
            # yes, point is in volume
            collision=True
    return collision

def isequals(Qline1,Qline2):
    st1 = Qline[0]
    st2 = Qline[1]
    return st1 == st2

def stateEqual(state1,state2):
    s1,s2,s3 = state1
    s4,s5,s6 = state2
    return s1==s4 and s2==s5 and s3==s6         

# just a utility to display the joint angle in degrees
def joint2deg(jointPos):
    return jointPos * (180.0 / 255.0)

def calHandPosition(stat):
    m1,m2,m3=stat
    # calculate hand position based on the position of the servo motors
    # m1, m2, m3 = motor command from 0 to 255
    # forward kinematics
    # we first convert each to an angle 
    d1 = 102.5  # length of first joint (sholder to elbow) in mm
    d2 = 97.26  # length of second joint arm (elbow to wrist) in mm
    d3 = 141    # length of thrird joint arm (wrist to hand)
    right = pi/2.0 # right angle, 90 degrees or pi/2 radians
    m1Theta = pi - m1*(pi/255.0)
    m2Theta = pi - m2*(pi/255.0)
    m3Theta = pi - m3*(pi/255.0)
    
    m2Theta = m1Theta-right+m2Theta
    m3Theta = m2Theta-right+m3Theta
    joint1 = np.array([d1*cos(m1Theta),d1*sin(m1Theta)])
    joint2 = np.array([d2*cos(m2Theta),d2*sin(m2Theta)])+joint1
    joint3 = np.array([d3*cos(m3Theta),d3*sin(m3Theta)])+joint2
    return joint3

def action_sample(mode,state,Qmatrix):
    if mode=="random":
        index = np.random.randint(0,ACTIONMAT.shape[0])
        action = ACTIONMAT[index]
        return index
    if mode=="Q":
         # Qmatrix contains a list of states [x,y], actions [1...27] and rewards
        # for example [34,14, 122],5,23  
        # where 34,14,122 is the state (motor posisions), 5 is the index to the ACTIONMAT action 5[0,-1,1]
        # we want to see if this state has an action, and if so, to take the max reward
        #
        # find the states
        myStatesQ =[]
        for datum in Qmatrix:
            if stateEqual(datum[0],state):
                myStatesQ.append(datum)
        if len(myStatesQ)==0:
            # no data found, do a random move
            # note recursion
            action=action_sample('random',state,Qmatrix)
        else:
            maxState=[[0,0,0],0,-9999.0]
            for thisStateQ in myStatesQ:
                # find the state with the highest value
                if thisStateQ[2]>maxState[2]:
                    maxStat = thisStateQ
            if maxState[2]==-9999.0:
                # no max state was found?
                action=action_sample('random',state,Qmatrix)
            else:
                action = maxState[1]
        index=action
    return index
    
def maxQ(Q, state):
    #retrieve the maximum Q value for this state
    maxQvalue=0
    for thisQ in Q:
        thisState=thisQ[0]
        action=thisQ[1]
        qvalue = thisQ[2]
        if stateEqual(state,thisState):
            maxQvalue = max(qvalue,maxQvalue)
    return maxQvalue
    
    
def setQ(Q,state,action,value):
    index = 0
    found=False
   # print ("setQ ",state,action,value),
    for datum in Q:
        try:
            if stateEqual(state,datum[0]) and action==datum[1]:
                Q[index]=[state,action,value]
                found=True
                break
        except:
            print ("except setQ",action, datum)
        index += 1
    if not found:        
        Q.append([state,action,value])
    
# begin main program

# starting state
# our arm has states from 0 to 255 which map to degrees from 0 to 180
# here is our beginning state
# initialize our "Q" matrix
# @ Q is a matrix of the number of states by the number of actions
# we will add states as we go in this version of this program
#Q=[[[127,127,127],1,0.0]]
Q=[]
state = [127,127,127]
oldState = state
# initial learning rate for the arm - we start with 10 units
learningRate = 10.0
robotArm = RobotArm()
robotArm.setState(state)
goal=[14,251]
robotArm.setGoal(goal)
knt = 0 # counter
reward=0.0  # no reward yet...
d2g=0.0
oldd2g = d2g
curve = []
curve2=[]
posx = []
posy=[]
oldReward=0.0
gamma = 0.6  # discount for rewards that take more time


# Q learning phase
stateReset = [127,127,127]
state = stateReset
robotArm.setState(state)
knt = 0
reward = 0
gamma = 0.99
G=0
# perform training on Q Learning
for epoch in range(1,100):
    done=False
    G,reward,knt = 0,0,0
    state = stateReset
    robotArm.setState(state)
    while not done:
        action = action_sample("Q",state,Q)
        motorAction = ACTIONMAT[action]
        state2,reward = robotArm.step(motorAction,learningRate)
        newQ=reward + gamma * maxQ(Q, state2)
        setQ(Q,state,action,newQ)
        G += reward
        knt +=1
        if knt > 1000 or reward > 90:
            done=True
        state = state2
        robotArm.setState(state)
    if epoch % 2 == 0:
        print("Epoch ",epoch,"TotalReward:",G," counter:",knt,"Q Len ",len(Q))
        
        


        
        
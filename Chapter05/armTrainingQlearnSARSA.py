# -*- coding: utf-8 -*-
"""
Created on Sun May 20 21:26:58 2018

@author: Francis
"""
import numpy as np
from math import *
import matplotlib.pyplot as mp
import random
import pickle

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

STATEINDEX=[]
MAXSTATE = 0

COLLISIONVOLUMES=[[0,0,95,180],[0,0,60,130]]

def getStateIndex(state):
    global MAXSTATE
    thisState = state.tolist()
    if thisState in STATEINDEX:
        return STATEINDEX.index(thisState)
    else:
        # not found in state index, we add it
        STATEINDEX.append(thisState)
        MAXSTATE +=1
        return len(STATEINDEX)-1

def roundup(x):
    return int(ceil(x / 10.0)) * 10            

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
            #print ("COLLISION WARNING")
        self.reward = reward
        return reward
        

    def step(self,act,learningRate):
        newState = self.state + (act * learningRate)
        #newState=np.array([roundup(newState[0]),roundup(newState[1]),roundup(newState[2])])
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
   
def predictReward(state,goal,action, learningRate):
    newState = state + (action * learningRate)
    position = calHandPosition(newState)
    dx = goal[0] - position[0]
    dy = goal[1] - position[1]
    dist2goal = sqrt(dx*dx + dy*dy)
    dist2goal = dist2goal
    # we want the reqard to be 100 if the goal is met
    # and porportional to the distance from goal otherwise
    # the arm is 340mm long, so that is as far away as we can get
    #
    reward = (340.0-dist2goal)/340.0 * 100.0
    # check for collisions with robot body or the floor
    colide = isWithin(position)
    if colide:
        reward=0
        #print ("COLLISION WARNING")
    reward = reward
    return reward
# just a utility to display the joint angle in degrees
def joint2deg(jointPos):
    return jointPos * (180.0 / 255.0)

def calHandPosition(state):

    m1,m2,m3=state
    # calculate hand position based on the position of the servo motors
    # m1, m2, m3 = motor command from 0 to 255
    # forward kinematics
    # we first convert each to an angle 
    d1 = 102.5  # length of first joint (sholder to elbow) in mm
    d2 = 97.26  # length of second joint arm (elbow to wrist) in mm
    d3 = 141    # length of thrird joint arm (wrist to hand)
    right = pi/2.0 # right angle, 90 degrees or pi/2 radians
    # convert angle from units to radians and reverse the direction
    m1Theta = pi - m1*(pi/255.0)
    m2Theta = pi - m2*(pi/255.0)
    m3Theta = pi - m3*(pi/255.0)
    
    # add the joint angles  m2 = m1 + m2, m3 = m2+m3
    m2Theta = m1Theta-right+m2Theta
    m3Theta = m2Theta-right+m3Theta
    # compute the cartesian coordinates of the arm in mm
    # translate the origin of the new joint to the end of the preceeding joint
    joint1 = np.array([d1*cos(m1Theta),d1*sin(m1Theta)])
    joint2 = np.array([d2*cos(m2Theta),d2*sin(m2Theta)])+joint1
    joint3 = np.array([d3*cos(m3Theta),d3*sin(m3Theta)])+joint2
    return joint3

def action_sample(mode,stat,Qmatrix, learningRate, goal):
    state = STATEINDEX[stat]
    if mode=="random":
        index = np.random.randint(0,ACTIONMAT.shape[0])
        print("RAND ACTION",index)
        #action = ACTIONMAT[index]
        return index
    if mode=="maxreturn":
        state=STATEINDEX[stat]
        maxRew=-9990
        doAction=1
        for adex in range(len(ACTIONMAT)):
            act = ACTIONMAT[adex]
            thisReward = predictReward(state,goal,act,learningRate)
            if thisReward > maxRew: 
                maxRew=thisReward
                doAction = adex
            if maxRew == -9990:
                doAction= action_sample("random",stat,Qmatrix,learningRate,goal)
        return doAction
    if mode=="Q":
        try:
            # note this returns the number of the action with the highest Q score!
            action = np.argmax(Q[statQ])
            return(action)
        except:
            # no action found
            action= action_sample("maxreturn",stat,Qmatrix,learningRate,goal)
            return action

def randomState():
    m1=roundup(random.randint(0,127))
    m2=roundup(random.randint(0,127))
    m3=roundup(random.randint(0,127))
    state = np.array([m1,m2,m3])
#    m1=random.randint(0,127)
#    m2=random.randint(0,127)
#    m3=random.randint(0,127)
#    state = np.array([m1,m2,m3])
    return state
    

# begin main program

# starting state
# our arm has states from 0 to 255 which map to degrees from 0 to 180
# here is our beginning state
# initialize our "Q" matrix
# @ Q is a matrix of the number of states by the number of actions
# we will add states as we go in this version of this program
#We preallocate 15,000 spaces to put states and actions in the q matrix
# if we need to make it bigger, we use np.resize to expand
Q=np.zeros((150000,27))
state = np.array([127,127,127])
# initial learning rate for the arm - we start with 3 units
learningRate = 3
robotArm = RobotArm()
robotArm.setState(state)
goal=[14,251]
robotArm.setGoal(goal)
knt = 0 # counter
reward=0.0  # no reward yet...


# Q learning phase
stateReset = np.array([127,127,127])
state = stateReset
robotArm.setState(state)
knt = 0
reward = 0
# discount function
gamma = 0.6
G=0
rewardData = []
# perform training on Q Learning
for epoch in range(1,5000):
    done=False
    G,reward,knt = 0,0,0
    state = randomState()
    #state = np.array([127,127,127])
    robotArm.setState(state)
    stat = getStateIndex(state)
    lastStat=stat
    lastAction=1 # i.e. do nothing[0,0,0]
    while not done:
        action = action_sample("Q",stat,Q, learningRate,robotArm.goal)
        motorAction = ACTIONMAT[action]
        state2,reward = robotArm.step(motorAction,learningRate)
        stat2 = getStateIndex(state2)
        # SARSA = State, Action, Reward, State, Action
        Q[lastStat,lastAction]=reward+gamma*Q[stat2,action]
        #Q[stat,action]=reward + gamma * np.max(Q[stat2])
        # new SARSA based Q function

        G += reward
        knt +=1
        if knt > 1000 or reward > 95:
            done=True
        stat = stat2
        lastStat = stat
        lastAction=action
        robotArm.setState(STATEINDEX[stat])
        # see if we need more states...
        if len(STATEINDEX)> len(Q)-10:
            wid,hit = Q.shape
            wid += 1000
            print( "Adding 1,000 more Q values", wid)
            Q.append(np.zeros((1000,hit)))
            
    if epoch % 2 == 0:
        print("Epoch ",epoch,"TotalReward:",int(G)," counter:",knt,"Q Len ",MAXSTATE)
        
 # now that we are done, we save the Q array so we can reuse it
       
output = open('q-arm-matrix.pkl', 'wb')

# Pickle dictionary using protocol 0.
pickle.dump(Q, output)
output.close()

        
        
import numpy as np
from math import sqrt

class Agent(object):

    def __init__(self, csvParameters, ksi=0.5, dhor = 10, timehor=5, goalRadiusSq=1, maxF = 10):
        """ 
            Takes an input line from the csv file,  
            and initializes the agent
        """
        self.id = int(csvParameters[0]) # the id of the agent
        self.gid = int(csvParameters[1]) # the group id of the agent
        self.pos = np.array([float(csvParameters[2]), float(csvParameters[3])]) # the position of the agent 
        self.vel = np.zeros(2) # the velocity of the agent
        self.goal = np.array([float(csvParameters[4]), float(csvParameters[5])]) # the goal of the agent
        self.prefspeed = float(csvParameters[6]) # the preferred speed of the agent
        self.gvel = self.goal-self.pos # the goal velocity of the agent
        self.gvel = self.gvel/(sqrt(self.gvel.dot(self.gvel )))*self.prefspeed       
        self.maxspeed = float(csvParameters[7]) # the maximum sped of the agent
        self.radius = float(csvParameters[8]) # the radius of the agent
        self.goalRadiusSq =goalRadiusSq # parameter to determine if agent is close to the goal
        self.atGoal = False # has the agent reached its goal?
        self.ksi = ksi # the relaxation time used to compute the goal force
        self.dhor = dhor # the sensing radius
        self.timehor = timehor # the time horizon for computing avoidance forces
        self.F = np.zeros(2) # the total force acting on the agent
        self.maxF = maxF # the maximum force that can be applied to the agent
        self.Favoid = np.zeros(2)
            
    def ttc(self,i):
        R = self.radius + i.radius
        W = self.pos - i.pos
        C = np.dot(W,W) - R*R
        if(C<0):
            return 0
        V = self.vel - i.vel
        A = np.dot(V,V)
        B = np.dot(W,V)
        if (B > 0):
            return np.inf
        D = B*B - A*C
        if (D <=0):
            return np.inf
        tau = C/(-B + sqrt(D))
        if (tau < 0):
            return 0
        return tau
    
    def computeForces(self, neighbors=[]):
        
        if not self.atGoal:
            favoid = np.zeros(2)
            for i in neighbors:
                if(self.id!=i.id):
                    d_ab = np.linalg.norm(self.pos - i.pos) - self.radius - i.radius 
                    if(d_ab < self.dhor):
                        tau = self.ttc(i)
                        eta_ab = (self.pos + self.vel*tau - i.pos - i.vel*tau)/np.linalg.norm(self.pos + self.vel*tau - i.pos - i.vel*tau)
                        if(tau == 0 or tau == np.inf):
                            continue
                        favoid = favoid + (max(self.timehor - tau,0)*eta_ab)/tau
            
            fgoal = (self.gvel - self.vel)/self.ksi 
            
            if(np.linalg.norm(fgoal + favoid) > self.maxF):
                self.F = ((fgoal + favoid)/np.linalg.norm(fgoal+favoid))*self.maxF 
                
            else:
                self.F = fgoal + favoid


    def update(self, dt):
        """ 
            Code to update the velocity and position of the agents.  
            as well as determine the new goal velocity 
        """
        if not self.atGoal:
            self.vel += self.F*dt     # update the velocity
            self.pos += self.vel*dt   #update the position
            
            # compute the goal velocity for the next time step. do not modify this
            self.gvel = self.goal - self.pos
            distGoalSq = self.gvel.dot(self.gvel)
            if distGoalSq < self.goalRadiusSq: 
                self.atGoal = True  # goal has been reached
            else: 
                self.gvel = self.gvel/sqrt(distGoalSq)*self.prefspeed  
            
            
  
#!/usr/bin/python

import numpy as np

class DiffDriveController():
    """
    Class used for controlling the robot linear and angular velocity
    """
    def __init__(self, max_speed, max_omega):
        # TODO for Student: Specify these parameters
        self.kp=0.6 #3 in sim
        self.ka=0.8 #6 in sim
        self.kb=0 #-2 in sim
        self.MAX_SPEED = max_speed
        self.MAX_OMEGA = max_omega
        
    def compute_vel(self, state, goal):
        """
        Function that computes the desired outputs given the state and goal
        Inputs:
        state - a numpy vector of size 3 by 1 with components (x,y,theta)
        goal - a numpy vector of size 2 by 1 specifying the location of the goal
        Outputs: a tuple with 3 elements
        v - a number specifying the forward speed (in m/s) of the robot (should 
            be no more than max_speed)
        omega - a number specifying the angular velocity (in rad/s) of the robot
            (should be no more than max_omega)
        done - a boolean value specifying if the robot has reached its goal (or
            is close enough
        """
        # YOUR CODE HERE

        state = state.flatten()
        goal = goal.flatten()

        dx = goal[0] - state[0]
        dy = goal[1] - state[1]
        theta = state[2]

        rho = np.sqrt(dx*dx + dy*dy)
        alpha = np.arctan2(dy,dx) - theta
        beta = -theta - alpha

        v = self.kp*rho
        omega = self.ka*alpha + self.kb*beta

        done = False

        if abs(rho) < 0.2:
            v = 0
            omega = 0
            done = True

        if omega > self.MAX_OMEGA:
            omega = self.MAX_OMEGA

        if omega < -self.MAX_OMEGA:
            omega = -self.MAX_OMEGA

        if abs(v) > self.MAX_SPEED:
            v = self.MAX_SPEED

        return v, omega, done

        #pass

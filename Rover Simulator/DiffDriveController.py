#!/usr/bin/python

import numpy as np

class DiffDriveController():
    """
    Class used for controlling the robot linear and angular velocity
    """
    def __init__(self, max_speed, max_omega):
        # TODO for Student: Specify these parameters
        self.kp=0.5
        self.ka=1.5
        self.kb=-0 # set to 0 if the goal orientation doesn't matter
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

        alpha = (np.arctan2(dy,dx) - theta + np.pi) % (2*np.pi) - np.pi
        beta = (-theta - alpha + np.pi) % (2*np.pi) - np.pi

        v = self.kp*rho
        omega = self.ka*alpha + self.kb*beta

        if alpha > np.pi / 2 or alpha < -np.pi / 2:
            v = -v

        done = False

        if abs(v) > self.MAX_SPEED:
            v = self.MAX_SPEED

        if omega > self.MAX_OMEGA:
            omega = self.MAX_OMEGA

        if omega < -self.MAX_OMEGA:
            omega = -self.MAX_OMEGA
        
        if abs(rho) < 0.1:
            v = 0
            omega = 0
            done = True

        return v, omega, done

        #pass

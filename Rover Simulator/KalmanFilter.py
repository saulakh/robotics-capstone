#!/usr/bin/python
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib import patches
#import pylab
import time
import math

class KalmanFilter:
    """
    Class to keep track of the estimate of the robots current state using the
    Kalman Filter
    """
    def __init__(self, markers):
        """
        Initialize all necessary components for Kalman Filter, using the
        markers (AprilTags) as the map
        Input: 
        markers - an N by 4 array loaded from the parameters, with each element
            consisting of (x,y,theta,id) where x,y gives the 2D position of a
            marker/AprilTag, theta gives its orientation, and id gives its
            unique id to identify which one you are seeing at any given
            moment
        """
        self.markers = markers
        self.last_time = None # Used to keep track of time between measurements 
        self.Q_t = 50*np.eye(2)
        self.R_t = 0.01*np.eye(3)
        # YOUR CODE HERE
        self.x_t = np.array([[0],[0],[0]]) # state
        self.P_t = 10000*np.eye(3) # covariance

    def prediction(self, v, imu_meas):
        """
        Performs the prediction step on the state x_t and covariance P_t
        Inputs:
        v - a number representing in m/s the commanded speed of the robot
        imu_meas - a 5 by 1 numpy array consistening of the values
            (acc_x,acc_y,acc_z,omega,time), with the fourth of the values giving
            the gyroscope measurement for angular velocity (which you should
            use as ground truth) and time giving the current timestamp. Ignore
            the first three values (they are for the linear acceleration which
            we don't use)
        Outputs: a tuple with two elements
        predicted_state - a 3 by 1 numpy array of the prediction of the state
        predicted_covariance - a 3 by 3 numpy array of the prediction of the
            covariance
        """
        # YOUR CODE HERE
        imu_omega = imu_meas[3,0]
        dt = imu_meas[4, 0] - self.last_time
        self.last_time = imu_meas[4,0]

        theta = self.x_t[2]

        dfdx = np.eye(3) + dt * np.array([[0, 0, -v*np.sin(theta)], [0, 0, v*np.cos(theta)], [0, 0, 0]], dtype = 'float')
        dfdn = dt * np.array([[np.cos(theta), 0], [np.sin(theta), 0], [0,1]], dtype = 'float')

        x_pred = self.x_t[0] + dt*v*np.cos(theta)
        y_pred = self.x_t[1] + dt*v*np.sin(theta)
        theta_pred = theta + dt*imu_omega

        if theta_pred >= np.pi:
            theta_pred -= 2*np.pi
        elif theta_pred <= -np.pi:
            theta_pred += 2*np.pi
        
        self.x_t_prediction = np.array([x_pred, y_pred, theta_pred], dtype='float')

        #self.x_t_prediction = self.x_t + dt*np.array([[v*np.cos(theta_r)],[v*np.cos(theta_r)],[imu_omega]], dtype = 'float')
        self.P_t_prediction = (dfdx.dot(self.P_t)).dot(np.transpose(dfdx)) + (dfdn.dot(self.Q_t)).dot(np.transpose(dfdn))

        return (self.x_t_prediction, self.P_t_prediction)

        #pass

    def update(self,z_t):
        """
        Performs the update step on the state x_t and covariance P_t
        Inputs:
        z_t - an array of length N with elements that are 4 by 1 numpy arrays.
            Each element has the same form as the markers, (x,y,theta,id), with
            x,y gives the 2D position of the measurement with respect to the
            robot, theta the orientation of the marker with respect to the
            robot, and the unique id of the marker, which you can find the
            corresponding marker from your map
        Outputs:
        predicted_state - a 3 by 1 numpy array of the updated state
        predicted_covariance - a 3 by 3 numpy array of the updated covariance
        """
        # YOUR CODE HERE
        dhdx = np.eye(3)
        K = (self.P_t_prediction.dot(np.transpose(dhdx))).dot(np.linalg.inv((dhdx.dot(self.P_t_prediction)).dot(np.transpose(dhdx)) + self.R_t))

        if z_t != None and z_t != []:
            z_t = np.array(z_t)

            for i in range(z_t.shape[0]):

                tag_id = int(z_t[i][3])
                print("tag id is:", tag_id)
                tag_robot = z_t[i][0:3] # tag pose in robot frame
                #print("tag robot frame is:", tag_robot)
                tag_world = self.markers[tag_id, 0:3] # tag pose in world frame

                xw = tag_world[0]
                yw = tag_world[1]
                theta_w = tag_world[2]

                xr = tag_robot[0]
                yr = tag_robot[1]
                theta_r = tag_robot[2]

            H_w = np.array([[np.cos(theta_w), -np.sin(theta_w), xw], [np.sin(theta_w), np.cos(theta_w), yw], [0, 0, 1]], dtype = 'float')
            H_r = np.array([[np.cos(theta_r), -np.sin(theta_r), xr], [np.sin(theta_r), np.cos(theta_r), yr], [0, 0, 1]], dtype = 'float')
            H_r_inv = np.linalg.inv(H_r)

            w_r = H_w.dot(H_r_inv)
            robot_pose = np.array([[w_r[0, 2]], [w_r[1, 2]], [np.arctan2(w_r[1, 0], w_r[0, 0])]])
            print("robot pose is", robot_pose)
            self.x_t = self.x_t_prediction + K.dot(robot_pose - self.x_t_prediction)
        else:
            self.x_t = self.x_t_prediction

        self.P_t = self.P_t_prediction - (K.dot(dhdx)).dot(self.P_t_prediction)

        return self.x_t, self.P_t

        #pass

    def step_filter(self, v, imu_meas, z_t):
        """
        Perform step in filter, called every iteration (on robot, at 60Hz)
        Inputs:
        v, imu_meas - descriptions in prediction. Will be None value if
            values are not available
        z_t - description in update. Will be None value if measurement is not
            available
        Outputs:
        x_t - current estimate of the state
        """
        # YOUR CODE HERE
        if np.all(imu_meas!= None) and imu_meas.shape == (5, 1):
            if self.last_time == None:
                self.last_time = imu_meas[4, 0]
            else:
                self.prediction(v, imu_meas)
                if z_t != None:
                    self.update(z_t)

        return self.x_t
        
        #pass

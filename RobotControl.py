#!/usr/bin/env python
"""
ROS based interface for the Course Robotics Specialization Capstone Autonomous Rover.
Updated June 15 2016.
"""
import rospy

import yaml
import numpy as np

import sys

from RosInterface import ROSInterface

# User files, uncomment as completed
from ShortestPath import dijkstras
from KalmanFilter import KalmanFilter
from DiffDriveController import DiffDriveController

class RobotControl(object):
    """
    Class used to interface with the rover. Gets sensor measurements through ROS subscribers,
    and transforms them into the 2D plane, and publishes velocity commands.
    """
    def __init__(self, world_map,occupancy_map, pos_init, pos_goal, max_speed, max_omega, x_spacing, y_spacing, t_cam_to_body):
        """
        Initialize the class
        """
        
        # Handles all the ROS related items
        self.ros_interface = ROSInterface(t_cam_to_body)

        # YOUR CODE AFTER THIS
        self.path = dijkstras(occupancy_map, x_spacing, y_spacing, pos_init, pos_goal)
        self.path_index = 1

        # Uncomment as completed
        self.kalman_filter = KalmanFilter(world_map)
        self.diff_drive_controller = DiffDriveController(max_speed, max_omega)
        self.v_last = 0.0
        self.omega_last = 0.0

    def process_measurements(self):
        """ 
        YOUR CODE HERE
        This function is called at 60Hz
        """
        meas = self.ros_interface.get_measurements()
        imu_meas = self.ros_interface.get_imu()

        pose_est = self.kalman_filter.step_filter(self.v_last, imu_meas, meas)
        est_state = np.array([[pose_est[0]],[pose_est[1]],[pose_est[2]]], dtype='float')
        #print("est_state is:", est_state)

        state = est_state
        #print("goal is:", self.path[self.path_index])

        v, omega, done = self.diff_drive_controller.compute_vel(state, self.path[self.path_index])

        self.ros_interface.command_velocity(v, omega)

        if self.path_index >= len(self.path):
            self.ros_interface.done = True # might not be correct syntax
            self.ros_interface.command_velocity(0,0)
            return  # Done!

        self.v_last = v
        self.omega_last = -omega

        if done: self.path_index += 1
        self.ros_interface.command_velocity(v, omega)
        
        return
    
def main(args):
    rospy.init_node('robot_control')

    # Load parameters from yaml
    param_path = rospy.get_param("~param_path")
    f = open(param_path,'r')
    params_raw = f.read()
    f.close()
    params = yaml.load(params_raw)
    occupancy_map = np.array(params['occupancy_map'])
    world_map = np.array(params['world_map'])
    pos_init = np.array(params['pos_init'])
    pos_goal = np.array(params['pos_goal'])
    max_vel = params['max_vel']
    max_omega = params['max_omega']
    t_cam_to_body = np.array(params['t_cam_to_body'])
    x_spacing = params['x_spacing']
    y_spacing = params['y_spacing']

    # Intialize the RobotControl object
    robotControl = RobotControl(world_map,occupancy_map, pos_init, pos_goal, max_vel, max_omega, x_spacing, y_spacing, t_cam_to_body)

    # Call process_measurements at 60Hz
    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        robotControl.process_measurements()
        r.sleep()
    # Done, stop robot
    robotControl.ros_interface.command_velocity(0,0)

if __name__ == "__main__":
    try:
        main(sys.argv)
    except rospy.ROSInterruptException: pass



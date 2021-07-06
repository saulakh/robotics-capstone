## Robotics Specialization - Capstone Project
Courses offered through UPenn and Coursera

## Assembling the Rover
Most of the parts were purchased from Adafruit Industries, through the Robotics Capstone [base kit](https://www.adafruit.com/wishlists/402816). The assembly was modified, but the general [instructions](https://learn.adafruit.com/simple-raspberry-pi-robot/assembly) were available on the Adafruit website.

##### Soldering/IMU
Soldered the IMU to the motor hat, using these 4 connections:

| Motor Hat |	IMU |
|-----------|-----|
| 5V	      | Vin |
| GND	      | GND |
| SCL	      | SCL |
| SDA	      | SDA |

I eventually replaced the Adafruit Precision NXP 9-DOF Breakout Board - FXOS8700 + FXAS21002 ([PID: 3463](https://www.adafruit.com/product/3463)), because this IMU was not included in the [RTIMULibCal](https://github.com/RPi-Distro/RTIMULib) library. To make the IMU data easier to calibrate with the rest of the project, I switched to using the Adafruit LSM9DS1 Accelerometer + Gyro + Magnetometer 9-DOF Breakout ([PID: 3387](https://www.adafruit.com/product/3387)).

![image](https://user-images.githubusercontent.com/74683142/122626134-b36e4a00-d076-11eb-89a8-5038a8c2733d.png) ![image](https://user-images.githubusercontent.com/74683142/122626140-b9642b00-d076-11eb-9e00-1452345fa6af.png) 
![image](https://user-images.githubusercontent.com/74683142/122626145-bf5a0c00-d076-11eb-902c-6446ea38b252.png)

##### Installing the Raspberry Pi OS

The [Pi image](https://d18ky98rnyall9.cloudfront.net/_aaf798f8a9b9dc160c0540dfb59115ae_coursera_robotics_capstone_pi.zip?Expires=1624147200&Signature=BHsg1yA-QXutesEbSiDbe0YPBMxabPAY-t6O5rNgr4SZGHkHT49pOCKr5rkPhCuHMcBH5loYFGurRKMU8R~QhTawQpyp4E4AxnBZLb4Y1K~BjXoSDWNxepsC2Hr4916OZPbuk4XOqJMNbBlLLf5EedUjfv1WvXGZymkSi6kLm~g_&Key-Pair-Id=APKAJLTNE6QMUY6HBC5A) was provided by Coursera, so I installed the image using [Raspberry Pi Imager](https://www.raspberrypi.org/documentation/installation/installing-images/README.md), and inserted the SD card into the Raspberry Pi.

##### Connecting to the Pi
Next, I connected the Pi to a monitor, and configured the WiFi to be able to access the Pi remotely. I also used this command to expand the SD card partition:

`sudo raspi-config`

I used [ssh](https://www.raspberrypi.org/documentation/remote-access/ssh/README.md) to access the Pi from my local machine, and [scp](https://www.raspberrypi.org/documentation/remote-access/ssh/scp.md) to transfer files back and forth as needed.

##### Controlling the Rover
The function RobotControl.py is the main file used to move the rover. Using the Pi image from above, this function can be found in the following folder: 

/home/pi/catkin_ws/src/robot_control/src/ 

This function imports the ROSInterface class, which imports information from the sensors and sends information to the motors. To run the code, two terminal windows are needed.

In the first terminal, the following command can be executed to load the robot.launch file:

`roslaunch robot_launch robot.launch`

The robot.launch file contains the packages that control the robot, including the AprilTag detector, the IMU driver, the camera driver, and the motor driver. In the second terminal, the RobotControl.py function can be launched using:

`roslaunch robot_control robot_control.launch`

## Calibration
The Coursera lecture videos explained each step of the calibration:

##### Camera Calibration
I printed off a checkboard calibration pattern, and used the camera calibration process to find the distortion coefficients of the camera. I used one terminal window to load the robot.launch file, which contains all of the packages for the robot, including the camera. 

With the Pi connected to a monitor, I opened a second terminal window and used the following command to run the calibration (using my checkerboard dimensions):

`rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.0254 image:=/camera/image_raw camera:=/camera`

Running the calibration outputs the camera matrix, distortion coefficients, rectification matrix, and projection matrix. I saved these values to the following file:

/home/pi/catkin_ws/src/raspicam_node/calibrations/camera.yaml

##### Motor Calibration
The file motor_params.yaml changes the motor gains, to make sure the robot is moving the correct distance given a velocity input. The parameter motor_gain is the ratio between the gear ratio and max rpm. Increasing this value increases the distance the rover travels within a second at a given velocity. To calibrate the motor, I set the velocity to 30cm/s, and ran RobotControl.py for one second. I tuned the motor gain to get the robot to travel as close as possible to the 30cm mark, and saved it in this file:

/home/pi/catkin_ws/src/dc_motor_driver/params/motor_params.yaml

##### Camera to Body Calibration

To calibrate the translation from the camera frame to the center of the robot, I updated the t_cam_to_body values in the following file of the Pi:

/home/pi/catkin_ws/src/robot_control/params/params.yaml

##### IMU Calibration
I calibrated the IMU accelerometers by navigating to this folder in the Pi:

/home/pi/catkin_ws/src/imu_rtimulib/params/

and running the following program:

`RTIMULibCal`

##### April Tags
I downloaded and printed the AprilTags with this [link](https://d18ky98rnyall9.cloudfront.net/_04773bdf24b984103a66dffa51b391ec_apriltags_0_thru_15.pdf?Expires=1624147200&Signature=I1Q8Q5Nad8f0T99GlJhQjqTawnxPbdgYHD~18PEcr29UIDypEUr0RiC7alRY5fyNPxu9Ua-JTwXG49UvwQSktsvxg7AyH9nc2m5jDWDmcj8l8WPLHJHhUsOpLRqbt3ReRWCqqKlJcZpafboyEXo39BHQOT6-RxEmPGkb7ksG7hM_&Key-Pair-Id=APKAJLTNE6QMUY6HBC5A). I resized the tags, and saved the dimensions in this file on the Pi:

/home/pi/catkin_ws/src/robot_launch/launch/tag_detection.launch

## Differential Drive Controller
The motor calibration used open loop control, so there was no feedback to regulate how far the robot had travelled. I used a Differential Drive controller to create a closed loop feedback, which helped to minimize this gap. By defining the AprilTag position as the goal, the robot was able to move closer to the desired goal, and the gains were tuned to change the speed of convergence for each variable. I saved these values in the DiffDriveController.py file. Using the Pi, this file is located in the following folder:

/home/pi/catkin_ws/src/robot_control/src/

## Extended Kalman Filter
I used an extended Kalman Filter to better track the position of the robot throughout the map. The Kalman Filter used a prediction step to estimate where the robot was, based on its output velocity commands and IMU accelerometer data. Whenever an AprilTag was within view, the measurement was used to update the Kalman filter prediction with a much higher accuracy, and the pose estimation jumped to this updated location. The more often AprilTags were within view, the more accurately the robot was able to move through the map. If the robot relied on its prediction step for too long, without any updated measurements, the robot would often miscalculate its own pose estimation. Using the Pi, the KalmanFilter.py file can also be found in:

/home/pi/catkin_ws/src/robot_control/src/

## Shortest Path
I used Dijsktra's algorithm to help the robot find the shortest path to the goal, and maneuver around obstacles. For this project, the measurements from the locations of the AprilTags and the map designed for the robot to travel were hardcoded into params.yaml. Using the Pi, the ShortestPath.py file can also be found in:

/home/pi/catkin_ws/src/robot_control/src/

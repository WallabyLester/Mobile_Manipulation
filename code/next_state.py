# Example inputs 
# # init the initial config of the youBot
# curr_config = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])

# # speeds vector
# thetadot = np.array([0, 0 ,0 ,0 ,0])
# u = np.array([10, 10, 10, 10])      # robot drives forward
# # u = np.array([-10, 10, -10, 10])  # robot slide sideways
# # u = np.array([-10, 10, 10, -10])  # robot spins counterclockwise
# speeds = np.hstack((thetadot, u))

# max_speed = 5
# dt = 0.01
# time = 1              # total time (sec)
# iters = int(time/dt)  # number of iterations

# # loop through iterations of NextState
# config = np.zeros((iters, 13))      # init config array 
# config[0] = curr_config

# for i in range(1, iters):
#     curr_config = NextState(curr_config, speeds, dt, max_speed)
#     config[i][:12] = curr_config

import numpy as np
import csv 
from numpy import sin, cos

""" 
Simulator for the kinematics of the youBot

Computes the configuration of the robot for each time step

"""

def NextState(curr_config, speeds, dt, max_speed):
    """ Function to compute the configuration of the robot

    The NextState function is based on a simple first-order Euler step:
    - new arm joint angles = (old arm joint angles) + (joint speeds) * Δt
    - new wheel angles = (old wheel angles) + (wheel speeds) * Δt
    - new chassis configuration is obtained from odometry

    Args:
        curr_config : Current configuration of the robot 
                      (phi,x,y,J1,J2,J3,J4,J5,W1,W2,W3,W4,gripper)
        speeds (radians/s): Arm joint speeds and wheel speeds (thetadot, u)
        dt : Timestep
        max_speed (m/s) : The maximum positive and negative angular speed of the arm 
                    joints and wheels

    Returns: 
        config : The new configuration of the robot
                 (phi,x,y,J1,J2,J3,J4,J5,W1,W2,W3,W4,gripper)
    """
    # initialize kinematics variables
    l = 0.47/2  # forward-backward distance between the wheels (m)
    w = 0.3/2   # side-to-side distance between wheels (m)
    r = 0.0475  # radius of each wheel (m)

    # current chassis config, joint angles, wheel angles
    curr_chassis = curr_config[:3]
    curr_joint_ang = curr_config[3:8]
    curr_wheel_ang = curr_config[8:12]

    # limit the speeds based on the max speed
    for i in range(len(speeds)):
        if speeds[i] > max_speed:
            speeds[i] = max_speed
        elif speeds[i] < -max_speed:
            speeds[i] = -max_speed

    # current speeds after limiting
    thetadot = speeds[:5]     # joint speeds
    u = speeds[5:]            # wheel speeds

    # first order Euler step equations 
    new_joint_ang = curr_joint_ang + thetadot * dt
    new_wheel_ang = curr_wheel_ang + u * dt

    # new chassis config based on odometry
    F = r/4 * np.array([[-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)],
                        [       1,       1,       1,        1], 
                        [      -1,       1,      -1,        1]])
    Vb = F @ (u * dt).T
    wbz, vbx, vby = Vb      # Vb6 is [0, 0, wbz, vbx, vby, 0]
    
    if wbz == 0:
        deltaqb = np.array([0, vbx, vby])
    else:
        deltaqb = np.array([                   wbz, 
                            (vbx*sin(wbz) + vby*(cos(wbz) - 1))/wbz,
                            (vby*sin(wbz) + vbx*(1 - cos(wbz)))/wbz])
    
    # transforming deltaqb in {b} frame to deltaq in {s} frame using the chassis angle
    chassis_angle = curr_chassis[0]
    T = np.array([[1,                  0,                   0], 
                  [0, cos(chassis_angle), -sin(chassis_angle)],
                  [0, sin(chassis_angle), cos(chassis_angle)]])
    deltaq = T @ deltaqb

    # calculate updated odometry estimate
    q = curr_chassis + deltaq
    
    config = np.hstack((q, new_joint_ang, new_wheel_ang))
    
    return config


if __name__ == "__main__":
    """ Main function for running the NextState function
    """
    # init the initial config of the youBot
    curr_config = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])

    # speeds vector
    thetadot = np.array([0, 0, 0, 0, 0])
    u = np.array([10, 10, 10, 10])      # robot drives forward
    # u = np.array([-10, 10, -10, 10])  # robot slide sideways
    # u = np.array([-10, 10, 10, -10])  # robot spins counterclockwise
    speeds = np.hstack((thetadot, u))

    max_speed = 5
    dt = 0.01
    time = 1              # total time (sec)
    iters = int(time/dt)  # number of iterations
 
    # loop through iterations of NextState
    config = np.zeros((iters, 13))      # init config array 
    config[0] = curr_config

    for i in range(1, iters):
        curr_config = NextState(curr_config, speeds, dt, max_speed)
        config[i][:12] = curr_config

    # write to csv file
    with open("next_state.csv", "w+") as my_csv:
        csvWriter = csv.writer(my_csv, delimiter=",")
        csvWriter.writerows(config)
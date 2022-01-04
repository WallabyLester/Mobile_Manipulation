# Example call with inputs built in
# MobileManipulation()
# Code call using terminal
# python3 newTask.py

from modern_robotics.core import FKinBody
import numpy as np
import modern_robotics as mr
import csv 
import matplotlib.pyplot as plt
from numpy import sin, cos, pi

from next_state import NextState
from trajectory_generator import TrajectoryGenerator
from feedback_control import FeedbackControl

import logging

""" Code to run full mobile manipulation of the youBot

    Makes use of the NextState, TrajectoryGenerator, and FeedbackControl functions to
    pick and place a cube with the youBot. 

"""

# Creating log file
logging.basicConfig(filename='best.log', level=logging.INFO)

def MobileManipulation():
    """ Main function to run the manipulation task

    Generates the robot's trajectory using the listed functions

    Args:
        None
    
    Returns:
        A csv file for the configurations of the youBot
        A saved plot of the Xerr over time
    """
    # Current configuration of the robot (phi,x,y,J1,J2,J3,J4,J5,W1,W2,W3,W4,gripper)
    curr_config = np.array([0.1, -0.5, 0.1, 0.2, -0.2, 0.6, -0.2, 0, 0, 0, 0, 0, 0])


    # the initial configuration of the end effector in the reference trajectory
    Tse_init = np.array([[ 0, 0, 1,   0], 
                         [ 0, 1, 0,   0], 
                         [-1, 0, 0, 0.5],
                         [ 0, 0, 0,   1]])

    # the cube's initial configuration
    Tsc_init = np.array([[1, 0, 0,     1],
                         [0, 1, 0,     1],
                         [0, 0, 1, 0.025],
                         [0, 0, 0,     1]])

    # the cube's desired final configuration
    Tsc_final = np.array([[ 0, 1, 0,     1],
                          [-1, 0, 0,    -1],
                          [ 0, 0, 1, 0.025],
                          [ 0, 0, 0,     1]])

    # the end effector's configuration relative to the cube when grasping
    Tce_grasp = np.array([[-np.sqrt(2)/2, 0,  np.sqrt(2)/2, 0],
                          [ 0, 			  1,  0, 			0],
                          [-np.sqrt(2)/2, 0, -np.sqrt(2)/2, 0],
                          [ 0, 			  0,  0,          	1]])

    # the end effector's standoff above the cube, before and after grasping
    Tce_standoff = np.array([[-np.sqrt(2)/2,  0,  np.sqrt(2)/2,    0],
                             [ 0, 			  1,  0, 			   0],
                             [-np.sqrt(2)/2,  0, -np.sqrt(2)/2, 0.15],
                             [ 0, 			  0,  0,          	   1]])

    # the fixed offset from the chassis frame {b} to the base frame of the arm {0}
    Tb0 = np.array([[1, 0, 0, 0.1662],
                    [0, 1, 0,      0],
                    [0, 0, 1, 0.0026],
                    [0, 0, 0,      1]])

    # end-effector frame {e} relative to the arm base frame {0}
    M0e = np.array([[1, 0, 0,  0.033],
                    [0, 1, 0,      0],
                    [0, 0, 1, 0.6546],
                    [0, 0, 0,      1]])

    # the screw axes for the five joints in the end-effector frame {e}
    Blist = np.array([[0,  0, 1,       0, 0.033, 0],
                      [0, -1, 0, -0.5076,     0, 0],
                      [0, -1, 0, -0.3526,     0, 0],
                      [0, -1, 0, -0.2176,     0, 0],
                      [0,  0, 1,       0,     0, 0]]).T

    # feedback control 
    # kp gain and ki gain
    Kp_gain = 20
    Kp = np.identity(6) * Kp_gain
    logging.info(f"Kp_gain: {Kp_gain}")
    Ki_gain = 1 
    logging.info(f"Ki_gain: {Ki_gain}")
    Ki = np.identity(6) * Ki_gain

    # simulation constants
    k = 1                 # the number of trajectory configurations per 0.01 seconds
    max_speed = 50   
    dt = 0.01
    time = 15             # total time (sec)
    iters = int(time/dt)  # number of iterations

    Xerr_integral = np.zeros(6)
    config = np.zeros((iters, 13))
    Xerr_plot = np.zeros((iters, 6))

    config[0] = curr_config

    # Compute the trajectory
    logging.info("Generating trajectory")
    traj_list = TrajectoryGenerator(Tse_init, Tsc_init, Tsc_final, Tce_grasp, 
                        Tce_standoff, k)
    logging.info("Finished generating trajectory")
    
    for i in range(1, iters-1):
        curr_config = config[i-1, :]

        Tsb = np.array([[cos(curr_config[0]), -sin(curr_config[0]), 0, curr_config[1]],
                        [sin(curr_config[0]),  cos(curr_config[0]), 0, curr_config[2]],
                        [                  0,                    0, 1,         0.0963],
                        [                  0,                    0, 0,              1]])
        
        # find current joint angles
        curr_joint_ang = curr_config[3:8]

        T0e = FKinBody(M0e, Blist, curr_joint_ang)

        # computing X
        Tbe = Tb0@T0e
        X = Tsb@Tbe
        # print(f"X: {X}")

        Xd = np.array([[traj_list[i][0], traj_list[i][1], traj_list[i][2],  traj_list[i][9]],
                       [traj_list[i][3], traj_list[i][4], traj_list[i][5], traj_list[i][10]],
                       [traj_list[i][6], traj_list[i][7], traj_list[i][8], traj_list[i][11]],
                       [              0,               0,               0,                1]])
        # print(f"Xd: {Xd}")
        Xd_next = np.array([[traj_list[i+1][0], traj_list[i+1][1], traj_list[i+1][2],  traj_list[i+1][9]],
                            [traj_list[i+1][3], traj_list[i+1][4], traj_list[i+1][5], traj_list[i+1][10]],
                            [traj_list[i+1][6], traj_list[i+1][7], traj_list[i+1][8], traj_list[i+1][11]],
                            [                0,                 0,                 0,                  1]])
        # print(f"Xd_next: {Xd_next}")

        # compute the control law
        V, speeds, Xerr = FeedbackControl(X, Xd, Xd_next, Kp, Ki, dt, curr_config, Xerr_integral)
        
        # swap thetadot and u for NextState
        thetadot = speeds[4:9]
        u = speeds[:4]
        control_speeds = np.hstack((thetadot,u))
        
        # compute the next configuration
        # curr_config = NextState(curr_config[:12], control_speeds, dt, max_speed)
        curr_config = NextState(curr_config, control_speeds, dt, max_speed)
        config[i] = np.hstack((curr_config, traj_list[i][12]))

        # store the Xerr for plotting
        Xerr_plot[i-1] = Xerr
        
    # write configurations to csv file
    logging.info("Generating full configuration csv file")
    with open("mobile_manipulation.csv", "w+") as my_csv:
        csvWriter = csv.writer(my_csv, delimiter=",")
        csvWriter.writerows(config)

    # write Xerr to csv file
    logging.info("Generating Xerr csv file")
    with open("Xerr.csv", "w+") as my_csv:
        csvWriter = csv.writer(my_csv, delimiter=",")
        csvWriter.writerows(Xerr_plot)

    # plot the error as a function of time
    logging.info("Plotting Xerr")
    x_traj = np.linspace(1, 15, 1500)
    plt.figure()
    plt.plot(x_traj, Xerr_plot[:, 0], label='Xerr0')
    plt.plot(x_traj, Xerr_plot[:, 1], label='Xerr1')
    plt.plot(x_traj, Xerr_plot[:, 2], label='Xerr2')
    plt.plot(x_traj, Xerr_plot[:, 3], label='Xerr3')
    plt.plot(x_traj, Xerr_plot[:, 4], label='Xerr4')
    plt.plot(x_traj, Xerr_plot[:, 5], label='Xerr5')
    plt.title(f"Error evolution over time with kp={Kp_gain} and ki={Ki_gain}")
    plt.xlabel("Time (s)")
    plt.xlim([1,15])
    plt.ylabel("Error")
    plt.legend(loc="best")
    plt.savefig(f"Error evolution over time with kp={Kp_gain} and ki={Ki_gain}.png")
    plt.show()

if __name__ == "__main__":
    logging.info("python3 mobile_manipulation.py")
    logging.info("Running full program")
    MobileManipulation()
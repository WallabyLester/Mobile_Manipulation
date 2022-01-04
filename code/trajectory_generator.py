# Example inputs and code call:
#
# Tse_init = np.array([[ 0, 0, 1, 0], 
#                      [ 0, 1, 0, 0], 
#                      [-1, 0, 0, 1],
#                      [ 0, 0, 0, 1]])
# Tsc_init = np.array([[1, 0, 0,     1],
#                      [0, 1, 0,     0],
#                      [0, 0, 1, 0.025],
#                      [0, 0, 0,     1]])
# Tsc_final = np.array([[0, 1, 0,     0],
#                       [-1, 0, 0,    -1],
#                       [0,  0, 1, 0.025],
#                       [0,  0, 0,     1]])
# Tce_grasp = np.array([[-np.sqrt(2)/2, 0,  np.sqrt(2)/2, 0],
#                       [ 0, 			1,  0, 			  0],
#                       [-np.sqrt(2)/2, 0, -np.sqrt(2)/2, 0],
#                       [ 0, 			0,  0,            1]])
# Tce_standoff = np.array([[-np.sqrt(2)/2,  0,  np.sqrt(2)/2,   0],
#                          [ 0, 			1,  0, 			    0],
#                          [-np.sqrt(2)/2,  0, -np.sqrt(2)/2, 0.2],
#                          [ 0, 			0,  0,          	1]])
# k = 1
# # initializing the trajectory list
# traj_list = []

# traj_list = TrajectoryGenerator(Tse_init, Tsc_init, Tsc_final, Tce_grasp, 
#                     Tce_standoff, k=1)

import numpy as np
import modern_robotics as mr
from modern_robotics.core import CubicTimeScaling, QuinticTimeScaling, MatrixExp6, \
                                 MatrixLog6, TransInv
import csv

"""
Code to output the trajectory needed for an end effector of the youBot to pick and place a cube
Contains two functions: ScrewTrajectoryList
                        TrajectoryGenerator 

"""
def ScrewTrajectoryList(Xstart, Xend, Tf, N, method, gripper_state, traj_list):
    """ Modified from the modern_robotics library ScrewTrajectory
    
    Computes a trajectory as a list of SE(3) matrices with a gripper value and
    converts into a list of lists

    Args:
        Xstart : The initial end-effector configuration
        Xend : The final end-effector configuration
        Tf : Total time of the motion in seconds from rest to rest
        N : The number of points N > 1 in the discrete representation of the trajectory
        method : The time-scaling method 
        gripper_state : The gripper open (0) and closed (1) value

    Returns:
        traj_list : list of rotations, positions, and gripper state
    """
    N = int(N)
    timegap = Tf / (N - 1.0)
    traj = [[None]] * N
    for i in range(N):
        if method == 3:
            s = CubicTimeScaling(Tf, timegap * i)
        else:
            s = QuinticTimeScaling(Tf, timegap * i)
        traj[i] = np.dot(Xstart, MatrixExp6(MatrixLog6(np.dot(TransInv(Xstart), Xend)) * s))
    traj = np.asarray(traj)

    for i in range(N):
        r11 = traj[i][0][0]
        r12 = traj[i][0][1]
        r13 = traj[i][0][2]
        r21 = traj[i][1][0]
        r22 = traj[i][1][1]
        r23 = traj[i][1][2]
        r31 = traj[i][2][0]
        r32 = traj[i][2][1]
        r33 = traj[i][2][2]
        px = traj[i][0][3]
        py = traj[i][1][3]
        pz = traj[i][2][3]
        
        traj_list.append([r11, r12, r13, r21, r22, r23, r31, r32, r33, px, py, pz, gripper_state])
        
    return traj_list

def TrajectoryGenerator(Tse_init, Tsc_init, Tsc_final, Tce_grasp, 
                        Tce_standoff, k):
    """ Generates the reference trajectory for the end effector frame. Each trajectory segment 
        begins and ends at rest. 

    Args:
        Tse_init : the initial configuration of the end effector in the reference trajectory
        Tsc_init : the cube's initial configuration
        Tsc_final : the cube's desired final configuration
        Tce_grasp : the end effector's configuration relative to the cube when grasping
        Tce_standoff : the end effector's standoff above the cube, before and after grasping
        k : the number of trajectory reference configurations per 0.01 seconds

    Returns:
        traj_list : the representation of the N configurations of the end 
                    effector along the concatenated eight segment trajectory
    """
    Tf = 5                      # total time of motion (sec)
    method = 5                  # polynomial time scaling
    N_grasping = 1*k / 0.01     # the number of points in the trajectory given the action
    N_moving = method*k / 0.01
    N_gripping = 0.5*k / 0.01 

    # initial config of the end effector above the cube, relative to {s}
    Tse_standoff = Tsc_init @ Tce_standoff

    # end effector (ee) config when grasping, relative to {s}
    Tse_grasp = Tsc_init @ Tce_grasp
    
    # ee config when above final config
    Tse_goal = Tsc_final @ Tce_standoff

    # final config of ee relative to {s}
    Tse_final = Tsc_final @ Tce_grasp

    traj_list = []

    ########################## Generating Trajectory ##########################
    # Trajectory 1: initial position -> standoff position
    gripper_state = 0
    traj_list = ScrewTrajectoryList(Xstart=Tse_init, Xend=Tse_standoff, Tf=Tf, N=N_moving, 
                                method=method, gripper_state=gripper_state, traj_list=traj_list)
    
    # Trajectory 2: standoff position -> grasp position 
    gripper_state = 0
    traj_list = ScrewTrajectoryList(Xstart=Tse_standoff, Xend=Tse_grasp, Tf=Tf, N=N_grasping, 
                                method=method, gripper_state=gripper_state, traj_list=traj_list)

    # Trajectory 3: gripping
    gripper_state = 1
    traj_list = ScrewTrajectoryList(Xstart=Tse_grasp, Xend=Tse_grasp, Tf=Tf, N=N_gripping, 
                                method=method, gripper_state=gripper_state, traj_list=traj_list)

    # Trajectory 4: grasp position -> standoff position
    gripper_state = 1
    traj_list = ScrewTrajectoryList(Xstart=Tse_grasp, Xend=Tse_standoff, Tf=Tf, N=N_grasping, 
                                method=method, gripper_state=gripper_state, traj_list=traj_list)

    # Trajectory 5: standoff position -> goal position
    gripper_state = 1
    traj_list = ScrewTrajectoryList(Xstart=Tse_standoff, Xend=Tse_goal, Tf=Tf, N=N_moving, 
                                method=method, gripper_state=gripper_state, traj_list=traj_list)

    # Trajectory 6: goal position -> final position
    gripper_state = 1
    traj_list = ScrewTrajectoryList(Xstart=Tse_goal, Xend=Tse_final, Tf=Tf, N=N_grasping, 
                                method=method, gripper_state=gripper_state, traj_list=traj_list)
    
    # Trajectory 7: opening gripper 
    gripper_state = 0
    traj_list = ScrewTrajectoryList(Xstart=Tse_final, Xend=Tse_final, Tf=Tf, N=N_gripping, 
                                method=method, gripper_state=gripper_state, traj_list=traj_list)
    
    # Trajectory 8: final position -> above position
    gripper_state = 0
    traj_list = ScrewTrajectoryList(Xstart=Tse_final, Xend=Tse_goal, Tf=Tf, N=N_grasping, 
                                method=method, gripper_state=gripper_state, traj_list=traj_list)
    
    return traj_list
    
if __name__ == "__main__":
    """ Main function to call the functions above
    """
    Tse_init = np.array([[ 0, 0, 1, 0], 
                         [ 0, 1, 0, 0], 
                         [-1, 0, 0, 1],
                         [ 0, 0, 0, 1]])
    Tsc_init = np.array([[1, 0, 0,     1],
                         [0, 1, 0,     0],
                         [0, 0, 1, 0.025],
                         [0, 0, 0,     1]])
    Tsc_final = np.array([[ 0, 1, 0,     0],
                          [-1, 0, 0,    -1],
                          [ 0, 0, 1, 0.025],
                          [ 0, 0, 0,     1]])
    Tce_grasp = np.array([[-np.sqrt(2)/2, 0,  np.sqrt(2)/2, 0],
                          [ 0, 			  1,  0, 			0],
                          [-np.sqrt(2)/2, 0, -np.sqrt(2)/2, 0],
                          [ 0, 			  0,  0,          	1]])
    Tce_standoff = np.array([[-np.sqrt(2)/2,  0,  np.sqrt(2)/2,    0],
                             [ 0, 			  1,  0, 			   0],
                             [-np.sqrt(2)/2,  0, -np.sqrt(2)/2, 0.15],
                             [ 0, 			  0,  0,          	   1]])
    k = 1

    # initializing the trajectory list
    traj_list = []
    
    traj_list = TrajectoryGenerator(Tse_init, Tsc_init, Tsc_final, Tce_grasp, 
                        Tce_standoff, k=1)
    
    # with open("/home/androo/Documents/ME449_RoboticManipulation/Final_Project/milestone2.csv", "w+") as my_csv:
    #     csvWriter = csv.writer(my_csv, delimiter=",")
    #     csvWriter.writerows(traj_list)
    np.savetxt("milestone2.csv", np.asarray(np.c_[traj_list]), delimiter = ",") 
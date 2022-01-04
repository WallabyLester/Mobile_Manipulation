import numpy as np
import modern_robotics as mr
from modern_robotics.core import Adjoint, FKinBody, JacobianBody, MatrixLog6, se3ToVec, TransInv

"""
Code to calculate the feedforward control for the youBot

"""

def FeedbackControl(X, Xd, Xd_next, Kp, Ki, dt, curr_config, Xerr_integral):
    """ Calculates the kinematic task-space feedforward and feedback control law

    Makes use of the equation: V(t) = [Adx^-1xd]Vd(t) + KpXerr(t) + Ki*integral(0:t)(Xerr(t))dt

    Args:
        X : Current actual end-effector configuration Tse
        Xd : Current end-effector reference configuration Tse,d
        Xd_next : End-effector reference configuration at the next timestep in 
                  the reference trajectory Xd at Δt later
        Kp : The feedback proportional gain 
        Ki : The feedback integral gain 
        dt : Timestep Δt between reference trajectory configurations
        curr_config : The current configuration of the robot
        Xerr_integral : Initial integral of the error (zeros)

    Returns:
        V : End-effector twist expressed in end-effector frame {e}
        Controls : The commanded wheel and arm joint speeds (m/s)
        Xerr : Error in X
    """
    # initialize kinematics variables
    l = 0.47/2  # forward-backward distance between the wheels (m)
    w = 0.3/2   # side-to-side distance between wheels (m)
    r = 0.0475  # radius of each wheel (m)

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

    # find current joint angles
    curr_joint_ang = curr_config[3:8]

    # transformation from {0} to {e}
    T0e = FKinBody(M0e, Blist, curr_joint_ang)

    # transformation from {e} to {b} 
    Teb = TransInv(T0e)@TransInv(Tb0)

    # compute the reference twist Vd
    Log = MatrixLog6(TransInv(Xd)@Xd_next)
    Vel = se3ToVec(Log)
    Vd = 1/dt * Vel
    # print(f"Vd: {Vd}")

    # compute the Ad(x^-1xd) matrix
    Adx_invxd = Adjoint(TransInv(X)@Xd)

    Adx_invxdVd = Adx_invxd@Vd     # 6x6 @ 6x1 = 6x1
    # print(f"Adx_invxdVd: {Adx_invxdVd}")

    # compute X error
    Xerr = se3ToVec(MatrixLog6(TransInv(X)@Xd))
    # print(f"Xerr: {Xerr}")

    # compute the integral of the error
    Xerr_integral += Xerr * dt
    # print(f"Integral of error: {Xerr_integral}")

    # compute V
    V = Adx_invxdVd + Kp@Xerr + Ki@Xerr_integral
    # print(f"V: {V}")

    # F6 matrix 
    F6 = r/4 * np.array([[       0,       0,       0,        0],
                         [       0,       0,       0,        0],
                         [-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)],
                         [       1,       1,       1,        1], 
                         [      -1,       1,      -1,        1],
                         [       0,       0,       0,        0]])

    # arm jacobian
    J = JacobianBody(Blist, curr_joint_ang)

    # # joint limits
    # J_limit = J.T
    # if curr_joint_ang[0] < -2.95 or curr_joint_ang[0] > 2.95:
    #     J_limit[0] = J_limit[0]*0
    # if curr_joint_ang[1] < -1 or curr_joint_ang[1] > 1:
    #     J_limit[1] = J_limit[1] * 0
    # if curr_joint_ang[2] < -2 or curr_joint_ang[2] > 2:
    #     J_limit[2] = J_limit[2] * 0
    # if curr_joint_ang[3] < -2 or curr_joint_ang[3] > 2:
    #     J_limit[3] = J_limit[3] * 0
    # if curr_joint_ang[4] < -2.92 or curr_joint_ang[4] > 2.92:
    #     J_limit[4] = J_limit[4] * 0
    # J = J_limit.T
    
    # body jacobian
    Jb = Adjoint(Teb) @ F6
    Je = np.hstack((Jb, J))  ### make joint column zero depending on config to place joint limits
    # print(f"Je: \n{np.around(Je, decimals=3)}")

    # calculate the commanded wheel and arm joint speeds: u and thetadot
    # using the Moore-Penrose pseudoinverse
    # Je_pinv = Je.T@np.linalg.inv(Je@Je.T)
    Je_pinv = np.linalg.pinv(Je, 1e-3)
    # Je_pinv = np.linalg.pinv(Je)
    controls = Je_pinv@V
    # print(f"Controls: {np.around(controls, decimals=1)}")

    return V, controls, Xerr


if __name__ == "__main__":
    """ Main function to call FeedbackControl
    """
    Xd = np.array([[ 0, 0, 1, 0.5],
                   [ 0, 1, 0,   0],
                   [-1, 0, 0, 0.5],
                   [ 0, 0, 0,   1]])
    Xd_next = np.array([[ 0, 0, 1, 0.6],
                        [ 0, 1, 0,   0],
                        [-1, 0, 0, 0.3],
                        [ 0, 0, 0,   1]])
    X = np.array([[ 0.170, 0, 0.985, 0.387],
                  [     0, 1,     0,     0],
                  [-0.985, 0, 0.170, 0.570],
                  [     0, 0,     0,     1]])
    # Kp = np.zeros((6,6))
    Kp = np.identity(6)
    Ki = np.zeros((6,6))
    # Ki = np.identity(6)
    dt = 0.01
    curr_config = np.array([0, 0, 0, 0, 0, 0.2, -1.6, 0])
    Xerr_integral = np.zeros(6)

    V, speeds, Xerr = FeedbackControl(X, Xd, Xd_next, Kp, Ki, dt, curr_config, Xerr_integral)

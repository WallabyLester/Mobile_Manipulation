# Mobile Manipulation
**Andru Liu**

This repository contains the code for simulating a pick and place task with the youBot. 

## Software
The `code` directory contains the code for running the system. The file `mobile_manipulation.py` is the main code to run through and create the youBot configurations. `next_state.py` gives the new configuration of the robot, `trajectory_generator.py` generates the reference trajectory of the end effector, and `feedback_control.py` computes the control law based on the configuration and Kp and Ki. 

To run each separate task in order to attain the results, use the following scripts: 

Best `python3 best.py`

Overshoot `python3 overshoot.py`

newTask `python3 newTask.py`

## Results
The `results` directory contains three sets of results:

`best` contains the optimally controlled system. The system used feedforward with PI control. `Kp_gain = 1.3` and `Ki_gain = 0.01` were used to achieve an Xerr of 0 partway through the first trajectory (moving to the standoff position above the cube at 5 seconds). `best.log` and `best.txt` contain logs of the commands used to run the program. Initial inputs are saved within the file `mobile_manipulation.py`. The Xerr plot vs. Time(s) is displayed in the `.png` file and is labeled with the Kp and Ki values used. The `mobile_manipulation.csv` and `Xerr.csv` are the csv files containing the configurations and Xerr, respectively. `best.mp4` is a video recording of the task. 

`overshoot` contains a system that was less-well-tuned. A feedforward with PI controller was also used. `Kp_gain = 2` and `Ki_gain = 7` were used to achieve a slight overshoot of the robot and oscillation until it eventually picked up the cube and perfectly placed it with 0 Xerr by the end. `overshoot.log` and `overshoot.txt` contain logs of the commands used to run the program. Initial inputs are saved within the file `mobile_manipulation.py`. The Xerr plot vs. Time(s) is displayed in the `.png` file and is labeled with the Kp and Ki values used. The `mobile_manipulation.csv` and `Xerr.csv` are the csv files containing the configurations and Xerr, respectively. `overshoot.mp4` is a video recording of the task. 

`newTask` contains a system where the initial configuration of the cube was moved. The cube positions were changed to:
```
Tsc_init = np.array([[1, 0, 0,     1],
                     [0, 1, 0,     1],
                     [0, 0, 1, 0.025],
                     [0, 0, 0,     1]])

Tsc_final = np.array([[ 0, 1, 0,     1],
                      [-1, 0, 0,    -1],
                      [ 0, 0, 1, 0.025],
                      [ 0, 0, 0,     1]])
```
adding 1 to the initial y position and 1 to the final x position. `Kp_gain = 20` and `Ki_gain = 1` were used to perform the task with zero error being achieved before partway through the first trajectory. `newTask.log` and `newTask.txt` contain logs of the commands used to run the program. Initial inputs are saved within the file `mobile_manipulation.py`. The Xerr plot vs. Time(s) is displayed in the `.png` file and is labeled with the Kp and Ki values used. The `mobile_manipulation.csv` and `Xerr.csv` are the csv files containing the configurations and Xerr, respectively. `newTask.mp4` is a video recording of the task. 

There is also the `joint_limits` directory which contains two videos: `without_joint_limits.mp4` and `with_joint_limits.mp4`. The first is a task where the robot experiences collisions and violates joint limits. The second is the same task, but with joint limits and singularities accounted for. The joint limit values are accounted for in `feedback_control.py`, as are the singularities in the Numpy `pinv` call with the conditional of `1e-3`. The code is below. The joint limits themselves were found from the youBot documentation, Vrep scene 3, and from testing.

```
# If joint limits are violated, the column of the Jacobian is made 
to be 0, which removes it from calculations of the pseudo inverse
if curr_joint_ang[0] < -2.95 or curr_joint_ang[0] > 2.95:
    J_limit[0] = J_limit[0]*0
if curr_joint_ang[1] < -1.13 or curr_joint_ang[1] > 1:
    J_limit[1] = J_limit[1] * 0
if curr_joint_ang[2] < -3 or curr_joint_ang[2] > 3:
    J_limit[2] = J_limit[2] * 0
if curr_joint_ang[3] < -5 or curr_joint_ang[3] > 5:
    J_limit[3] = J_limit[3] * 0
if curr_joint_ang[4] < -2.92 or curr_joint_ang[4] > 2.92:
    J_limit[4] = J_limit[4] * 0

# Pseudoinverse 
Je_pinv = np.linalg.pinv(Je, 1e-3)
```

### Best
![mobile manipulation](/results/gifs/mobile-manipulation.gif)

### Overshoot
![overshoot](/results/gifs/overshoot.gif)

### New Task
![new task](/results/gifs/new-task.gif)

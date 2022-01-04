# newTask
**Andru Liu**

This code uses a feedforward with PI controller. 

The following inputs are used: 

```
curr_config = np.array([0.1, -0.5, 0.1, 0.2, -0.2, 0.6, -0.2, 0, 0, 0, 0, 0, 0])
    
Tse_init = np.array([[ 0, 0, 1,   0], 
                     [ 0, 1, 0,   0], 
                     [-1, 0, 0, 0.5],
                     [ 0, 0, 0,   1]])

Tsc_init = np.array([[1, 0, 0,     1],
                     [0, 1, 0,     1],
                     [0, 0, 1, 0.025],
                     [0, 0, 0,     1]])

Tsc_final = np.array([[ 0, 1, 0,     1],
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

Kp_gain = 20 
Kp = np.identity(6) * Kp_gain
Ki_gain = 1    
Ki = np.identity(6) * Ki_gain

k = 1              
max_speed = 50
dt = 0.01
time = 15         
```

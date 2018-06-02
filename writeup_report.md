# Project: Control of a 3D Quadrotor

## Required Steps for a Passing Submission:
1. Body rate and roll/pitch control (scenario 2)
2. Position/velocity and yaw angle control (scenario 3)
3. Non-idealities and robustness (scenario 4)
4. Tracking trajectories (scenario 5)

## [Rubric](https://review.udacity.com/#!/rubrics/1643/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.


## 1. Task 1 Body Rate & Roll/Pitch Control (Scenario 2)
This task involves the following:

* implement the code in the function GenerateMotorCommands()
* implement the code in the function BodyRateControl()
* Tune kpPQR in QuadControlParams.txt to get the vehicle to stop spinning quickly but not * overshoot 
* implement the code in the function RollPitchControl()
* Tune kpBank in QuadControlParams.txt to minimize settling time but avoid too much overshoot

### 1.1. GenerateMotorCommands

In order calculate the thrust of each rotor, the following equations were applied.

```
tx = (F1 - F2 + F3 - F4) * l
ty = (F1 + F2 - F3 - F4) * l
tz = t1 + t2 + t3 + t4
tz = (-F1 + F2 + F3 - F4) * (1 / k)
F = F1 + F2 + F3 + F4

tx / l = (F1 - F2 + F3 - F4)
ty / l = (F1 + F2 - F3 - F4)
(tx + ty) / l =  2F1 - 2F4
(tx - ty) / l = -2F2 + 2F3

F      =  F1 + F2 + F3 + F4
tz / k = -F1 + F2 + F3 - F4

F + tz / k = 2F2 + 2F3
F - tz / k = 2F1 + 2F4

F - tz / k + (tx + ty) / l = 4F1
F + tz / k - (tx - ty) / l = 4F2
F + tz / k + (tx - ty) / l = 4F3
F - tz / k - (tx + ty) / l = 4F4


F - tz / k + tx / l  + ty / l = 4F1
F + tz / k - tx / l  + ty / l = 4F2
F + tz / k + tx / l  - ty / l = 4F3
F - tz / k - tx / l  - ty / l = 4F4
```

### 1.2. BodyRateControl
The commanded and actual roll, pitch and yaw values were compared, and the error was calculated for each of them. After multiplying with corresponding moment of inertia (Ixx, Iyy, Izz), the moment commands were generated in the body frame.

### 1.3. RollPitchControl
This function compares the target and the current tilt angles. Then using a P controller it calculates the desired pitch and roll angle rates in the body frame.

### 1.4. Parameters
The tested parameters and their resulting t_set times for qual.roll and qual.omega.x are given in the below table. After implementing the other functions, it was required to change these parameters. 

| kp_P | kp_Q | kp_R | kpBank | t_set quad.roll | t_set quad.omega.x |
|:----:|:----:|:----:|:------:|:---------------:|:------------------:|
|  28  |  28  |  15  |     9  |  0.210          | 0.190              |
|  30  |  30  |  15  |    10  |  0.185          | 0.185              |
|  37  |  37  |  15  |    11  |  0.170          | 0.170              |
|  44  |  44  |  15  |    12  |  0.160          | 0.165              |
|  49  |  49  |  15  |    13  |  0.190          | 0.165              |
|  55  |  55  |  15  |    14  |  0.190          | 0.160              |
|  58  |  58  |  15  |    15  |  0.195          | 0.160              |
| 120  | 120  |  15  |    20  |  0.200          | 0.205              |

## 2. Task 2 Position/Velocity and Yaw Angle Control (Scenario 3)
### 2.1. LateralPositionControl

The lateral controller us a PD controller to command the target b<sup>x</sup><sub>c</sub> and b<sup>y</sup><sub>c</sub> values. These values correspond to the R<sub>13</sub> and R<sub>23</sub> elements of the rotation matrix.

The PD controller compares the target and current position and velocity vectors of x and y, then generates the horizontal acceleration vector.


### 2.2. AltitudeControl

This function implements 2 P controllers, the first one calculates the desired vertical velocity using the position error on z. The P controller calculates the desired vertical acceleration using the desired velocity calculated by the first controller, and the actual velocity. Then the gravity was subtracted, and rotated back to the world frame. The resulting value multiplied with the mass and the required thrust calculated.


### 2.3. YawControl

Yaw controller is a P controller to calculate the desired yaw rate in rad/s.


### 2.4. Parameters
| kpPosXY | kpPosZ | kpVelXY | kpVelZ  | kpYaw | kp_R | t_set quad.pos.x |
|:-------:|:------:|:-------:|:-------:|:-----:|:----:|:----------------:|
|   40    |    4   |   12    |    8    |    4  |  15  |   0.540          |
|   45    |    4   |   12    |    8    |    4  |  15  |   0.465          |
|   51    |    4   |   12    |    8    |    4  |  15  |   0.435          |
|   60    |    4   |   12    |    8    |    4  |  15  |   0.760          |
|   20    |    4   |   12    |    8    |    4  |  15  |   1.290          |
|   30    |    4   |   12    |    8    |    4  |  15  |   0.855          |
|   30    |    4   |   15    |    8    |    4  |  15  |   1.110          |
|   40    |    4   |   10    |    8    |    4  |  15  |   0.450          |
|   42    |    4   |   10    |    8    |    4  |  15  |   0.440          |

## 3. Non-idealities and robustness (scenario 4)

The first P controller in the AltitudeController function, which calculates the desired velocity, converted to PI controller it adds i_term using the integrated altitude error.

## 4. Tracking trajectories (scenario 5)

In this step the parameters were tuned to pass the scenarios from 2 to 5. The resulting parameters are as follows:

### Position control gains  
kpPosXY = 30  
kpPosZ = 4  
KiPosZ = 4  

### Velocity control gains  
kpVelXY = 12  
kpVelZ = 5  

### Angle control gains  
kpBank = 13  
kpYaw = 4  

### Angle rate gains  
kpPQR = 44, 44, 15



This implementation is based on the Lesson 4 - 3D Drone-Full- notebook in the lesson, and the python solution implementation.
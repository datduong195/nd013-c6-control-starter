# Control and Trajectory Tracking for Autonomous Vehicle

# Proportional-Integral-Derivative (PID)

In this project, you will apply the skills you have acquired in this course to design a PID controller to perform vehicle trajectory tracking. Given a trajectory as an array of locations, and a simulation environment, you will design and code a PID controller and test its efficiency on the CARLA simulator used in the industry.

### Installation

Run the following commands to install the starter code in the Udacity Workspace:

Clone the <a href="https://github.com/udacity/nd013-c6-control-starter/tree/master" target="_blank">repository</a>:

`git clone https://github.com/udacity/nd013-c6-control-starter.git`

## Run Carla Simulator

Open new window

* `su - student`
// Will say permission denied, ignore and continue
* `cd /opt/carla-simulator/`
* `SDL_VIDEODRIVER=offscreen ./CarlaUE4.sh -opengl`

## Compile and Run the Controller

Open new window

* `cd nd013-c6-control-starter/project`
* `./install-ubuntu.sh`
* `cd pid_controller/`
* `rm -rf rpclib`
* `git clone https://github.com/rpclib/rpclib.git`
* `cmake .`
* `make` (This last command compiles your c++ code, run it after every change in your code)

## Testing

To test your installation run the following commands.

* `cd nd013-c6-control-starter/project`
* `./run_main_pid.sh`
This will silently fail `ctrl + C` to stop
* `./run_main_pid.sh` (again)
Go to desktop mode to see CARLA

If error bind is already in use, or address already being used

* `ps -aux | grep carla`
* `kill id`


## Project Instructions

In the previous project you built a path planner for the autonomous vehicle. Now you will build the steer and throttle controller so that the car follows the trajectory.

You will design and run the a PID controller as described in the previous course.

In the directory [/pid_controller](https://github.com/udacity/nd013-c6-control-starter/tree/mathilde/project_c6/project/pid_controller)  you will find the files [pid.cpp](https://github.com/udacity/nd013-c6-control-starter/tree/mathilde/project_c6/project/pid_controller/pid.cpp)  and [pid.h](https://github.com/udacity/nd013-c6-control-starter/tree/mathilde/project_c6/project/pid_controller/pid.h). This is where you will code your pid controller.
The function pid is called in [main.cpp](https://github.com/udacity/nd013-c6-control-starter/tree/mathilde/project_c6/project/pid_controller/main.cpp).

### Step 1: Build the PID controller object
Complete the TODO in the [pid_controller.h](https://github.com/udacity/nd013-c6-control-starter/tree/mathilde/project_c6/project/pid_controller/pid_controller.h) and [pid_controller.cpp](https://github.com/udacity/nd013-c6-control-starter/tree/mathilde/project_c6/project/pid_controller/pid_controller.cpp).

Run the simulator and see in the desktop mode the car in the CARLA simulator. Take a screenshot and add it to your report. The car should not move in the simulation.
### Step 2: PID controller for throttle:
1) In [main.cpp](https://github.com/udacity/nd013-c6-control-starter/tree/mathilde/project_c6/project/pid_controller/main.cpp), complete the TODO (step 2) to compute the error for the throttle pid. The error is the speed difference between the actual speed and the desired speed.

Useful variables:
- The last point of **v_points** vector contains the velocity computed by the path planner.
- **velocity** contains the actual velocity.
- The output of the controller should be inside [-1, 1].

2) Comment your code to explain why did you computed the error this way.

3) Tune the parameters of the pid until you get satisfying results (a perfect trajectory is not expected).

### Step 3: PID controller for steer:
1) In [main.cpp](https://github.com/udacity/nd013-c6-control-starter/tree/mathilde/project_c6/project/pid_controller/main.cpp), complete the TODO (step 3) to compute the error for the steer pid. The error is the angle difference between the actual steer and the desired steer to reach the planned position.

Useful variables:
- The variable **y_points** and **x_point** gives the desired trajectory planned by the path_planner.
- **yaw** gives the actual rotational angle of the car.
- The output of the controller should be inside [-1.2, 1.2].
- If needed, the position of the car is stored in the variables **x_position**, **y_position** and **z_position**

2) Comment your code to explain why did you computed the error this way.

3) Tune the parameters of the pid until you get satisfying results (a perfect trajectory is not expected).

### Step 4: Evaluate the PID efficiency
The values of the error and the pid command are saved in thottle_data.txt and steer_data.txt.
Plot the saved values using the command (in nd013-c6-control-refresh/project):

```
python3 plot_pid.py
```

You might need to install a few additional python modules: 

```
pip3 install pandas
pip3 install matplotlib
```

Answer the following questions:
- Add the plots to your report and explain them (describe what you see)
- What is the effect of the PID according to the plots, how each part of the PID affects the control command?
- How would you design a way to automatically tune the PID parameters?
- PID controller is a model free controller, i.e. it does not use a model of the car. Could you explain the pros and cons of this type of controller?
- (Optional) What would you do to improve the PID controller?


### Tips:

- When you wil be testing your c++ code, restart the Carla simulator to remove the former car from the simulation.
- If the simulation freezes on the desktop mode but is still running on the terminal, close the desktop and restart it.
- When you will be tuning the PID parameters, try between those values:

# Results

<img width="960" alt="step1" src="https://user-images.githubusercontent.com/36104217/180228468-443c68eb-221f-4eea-9e03-ae0a1a382649.png">

## Testing results for mutiple PID Controller

|  TEST  | KP_STEER | KI_STEER | KD_STEER | KP_THROTTLE | KI_THROTTLE | KD_THROTTLE |
|:------:|:--------:|:--------:|:--------:|:-----------:|:-----------:|:-----------:|
| Test 1 |   0.5    |   0.01   |   0.5    |    0.5      |    0.001    |     0.5     |
| Test 2 |   0.4    |   0.001  |   0.35   |    0.2      |    0.001    |     0.02    |
| Test 3 |   0.4    |   0.001  |   0.35   |    0.2      |    0.0015   |     0.04    |

### Case 1:

<img width="402" alt="data_0" src="https://user-images.githubusercontent.com/36104217/180228832-f0176921-12c1-4fb3-9f7f-38b7f8a9895c.png">
<img width="477" alt="step2_steering" src="https://user-images.githubusercontent.com/36104217/180228819-23a7a5da-eb6b-4fdc-81e9-046b3b833f68.png">
<img width="479" alt="step2_throttle" src="https://user-images.githubusercontent.com/36104217/180228828-c7061299-1987-4133-9f4a-0746b0fac243.png">

Steering is having acceptable error state, but overshoot could be reduce by slightly decreasing KP.
Throttle is having huge overshoot and steady state error . KP and KD should be decresed significantly. 
Vehicle is constantly crashing to obstacles.

### Case 2:

<img width="402" alt="data_1" src="https://user-images.githubusercontent.com/36104217/180229775-a89e1509-0773-4877-aeb1-1a37fe9596ee.png">
<img width="481" alt="step2_steering_1" src="https://user-images.githubusercontent.com/36104217/180229768-c9b98456-5353-41e5-b593-883837f9edb2.png">
<img width="481" alt="step2_throttle_1" src="https://user-images.githubusercontent.com/36104217/180229773-7a5f6d5a-74bf-4b89-b39a-a8d8664bf842.png">

Steering is having better error state and overshoot.
Throttle is having less overshoot but steady state error is still significant.
Vehicle can avoid some obstacles, but the steering and throttle jerking is still there, affecting driver experience.
### Case 3:

<img width="404" alt="data_2" src="https://user-images.githubusercontent.com/36104217/180229923-a24da3f9-e3a6-4df3-a7fc-366d80159db1.png">
<img width="485" alt="step2_steering_2" src="https://user-images.githubusercontent.com/36104217/180229925-69cfff4d-8577-4b7b-8615-fcc8abe2c979.png">
<img width="482" alt="step2_throttle_2" src="https://user-images.githubusercontent.com/36104217/180229918-73264b26-7fc1-4e6a-aa8d-be67a95d2c8a.png">

Steering is having better error state and overshoot.
Throttle is having less overshoot. Steady State Error is decreasing in later iterations, because it's a trade off for small KP and KD to avoid overshoot.
Vehicle can avoid all obstacles in simulation, but lane keeping is just acceptable and drive at very slow speed.

# Key questions for this project
## What is the effect of the PID according to the plots, how each part of the PID affects the control command?
+ Proportional term: Produces an output value that is proportional to the current error value. A high proportional gain results in a large change in the output for a given change in the error.
+ Integral term: Proportional to both the magnitude of the error and the duration of the error. The integral term accelerates the movement of the process towards setpoint and eliminates the residual steady-state error that occurs with a pure proportional controller.
+ Derivative term: calculated by determining the slope of the error over time and multiplying this rate of change by the derivative gain. Derivative action predicts system behavior and thus improves settling time and stability of the system.

## How would you design a way to automatically tune the PID parameters?
Twiddle algorithm: The twiddle algorithm continuously tunes the PID controller's hyperparameters by analyzing the cross-track error and keeping track of the smallest CTE.

## PID controller is a model free controller, i.e. it does not use a model of the car. Could you explain the pros and cons of this type of controller?

Pros:
+ Pid is easy to implement and feasible for many applications. It also needs to tune few parameters, via trial and error or cross-validation
Cons:
+ With PID it's difficult to handle multiple constraints

## What would you do to improve the PID controller?
Applying Twiddle algorithm might help in tuning PID parameters.

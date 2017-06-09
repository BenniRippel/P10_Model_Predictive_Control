# Model Predictive Control
Self-Driving Car Engineer Nanodegree Program


## Project Introduction

Model Predictive Control is a method for predictive control of a complex process. An optimizer is used to find the control 
inputs minimizing a well defined cost function. I this case, it is used to find the control 
inputs (steering and throttle/brake) to follow waypoints along a track in a simulator.
 
In general, the algorithm consists of the following steps:

Setup:

- define horizon (length of trajectory N and duration of timestep dt)
- define constraints (actuator and vehicle dynamics)
- define a cost function 

Apply at each time step (loop):

- pass current state to the controller
- solve optimization problem using a solver that minimizes the cost function and returns a vector of N-1 control inputs
- pass the control inputs for the next timestep to the according actuators (steering/throttle)
    

## Used Model

A kinematic model which ignores tire forces, gravity and mass is used. It is accurate enough for low 
longitudinal and lateral accelerations. 

The following vehicle states are represented:

states = [x, y, psi, v, cte, epsi]

with:

|state|description|
|---|---|
|x | position in x-direction|
|y | position in y-direction|
|psi | orientation|
|v | velocity|
|cte| cross track error|
|epsi| orientation error|
 

The following actuators are used to control the vehicle

actuators = [delta, a] 

with: 


|actuator|description|limits|
|---|---|---|
|delta | steering angle| [-25°, 25°] |
|a | throttle/brake | [-1, 1]|

The unitless values of a are not directly linked to the cars acceleration. Ignoring the vehicle dynamics (engine dynamics, mass etc.), a can be seen as
 the desired acceleration in g.

The folloing equations describe the kinematic model:

x_t+1 = x_t + v_t*cos(psi_t)*dt

y_t+1 = y_t + v_t*sin(psi_t)*dt

psi_t+1 = psi_t + v_t/L_f * delta_t * dt

v_t+1 = v_t + a_t*dt

cte_t+1 = y_desired_t - y_t + (v_t * sin(epsi_t)*dt)

epsi_t+1 = psi_t - psi_desired_t + (v_t/L_f * delta_t * dt)


where: 

- L_​f​​  measures the distance between the front axle of the vehicle and its center of gravity
- y_desired_t is the current desired lateral position on the track
- psi_desired_t is the current desired orientation


## Horizon Parameters

When applying MPC, the horizon parameters have to be set. They include the length of trajectory N and duration of timestep dt.
To minimize the discretization error, dt should be small, especially when driving with high speeds. The length of the trajectory 
should be adjusted for a maximum prediction horizon of a few seconds at most. Additionally, the computational load has to be taken into
 account, which increases with N.

The following parameters where chosen, which lead to a stable solution for a velocity of 75 mph.
  
|parameter|value|
|---|---|
|N|15|
|dt|0.1s|

If N is increased to 20 or higher, while dt stays at 0.1s, the predicted trajectory exceeds the last waypoints. Consequently, 
the fitted 3rd order polynomial is extrapolated and hence may lead to unstable solutions of the MPC solver. Decreasing N to 10 still works fine, 
while decreasing to even lower values leads to unstable behaviour with nonsense actuator inputs.
 
Increasing the time step dt to 0.2s with N at 15 leads to a prediction horizon of 3s, which is to large for a speed of 75 
mph and higher. Consequently, the solution is not stable. A dt of 0.05s leads to a very short predicted trajectory and 
consequently to a oscillating car.
 
To put it in a nutshell, a dt of 0.1s with N within the interval [10, 15], leading to a prediction horizon of 1s or 1,5s works best. 


## Polynomial Fitting and MPC Preprocessing

For easy calculation of the cross-track-error and the orientation error, the waypoints are transformed to the cars 
coordinate system at the cars position after the latency has passed. Afterwards, a 3rd order polynomial is fitted to these waypoints.
To calculate the cte, the resulting polynomial is evaluated at the current position of the car (x=0). The orientation error
is calculated by 

epsi=-atan(f'(x=0))=-atan(coeffs[1])

The current states x,y and psi are 0 due to the coordinate system. The value of v is calculated as follows, using the 
throttle value as a scaling factor of the current longitudinal acceleration:

v_pred = v + throttle * 9.81 * latency;


## Model Predictive Control with Latency
To deal with actuation latency, the cars state in global coordinates is predicted for t+=latency 
using the kinematic model. This new predicted state is used as the origin of the car coordinate system, which in 
turn is used as reference to fit the waypoints and calculate the cross-track-error and orientation error.
Hence, the actuator latency is taken into account when solving for the optimal actuator inputs.

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.



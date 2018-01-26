# Self-Driving Car Engineer Nanodegree Program
*Model Predictive Control Project*

## Introduction
The goal of this project was to have a virtual vehicle navigate a simulated track in Udacity's simulator. The simulator provided telemetry and track waypoint data using `uWS` (websocket) and the vehicle's code sent back steering and acceleration commands. The additional challenge of this project was that 100-ms latency had to be included in order to simulate real-world conditions. 

As the dependencies section alludes to, this project made us of the Ipopt and CppAD libraries to calculate the trajectory and actuation commands. The error was minimized using a third order polynomial fit to the given track waypoints.

## Dependencies
The dependencies for this project are:
* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
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

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.

I had a tremendously difficult time getting Ipopt installed since homebrew/science has been deprecated. After trying many of the solutions set forth in the Forums with very little success, I decided to try using [Docker](https://discussions.udacity.com/t/getting-started-with-docker-and-windows-for-the-ekf-project-a-guide/320236) rather than my local machine. Two minutes later and my code was compiled and the simulator was running. I have no idea why that is not the first suggested method for completing this project due to the major problems people are having with Ipopt.

## Build Instructions
1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Compilation
The project compiles without errors but includes a few warnings that don't affect the results.

## Implementation
A kinematic model was used that neglected the interactions between the tires and the road. The model equations are as follow:

```
x[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
y[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
psi[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
v[t] = v[t-1] + a[t-1] * dt
cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt
```

The variables that describe the state of the model are:

- `x, y` : position
- `psi` : heading direction
- `v` : velocity
- `cte` : cross-track error
- `epsi` : crientation error

`Lf` is the distance between the car's center of gravity and the front wheels.

The output of the model is:

- `a` : acceleration (throttle)
- `delta` : steering angle

The goal of this project is to find the acceleration (`a`) and the steering angle(`delta`) while minimizing an objective function.

The objective function is made up of:

- The squared sum of `cte` and `epsi`.
- The squared sum of the actuator values to reduce the use of the actuators.
- The squared sum of the difference between two sequential actuator values in order reduce sharp changes in direction.

Each of these factors were given a weight, through trial and error, that produced a relatively smooth and successful completion of the simulator track. 

### Timestep Length and Elapsed Duration
I chose 10 and 0.1 for `N` and `dt`, respectively. Using these values, a 1-second duration is used to determine the corrective trajectory. This was based off of a suggestion during Udacity's office hours. These values produce the smoothest ride around the track that I tested. I modified `N` in increments of 2 and `dt` in increments of 0.025. Changes to either caused significant changes to the behavior of the vehicle.

### Polynomial Fitting and MPC Preprocessing
All of the waypoints were transformed to the vehicles coordinate system in `main.cpp`. This means that the vehicles coordinates are located at the origin (0,0) and the heading is 0 as well.

### Model Predictive Control with Latency
Two approaches were taken:

1. The velocity and delta were included in the objective function in addition to the what's described above. This additional cost resulted in better navigation of the corners on the course.
2. The kinematic equations were modified to account for the delay of 100-ms.
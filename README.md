# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---
## The Model.

In this project, we used a Kinematic model. Kinematic models are simplifications of dynamic models that ignore tire forces, gravity, and mass. This simplification reduces the accuracy of the models, but it also makes them more tractable. At low and moderate speeds, kinematic models often approximate the actual vehicle dynamics. The model uses the state and actuations from the previous timestep to calculate the state for the current timestep. The model equations are as below:

	x[t+1] = x[t] + v[t] * cos(psi[t]) * dt

	y[t+1] = y[t] + v[t] * sin(psi[t]) * dt

	psi[t+1] = psi[t] + v[t] / Lf * delta[t] * dt

	v[t+1] = v[t] + a[t] * dt

	cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt

	epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt

where:
x,y is the position of the car,
psi is the car's orientation or heading,
v is the velocity of the vehicle,
cte is the cross track error; the error between the center of the road and the vehicle’s position,
epsi is the orientation error; the desired orientation subtracted from the current orientation,
delta is the steering angle,
a is the acceleration,
Lf​  is the distance between the center of mass of the vehicle and it's front axle,
psides is the desired orientation.

In this model, [x,y,psi,v,cte,epsi] is the state of the vehicle,  Lf​  is a physical characteristic of the vehicle, and  [delta,a]  are the actuators, or control inputs, to our system.

This is a good kinematic model!

## Timestep Length and Elapsed Duration (N & dt).

The prediction horizon (_T_) is the duration over which future predictions are made and  is the product of two other variables,  _N_  and  _dt_. _N_  is the number of timesteps in the horizon.  _dt_  is how much time elapses between actuations. For example, if  _N_  were 20 and  _dt_  were 0.5, then  _T_  would be 10 seconds.

In the case of driving a car,  _T_  should be a few seconds, at most. Beyond that horizon, the environment will change enough that it won't make sense to predict any further into the future. N also determines the length of the control input vector which needs to be optimized by the MPC controller. This is also the major driver of computational cost. MPC attempts to approximate a continuous reference trajectory by means of discrete paths between actuations. Larger values of  _dt_  result in less frequent actuations, which makes it harder to accurately approximate a continuous reference trajectory. This is sometimes called "discretization error".

A good approach to setting  _N_,  _dt_, and  _T_  is to first determine a reasonable range for  _T_  and then tune  _dt_and  _N_  appropriately, keeping the effect of each in mind.

By keeping the above guidelines in mind, several values of _N_ (in the range of 5 to 25) and _dt_ (in the range of 0.1 to 0.5) were tried and settled for the final valuesof _N_ = 10 and _dt_ = 0.1
    

## Polynomial Fitting and MPC Preprocessing.

In MPC, the reference trajectory is typically passed to the control block as a polynomial. This polynomial is usually 3rd order, since third order polynomials will fit trajectories for most roads. We used  `polyfit`  to fit a 3rd order polynomial to the given x and y coordinates representing waypoints and  `polyeval`  to evaluate y values of given x coordinates.

We displayed both these reference path and the MPC trajectory path in the simulator by sending a list of optional x and y values to the  `mpc_x`,`mpc_y`,  `next_x`, and  `next_y`  fields in the C++ main script.

These (x,y) points are displayed in reference to the vehicle's coordinate system. The x axis always points in the direction of the car’s heading and the y axis points to the left of the car and the server returns waypoints using the map's coordinate system, which is different than the car's coordinate system. So we preprocessed these waypoints by transforming them into vehicles coordinate system to both display them and to calculate the CTE and Epsi values for the model predictive controller.

## Model Predictive Control with Latency
In a real car, an actuation command won't execute instantly - there will be a delay as the command propagates through the system. A realistic delay might be on the order of 100 milliseconds. This is a problem called "latency", and it's a difficult challenge for some controllers - like a PID controller - to overcome. But a Model Predictive Controller can adapt quite well because we can model this latency in the system.

A contributing factor to latency is actuator dynamics. For example the time elapsed between when you command a steering angle to when that angle is actually achieved. This could easily be modeled by a simple dynamic system and incorporated into the vehicle model. Thus, MPC can deal with latency much more effectively, by explicitly taking it into account, than a PID controller.

Here in this model the delay of 100ms is countered by considering the actuator outputs [delta, a] from the previous time step [t-1] as the delay here is equal to the time step length _dt_ (0.1 s or 100 ms) of the model.

## Dependencies

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


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Build with Docker-Compose
The docker-compose can run the project into a container
and exposes the port required by the simulator to run.

1. Clone this repo.
2. Build image: `docker-compose build`
3. Run Container: `docker-compose up`
4. On code changes repeat steps 2 and 3.

## Tips

1. The MPC is recommended to be tested on examples to see if implementation behaves as desired. One possible example
is the vehicle offset of a straight line (reference). If the MPC implementation is correct, it tracks the reference line after some timesteps(not too many).
2. The `lake_track_waypoints.csv` file has waypoints of the lake track. This could fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Editor Settings

We have kept editor configuration files out of this repo to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. We omitted IDE profiles to ensure
students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. Most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio and develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

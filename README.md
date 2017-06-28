# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---
## Overview
The purpose of this project is to implement a model predictive controller (MPC) to drive a vehicle along a desired path (reference trajectory). It was tested in a simulator provided by Udacity. The simulator outputs x and y positions, speed, and orientation of the vehicle along with the reference trajectory.

In the simulator, the reference trajectory is show as a yellow line and the predicted path is in green. Below are images of the vehicle driving 51 mph around a curve and 81 mph on a straighter path with speed limit set to 100 mph and 98 mph around the same curve with speed limit set to 200 mph.

![51](https://github.com/CassLamendola/Model-Predictive-Control/blob/master/Screen%20Shot%202017-06-28%20at%209.50.36%20AM.png)
![81](https://github.com/CassLamendola/Model-Predictive-Control/blob/master/Screen%20Shot%202017-06-28%20at%209.51.09%20AM.png)
![98](https://github.com/CassLamendola/Model-Predictive-Control/blob/master/Screen%20Shot%202017-06-28%20at%2011.51.31%20AM.png)

## The Model
The model used is a kinematic bicycle model. The model state includes:
* `x` position
* `y` position
* orientation `psi`
* velocity `v`
* cross-track error `cte`
* orientation error `epsi`

The control inputs are:
* steering angle `delta`
* acceleration `a`

The following equations are used:
```
// values at timestep [t+1] based on values at timestep [t] after dt seconds 
// Lf is the distance between the front of the vehicle and the center of gravity

x[t+1] = x[t] + v[t] * cos(psi[t]) * dt;
y[t+1] = y[t] + v[t] * sin(psi[t]) * dt;
psi[t+1] = psi[t] + v[t]/Lf * delta[t] * dt;
v[t+1] = v[t] + a[t] * dt;
cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt;
epsi[t+1] = psi[t] - psi_des + v[t]/Lf * delta[t] * dt;
```

## Timestep Length and Elapsed Duration
The prediction horizon (T) is the product of the timestep length (N) and elapsed duration (dt). Timestep length refers to the number of timesteps in the horizon and elapsed duration is how much time elapses between each actuation.

The prediction horizon I settled on was one second. I set [N = 10 and dt = .1](https://github.com/CassLamendola/Model-Predictive-Control/blob/master/src/MPC.cpp#L8-L10)

I tuned these values until achieving a model that could complete the track at 20 mph as well as 200 mph. At high speeds, these hyperparameters had a greater effect on the performance of the controller. I first tried using a larger N (25) and smaller dt (.25). With these values, if the vehicle "overshot" the reference trajectory, it would begin to oscillate wildly and drive off the track. Lower N (5) resulted in the vehicle driving straight off the track.

I tried N = 20, dt = .1; N = 20, dt = .05; N = 15, dt = .1; N = 15, dt = .05; and N = 5, dt = .1;

I tried N = 10 and dt = .1 after watching this [video](https://www.youtube.com/watch?v=bOQuhpz3YfU&feature=youtu.be&utm_medium=email&utm_campaign=2017-06-05_carnd_term2_annoucements&utm_source=blueshift&utm_content=2017-06-01_carnd_announcements&bsft_eid=5d3d51b3-1acc-41d3-9f6d-cadc1f93a952&bsft_clkid=de8ad7f4-5532-4946-ba57-ca236702203f&bsft_uid=be460fc4-ea43-4a59-9282-51d5a6bd9981&bsft_mid=a085a1a1-2b8f-46e1-b2d3-d474284c60a2) about the project and I achieved the best result with these values.

## Polynomial Fitting and MPC Preprocessing
First, [the points are transformed](https://github.com/CassLamendola/Model-Predictive-Control/blob/master/src/main.cpp#L110-L113) into the vehicle's coordinate system, making the first point the origin. This is done by subtracting each point from the current position of the vehicle. 

Next, [the orientation is also transformed](https://github.com/CassLamendola/Model-Predictive-Control/blob/master/src/main.cpp#L115-L117) to 0 so that the heading is straight forward. Each point is rotated by psi degrees.

Then the vector of points is converted to an [Eigen vector](https://github.com/CassLamendola/Model-Predictive-Control/blob/master/src/main.cpp#L107-L111) so that it is ready to be an argument in the [polyfit](https://github.com/CassLamendola/Model-Predictive-Control/blob/master/src/main.cpp#L47-L72) function where the points are fitted to a 3rd order polynomial. That polynomial is then evaluated using the [polyeval](https://github.com/CassLamendola/Model-Predictive-Control/blob/master/src/main.cpp#L36-L42) function to calculate the cross-track error.

## Dealing with Latency
After getting a working MPC, a delay of 100 ms had to be dealt with. When this latency was first introduced, the model oscillated about the reference trajectory and, at high speeds, drove off the track.

To account for the latency, I used the [equations](https://github.com/CassLamendola/Model-Predictive-Control/blob/master/src/main.cpp#L100-L107) mentioned before to set the initial state to be the state after 100 ms. This allows the vehicle to "look ahead" and correct for where it will be in the future instead of where it is currently positioned.

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
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
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
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

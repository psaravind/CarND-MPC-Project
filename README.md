# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---
## Implementation

### The Model
MPC can be summarized as follows from [Robus Model Predictive Control Design](https://www.intechopen.com/books/model-predictive-control/robust-model-predictive-control-design).
* Predict the future behavior of the process state/output over the finite time horizon.
* Compute the future input signals on line at each step by minimizing a cost function under inequality constraints on the manipulated (control) and / or controlled variables.
* Apply on the vehicle only the first of vector control variable and repeat the previous step with new measured input/state/output variables.

MPC uses the measurable **state** of the vehicle, **actuators** that change the state of the vehicle and set of **equations** that are used to compute the new state from previous state values.

**State**: To keep track of the state of the vehicle, following parameters are used
* x, y - position of the car in map coordinates which need to be converted to vehicle coordinate system.
* ψ - orientation of the car
* v - velocity of the car
* cte - cross track error or distance between the center of the road and the vehicle's poisition
* epsi - difference of vehicle orientation and trajectory orientation

The simulator provides x, y, ψ and v for the car.

**Actuators**:  These parameters State: [x,y,ψ,v] and Actuators: [δ,a] are used to change the vehicle state over time.  Kinematic model equations are applied on previous state to obtain new state and as defned as follows:
* x = x + v * cos(ψ) * dt 
* y = y + v * sin(ψ) * dt 
* v = v + a * dt 
* ψ = ψ + v/Lf * δ * dt
dt is the change is time and Lf measures the distance between the front of the vehicle and its center of gravity.

**Cost function**: MPC.cpp(line 55:71) is the key cost function in solving MPC equation.  The equation requires lot of tuning, but follows the parameters shown in  [Self-Driving Car Project Q&A MPC Controller video](https://youtu.be/bOQuhpz3YfU?list=PLAwxTw4SYaPnfR7TzRZN-uxlxGbqxhtm2) and the solution to the class quiz.

The cost function includes several coefficients that have a lot of power on how much attention we want the cost function to pay to certain attributes.  We have high values for reference state to cross track error(cte) and vehicle orientation error(epsi), this forces the cost function to be very low.  More attention is given to cte and epsi, so that the car is following the track and orientation than going fast, this results in the car not going off the road and not getting misalligned. These parameters in the cost functions smooth out the driving in the simulator. Code calculates the deviation from reference cte = 0, reference epsi = 0 and reference velocity = 60, this calculation is done for N timesteps into the future.

		for (int t = 0; t < N; t++) {
			fg[0] += 2000 * CppAD::pow(vars[cte_start + t], 2);
			fg[0] += 2000 * CppAD::pow(vars[epsi_start + t], 2);
			fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
		}

Minimize the use of actuators in the cost function.

		for (int t = 0; t < N - 1; t++) {
			fg[0] += 5 * CppAD::pow(vars[delta_start + t], 2);
			fg[0] += 5 * CppAD::pow(vars[a_start + t], 2);
		}

Minimize the value gap between sequential actuations so that car is not jerky and not compromise the car.

		for (int t = 0; t < N - 2; t++) {
			fg[0] += 200 * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
			fg[0] += 10 * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
		}
  
**MPC Constraints** MPC.cpp(line 91:133) shows the setup for rest of the constraints.  Setup uses CppAD library that helps us do all the gradients calculation in linear algebra.

**MPC Steps**  Following are the steps for setting up MPC constrainsts and cost function.

1. Define N - length of tragectory, dt - duration of each timestep.
2. Fit polynomial to way points and use it to set initial cross track error and orientation error.
3. Define vehicle dynamics and actuator limitations along with other constraints.
4. Define cost function.

After setting up the model constrainsts, following are the steps for executing MPC:
1. Pass current state as the initial state to MPC
2. Call optimization solver with the initial state, the solver will return the vector of control inputs that minimizes the cost function.   Solver uses Ipopt library which is a library for large scale nonlinear optimization of continuous systems.

### Timestep Length and Elapsed Duration (N & dt)
Timestep length(N) and Frequency(dt) are key parameters that needed to be tuned for the MPC model.

Timestep N is the number of timesteps in the horizon, as it increases, the model predicts futher ahead.  Also as N increase the prediction becomes more inaccurate and takes more time to solve the equation in real time. Timestep values of 10 to 20 was experimented and value of 12 was selected.

dt is how much time elapses between acutations, for low values of dt the cars was oscillating a lot in the middle of the road, this is due to frequent responses from the actuator and the vehicle responding constantly.  For large value of dt, the car had jerky motion on the track, making sharp turns at the track edges, this resulted in car leaving the track few times.

### Polynomial Fitting and MPC Preprocessing

Polyeval and Polyfit functions in main.cpp(line 36:66) is used extensively to address the two objectives: Speed and polynomial track line to follow.  To optimize the speed we have a cost function that measures if the speed of the car is really close to the desired speed and to optimize the track to be followed by the car, we use a polynomial line through 6 waypoints, the car is doing well if the car is close to that line.

Since the x and y coordinates sent by the simulator is in map coordinates, these were converted to car cordinates and a 3rd degree polinomial was fit to calculate CTE.

### Model Predictive Control with Latency
Latency defines the delay in propagating commands through the system to the car controls, since the simulator has been setup with a latency of 100ms, the model is implemented with 100ms latency as well.  Removing this latecy, the model was able to achieve a maximum speed of 100mph, with the latecy the maximum speed had to be reduced to 60mph for smooth steering and throttle.

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

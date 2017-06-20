# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Project Description

#### Recording of Simulator Run
[YouTube Link](https://youtu.be/Yzkd5BQ8WRA).

#### Walkthrough of project

- Model predictive control (MPC) is an advanced method of process control. It has the ability to anticipate future events and can take control actions accordingly. This is an advantage of MPC over PID controllers.

- MPC is based on iterative, finite-horizon optimization of a plant model. At time t the current plant state is sampled and a cost minimizing control strategy is computed for a relatively short time horizon in the future. Only the first step of the control strategy is implemented, then the plant state is sampled again and the calculations are repeated starting from the new current state, yielding a new control and new predicted state path. The prediction horizon keeps being shifted forward.

- Inital state vector has 6 variables; 
  - `px`, the current location in the x-axis of an arbitrary global map coordinate system (given by simulator)
  - `py`, the current location in the y-axis of an arbitrary global map coordinate system (given by simulator)
  - `psi`, the current orientation (given by simulator)
  - `v`, the current velocity (given by simulator)
  - `cte`, the cross track error which is the difference between our desired position and actual position (calculated through polynomial fitting at point `px = 0`)
  - `epsi`, the orientation error which is the difference between our desired heading and actual heading (calculated using an arctan to the derivative of the fitted polynomial function at point `px = 0`)

- Actuators are represented through 2 variables;
  - `delta`, the steering value. The angle is restricted to be between -25 and 25 degrees, or `-0.4 * Lf` and `0.4 * Lf` radians.
  - `a`, the throttle value. It is restricted to be between -1 and 1. Negative values represent decceleration/braking while positive values represent acceleration.

- The main goal for us is to keep the car coordinates close to the waypoints given by an arbitrary global map coordinate system.

- `Main.cpp`:
  - First, waypoints were transformed to the vehicle's local coordinate system (lines 104 - 111). This helps with the polynomial fitting as the car is nearly following a horizontal line.
  - Waypoints were converted from a doubles vector to a VectorXd (lines 113-117).
  - Polyfit applied using 3rd order polynomial function. This estimates the road ahead. A smaller order polynomial may cause underfitting, and a higher order polynomial may cause overfitting (line 119).
  - `cte` and `epsi` were calculated from the coefficients given by the polyfit function (lines 121-122).
  - Applied a latency of `100ms`. A contributing factor to latency is actuator dynamics. For example the time elapsed between when you command a steering angle to when that angle is actually achieved. We initiate the vehicle model starting from the current state for the duration of the latency. The resulting state from the simulation is the new initial state for MPC (lines 130-138).
  - State vector is fed the new values for variables taking latency into account (line 140).
  - The future state of the vehicle is predicted using the MPC model (line 142).
  - We draw the refernce path for the car (the yellow line) (lines 145-154).
  - We draw the predicted path for the car (the green line) (lines 157-166).

- `MPC.cpp`:
  - Timestep length `N` is set to 10. It is a moderate value, smaller values will cause the car to go off the track while bigger values will be computationally expensive (lines 9-10).
  - `ref_cte` and `ref_epsi` are set to `0` as we want the car to align to the refernce path. `ref_v` is set to `100` which is somewhat fast (lines 24-26).
  - Car state variables initialized (lines 28-35).
  - We start calculating fg[0] which is the cost function that gets updated.
  - Cost weights were chosen through trial and error. `2000` was given as a weight for `cte` and `epsi` to keep them low. For `v` we give low attention so the no weight were given. For the steering `delta`, the weight given was moderate as we want the steering to be smooth. Acceleration `a` was given a low weight of `10` (lines 49-65).
  - We initialize the model to the initial state. `fg[0]` is reserved for the cost value, so the other indices are bumped up by 1 (lines 72-77).
  - All the other constraints based on the vehicle model. `fg[1 + psi_start]` is where we store the initial value of ψ. So, `fg[1 + psi_start + t]` is reserved for the values of ψ that the solver computes. We get the values for both current time `t` and time `t+1` (lines 82-95).
  - For steering and accelerattion, we only consider the actuations at time `t` (lines 98-99).
  - Cost function calculation using polynomial (lines 101-102).
  - The solver will force this value of fg to always be 0 (lines 104-110).
  - We set the upper and lower bounds for the actuators. Values were chosen through trial and error. The chosen values worked well in the simulator with smooth steering and good speeding/braking. A special case has been given to the initial state so the simulator knows where to start (lines 155-194).
  - We get the best solution given by the MPC solver and predict where the car will be in the future and we return that value (lines 238-248).

---

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

---

## References

1. [Udacity](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/338b458f-7ebf-449c-9ad1-611eb933b076/concepts/ecca4e03-46d1-44c6-a34e-99c0f653414e)
2. [Wikipedia](https://en.wikipedia.org/wiki/Model_predictive_control).

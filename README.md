# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

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

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).


## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.


## Project Description:

The main objective of this project is to implement Model predective controller using the c++ to steer a car around the track using the udacity simulator. Simulator provides feed of values containing the car position, speed and heading direction. All the co-ordinates to find the car location using the global co-ordinate system.

### Model:
In the PID model we were focused only the steering control but in this Project MPC we also control the throttle of the vehicle. In this project vehicle model used as kinematic model. The model takes the changes of heading direction in to account. 

Below are the project steps for this model

1. Set N and dt.  Here N is the number of time steps, dt is the time elapse between the actuations.
2. Fit the ploynomial to the waypoint.
3. Calculate the CTE and orientation error values.
4. Define the component of the cost function
5. Define model constarints.

MPC model uses the information from the simulator
* x- x position of the car in the global co-ordinate system
* y- y position of the car in the global map 
* psi - heading direction 
* v - current velocity of the vehicle.
* cte - cross track error
* epsi - orientaiton error

I have used the following update equations in my model

            fg[1 + x_start + j] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
            fg[1 + y_start + j] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
            fg[1 + psi_start + j] = psi1 - (psi0 - v0/Lf * delta * dt);
            fg[1 + v_start + j] = v1 - (v0 + a * dt);
            fg[1 + cte_start + j] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
            fg[1 + epsi_start + j] = epsi1 - ((psi0 - psides0) - v0/Lf * delta * dt);

### Timestep Length and Elapsed Duration (N & dt)
N is the number of timesteps in the horizon. dt is how much time elapses between actuations. The numner of steps impacts the controller performance. I tried different values for N and dt to drive the car smoothly around the track. 
If i increase the value N then controoler starts run slower. After trying the N value in between 10 to 20 then i decided to use 10. if the dt value is large then car drive smoothly but it navigates to the road in sharp bends. So i decided to use 100 ms i.e 0.1

### Polynomial Fitting and MPC Preprocessing
The waypoints provided by the simulator are transformed to car co-ordinate system. The coordinates of waypoints in vehicle coordinates are obtained by using the below code

                  for (int i = 0; i < ptsx.size(); i++) {
                        double shift_x = ptsx[i] - px;
                        double shift_y = ptsy[i] - py;

                        waypoints_x.push_back(shift_x * cos(-psi) - shift_y * sin(-psi));
                        waypoints_y.push_back(shift_x * sin(-psi) + shift_y * cos(-psi));

                    }

using the polyfi() functions by using the waypoints we can calculate the coeffs then using ployeval funciton we can get the cte and epsi values.
                    auto coeffs = polyfit(waypoints_x_eig, waypoints_y_eig, 3);

                    double cte = polyeval(coeffs, 0);
                    double epsi = -atan(coeffs[1]);                    

These polynomial coefficinets are used to calculate the cte and epsi later on.

### Model Predictive Control with Latency                    
In my model i added 100ms latency between the actuator calculation and when the simulator actually perform the action. I tried different latency values by changing the N and dt values but it always failed to finish the track at curves. I calculated the state of the car dt second from the current state and prediction controls initialized by that state.

## Output Result:

Here's a [link to my video result](https://youtu.be/u1OSUC6-rtI) 
Due to the limitation in github, i uploaded my vide to the youtube and provided the link above.


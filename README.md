# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

In this project we'll revisit the lake race track from the Behavioral Cloning Project. This time, however, we'll implement a PID controller in C++ to maneuver the vehicle around the track!

## Objective

The objective of this project is to have the car race around the track safely (without leaving the track bounds) as fast as possible by controlling it with a PID controller.

A successful implementation requires:

* Compilation on any platform (Linux, Mac, Windows)
* Control the steering using a PID controller as taught in the lessons
* Tune the controller's hyper parameters to drive safely around the track
* Successfully complete at least a full lap around the track

[image1]: ./Spreadsheet.png
[image2]: ./Simulation.gif

### Here I will consider each point individually and explain how I addressed them in my implementation

#### Compilation on any platform

The starter code provided for the project was already meant to compile correctly in Linux, Mac and Windows; although compilation in Windows required installing Windows 10 Bash on Ubuntu or using Docker to create a virtual environment.

Since my current platform is Windows but I have low resources, I decided to go for a native approach to avoid the installation of such resource-intensive environments. This decision was originally taken during the implementation of the first project of the term [Extended-KalmanFilter](https://github.com/satori-stan/CarND-Extended-Kalman-Filter-Project). Then, it had a non-trivial time cost, which I was able to capitalize on for this project. There is little in the repository that will give my choice away, except for a couple of lines at the end of the CMakeLists.txt file where the names of the libraries are redefined when a Windows environment is found.

The main program file (main.cpp) was modified initially to accommodate api changes in uWS 0.14.4 and finally to implement parts of the hyper parameter tuning algorithm. The main file receives a measurement package message from the simulator and passes it to the PID class, where the information is processed.

#### Control of the steering using a PID controller as taught in the lesson

The PID controller is the combination of a proportional, integral and derivative responses to the Cross-Track Error (CTE) between the expected value and the current value. It calculates an actuation or control value to apply to the system in order to reduce the CTE.

The PID is defined in pid.h and implemented in pid.cpp. The PID code has basically two methods: one to initialize the controller (Init @ pid.cpp:19) and another to issue a new control value for the system (Correct @ pid.cpp:30).

The initialization of the controller resets internal flags and counters as well as setting the coefficients for the controller.

The correction value is calculated by multiplying the cross-track error times the proportional coefficient. It also accumulates the error value, multiplies it times the integral coefficient. These first two operations can be applied for every execution, but the third needs at least one previous execution to have happened. The last calculation takes the previous and current error, calculates the time elapsed between measurements, approximates a differential and multiplies it times the differential coefficient.

The last step in the correction method is a validation to make sure the output is within certain desired bounds (pid.cpp:53). This is, of course, not part of the PID controller logic but still convenient to ensure the output values make sense.

The program creates two controllers on startup: one for steering and one for throttle, since it makes sense only to increase the speed when the car is on the expected trajectory and to decrease it when the car starts to stray. The controllers are instantiated in the main program method (main.cpp:42). Since creating the PID controllers requires selecting control output limits, the steering has -1 to 1 range consistent with the simulator's limits. The throttle controller has the same limit. This range will later allow us to control the car's maximum speed.

The controllers' parameters are ideally stored in a configuration file (params.txt). This allows testing new coefficient values without the overhead of recompilation. The program reads the file if possible (main.cpp:68-95) or reverts to hardcoded values. The hardcoded values represent the best parameters found after tuning the system to drive around the lakeside track.

With the controllers configured, each time a new reading is received from the sensors, the steering controller is called with the CTE to get a new steering value (main.cpp:134). As it is explained in the code, a factor is added to increase the response as the speed of the car increases. After that, the throttle controller is called with a combination of the CTE and steering values as input (main.cpp:145). This follows the logic that the target value to increase the speed is when the steering is set to going straight and we are driving along the desired track. Turning the wheels either for course correction or when following a curve should require the vehicle's speed to drop.

The output of the throttle controller is actually not used raw, but rather transformed (translated) to have a value consistent with the expected behavior. First, when the steering is zero, the throttle controller's CTE would be zero. At this point we want the car to reach maximum throttle (identified so far as 0.3) so the controller's output is subtracted from the maximum throttle. Second, the sign of the output is irrelevant to the system: we want a zero output when the tires are straight and up to twice the maximum throttle when the steering is maxed out to either direction so we use the absolute value of the controller's output. This is better than limiting the output of the controller since it provides the full range of controller's values.

#### Tune the controller's hyper parameters to drive safely around the track

Tuning of the controllers was a very labor-intensive task. Initially, the tuning was done manually, following the Twiddle algorithm from the lessons in a spreadsheet.

![Screen capture of manual twiddling][image1]

With deadlines pending, I modified the program and added a class to test parameters automatically, although I was reluctant since this is something that will almost certainly not be possible to do for a vehicle outside of a simulator.

The new class is defined in twiddler.h and implemented in twiddler.cpp. It has a method to check if the desired number of steps has been reached (HasFinishedSteps @ twiddler.cpp:20), a method to check if the parameter delta is under the expected tolerance (IsInTolerance @ twiddler.cpp:24) and a method to adjust the parameters (Twiddle @ twiddler.cpp:28) along with a couple of private methods that complement the algorithm.

The class is instantiated in the main program (main.cpp:103) and it takes as parameters: the number of steps to allow for the test, the tolerance for the sum of the deltas (under which the test will stop) and the parameter and delta vectors.

The program will only go into tuning mode if an argument is provided. This argument represents the number of steps for each loop with the current parameter set.

Once the program is running, the square of the CTE is accumulated. Following the twiddle algorithm, we check with every measurement received from the sensors (assuming we are in tuning mode) if the number of steps for the test has been reached. Checking if the number of steps has been reached also increases the step count (twiddler.cpp:21). When the desired number of steps have been elapsed, if the deltas are already under the tolerance (which means that further adjustments will have little or no effect) the program will disconnect from the sensors and stop the car. If, on the other hand, the parameters can still be tuned then they are (using the sum of squared CTE values) and the controllers are re-initialized.

The main twiddle algorithm follows the rules:
1. Increase the current parameter by the delta and test. If the average of the squared error is better than the previous, keep the new parameter value and increase the delta by a factor. Move to the next parameter.
1. If the result is not improved by increasing the parameter value, decrease it (from its setting before the increase) by the delta and test. If the result is better than the previous, keep the new parameter value and increase the delta by a factor. Move to the next parameter.
1. If the result is not improved by decreasing the parameter value, decrease the delta by a factor and move to the next parameter.

The implementation is somewhat hard to follow since it was modified to avoid wrapping the main program. It behaves as a state machine. The main Twiddle code (twiddler.cpp:29-63) handles the calculation of the squared error average, checking to see if the value is better than the last and returning the updated parameter vector. The ApplyDelta (lines 70 to 85) and NextParameter (lines 87 to 92) functions handle the parameter adjustments and keeping track of which parameter is being modified in turn, respectively.

#### Successfully complete at least a full lap around the track

Here is a small animation of the car driving around the track.

![Car controlled by PID in lakeside track simulator][image2]

The maximum average speed achieved was around 22 mph and although it wouldn't be a comfortable ride, it would be a safe one (as long as we are the only vehicle in the track).

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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

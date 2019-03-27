# CarND-Controls-PID
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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## Project Implementation

In this project a cascaded PID controller has been implemented to control the steering and throttle of the vehicle. The first PID calculates the steering value to minimize the CTE (cross-track error) in order to steer the car towards the center of the lane on the road. The second PID controller calculates the throttle position in order to achieve a target speed to ensure the vehicle drives as fast as possible while still ensuring it modulates the speed to assist in reduction of the CTE.

## Project Reflection

The "P" for proportional makes the car steer in proportion to the cross track error. Setting the "Kp" value, which is the constant for "P", very high, make the car overshoot the desired value as the controller tends to correct itself. The constant attempt to reach the desired value with repeated over-shooting results in high oscillations around the desired value. Setting the "Kp" value too low make the "P" controller un-responsive and car reduces the CTE or reaches the desired value so slow that it tends to go off-track if the steering angle is not pointing towards the center of the lane.

The "I" for integral sums up all the CTEs as the time passes by. If too many negative CTE values add up, which means the car has been to left of the middle of the lane for a long time, the aggregated value assists the controller in quickly increasing the steering value to bring the car towards the center to reduce the accumulated error. A low "Ki" value means the controller needs to build up large error for the "I" controller to contribute and stays ineffective. A higher "Ki" value reduces the oscillations significantly by amplifying the accumulated error quickly and ensuring quicker turns towwards the desired value.

The "D" for differential adds a smoothing effect by adjusting the controller to respond in proportion to the rate of change of error. It makes the steering more responsive when the CTE is large to reach the desired value faster but gradually reduces its responsiveness as it approaches the desired value to prevent overshooting. A high "Kd" value was increasing the osscillations as the car was becoming too responsive when it was away from the middle of the lane and very low "Kd" value was enough to smoothen out the drastic changes in steering angles while cornering giving a much smoother ride.

## PID Tuning

Traditional manual tuning method was adopted to tune the three PID gains. The gains for the steering were tuned first to ensure the car was riding smoothly between the lane and was able to finish the lap safely. The the PID gains for throttle were tuned to make the car go faster while still driving safely and smoothly. The trials and their effects are listed in the **main.cpp** file as a log.

# CarND-PID Control Project-P4
Udacity Self-Driving Car Nanodegree - PID Control Project 

# Overview
In this C++ project, I have used a Proportional-Integral-Derivative Controller, or PID controller, in order to drive a simulated car around a virtual track. The project involves implementing the controller primarily for the steering angle of the car (though I have also controlled the throttle), as well as tuning coefficients for each PID value in order to calculate a steering angle that keeps the car on the track.

#### Project Steps
* Implement PID Controller for Steering (Additional: controlling throttle as well)
* Optimize init parameters for each PID coefficient

## Results / Reflection

#### Components of PID
The actual implementation of code for a basic PID controller is fairly straightforward, but making the controller actually perform well is the tough part. Having knowledge of each part of "PID" is important:

The P or the "propotional" component has the highest impact on the car's performance. It is propotional to the cross track error (CTE) - the distance of the car from the center of the lane. This CTE is provided at each step by the simulator. Because of the P component when the car is far to the right it steers hard to the left, and when it is slightly to the left it steers slightly to the right.

The D, or "differential", component counteracts the P component's tendency to ring and overshoot the center line. If only P component is used then in the controller we observe a large swinging behavior. The car reaches the lane center, then overshoots by a large amount and then come back again, causing an oscillating behavior. But, a proper D parameter reduces this oscillation and bring the approach to the center line smoothly without oscillating.

The I, or "integral", component counteracts a bias in the CTE which prevents the P-D controller from reaching the center line. This bias can take several forms, such as a steering drift (as in the Control unit lessons) etc.

I have calculated a RMSE value of the CTE to measure the behavior of the controller.

The final PID controller video is given below.

[Final Parameters](https://github.com/nandi-abhishek/CarND-PID-Control-Project/blob/master/Final.mp4)

The following video demonstrates the subtle difference in performance when the 'P' component is removed from the controller. The car drives for a moment and then goes of the track.

['P' Parameter Removed](https://github.com/nandi-abhishek/CarND-PID-Control-Project/blob/master/ID.mp4)

This video demonstrates the disastrous effects of removing the 'D' component from the controller. It begins to swing back and forth across the center line until finally leaving the track.

['D' Parameter Removed](https://github.com/nandi-abhishek/CarND-PID-Control-Project/blob/master/PI.mp4)

When I removed the 'I' component the car was able to navigate through the track but with large RMSE value which means it is far from the lane center. With the 'I' component, RMSE after 3000 steps is 0.664590 and without the 'I' component it increases to 0.724511.

#### Finding the right coefficients

Hyperparameters were tuned manually by trial and error. Initially I tuned 'P' component and then adjusted 'D' to reduce oscillation. Finally 'I' component was tuned looking at the RMSE values. I have also implemented a Twiddle algorithm to automatically figure out best values but observed that it is not improving the result significantly. The Twiddle code is removed from final submission.

Once the parameters are tuned at low speed around 30 mph I have increased the speed and tuned them for higher speed. With higher speed I figured out that throttle also requires a tuning. I made throttle a function of steering angle.

    Throttle = (1 - 4 * speed/60 * fabs(steer_value)) * 0.6 + 0.4;

The idea is that, as steering angle increases it means the car is most likely going through a curve so we need to apply brake to reduce speed. Otherwise, the car may go off track at the curve with higher speed. With this throttle control I was able to achieve speed around 80 mph and successfully drive the car around the track.

The reduction in throttle is also a function of speed ('speed/60'). Once speed is reduced the reduction rate also reduces. Without this the car speed was reducing more than it is required to navigate the curve. Without this factor the car speed reduced to 34 mph at the last curve. But, once I introduced this factor it was able to drive that same curve at 40 mph.

#### Other observations

The behavior of the system depends quite heavily on the load in the system where the controller is running. I observed that if I am running many applications where some of them are cpu intensive then the PID controller performance detoriates. I think this is due to large gap between the time interval the simulator calls the calculation engine causing the car behave erratically. JFYI, here are my system configuration where I have run the controller.

    Processor: Intel(R) Core(TM) i7-5600U CPU @ 2.60 GHz 2.59 GHz
    RAM: 16.0 GB
    System type: 64-bit OS
    OS: Windows 10


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
* [uWebSockets](https://github.com/uWebSockets/uWebSockets) == 0.13, but the master branch will probably work just fine
  * Follow the instructions in the [uWebSockets README](https://github.com/uWebSockets/uWebSockets/blob/master/README.md) to get setup for your platform. You can download the zip of the appropriate version from the [releases page](https://github.com/uWebSockets/uWebSockets/releases). Here's a link to the [v0.13 zip](https://github.com/uWebSockets/uWebSockets/archive/v0.13.0.zip).
  * If you run OSX and have homebrew installed you can just run the ./install-mac.sh script to install this
* Simulator. You can download these from the [project intro page](https://github.com/udacity/CarND-PID-Control-Project/releases) in the classroom.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 


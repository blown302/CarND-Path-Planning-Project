# Path Planning Project

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

## Strategy

### Lane following

Used the suggested spline library to fit a smooth line to source and destination based on
frenet coordinates.

### Car following

When the car comes up on traffic and the car directly in front breaks a threshold, it will
match the speed to the in-front car to prevent a collision.

### Trajectory Distance Cost

Adds cost to potential trajectories based on distance from other cars. The distance has 2 sets
of thresholds. One for long distance and one for closer higher risk distances where the weight
is much higher.

### Change Lane Cost

In order to ties that cause the first trajectory to be evaluated selected, a small cost was 
added if a lane change was to occur.

### Speed Cost

Added cost to for vehicle detections that were off of the target speed.

### Trajectory Selection

Selected the trajectory with the lowest cost.

### Collision Avoidance

Primarily distance cost was used to avoid trajectories there are high risk.

## Future Enhancements

- Use a PrepareLaneChange state and allow vehicle control to determine when to perform
the lane change.
- Use predictions on vehicle location to factor in latency. 

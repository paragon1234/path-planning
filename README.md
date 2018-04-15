# CarND-Path-Planning-Project

## Goals
In this project we safely navigate a vehicle around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. We are provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit (without exceeding it), which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

## Inputs
#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.


## Implementation
The following steps are used:
1. Construct interpolated waypoints for driving vehicle (using spline)
2. From sensor fusion data determine whether the car can travel in the same lane
3. From sensor fusion data, predict the future positions of the nearby vehicles.
4. From step 2 and 3, determine whether the vehicle can travel in same lane or need to take left/right lane change
5. Produce new path

### 1. Construct Interpolated Waypoints of Nearby Area 

The track waypoints given in the `highway_map.csv` file are spaced roughly 30 meters apart, so the first step in the process is to interpolate a set of nearby map waypoints and produce a set of much more tightly spaced (0.5 meters apart) waypoints. The path at current position and at a distance of 30m, 60m and 90m are used, to create smooth curve using spline. The curve between current position and 30m is interpolated at 0.5m. Each frame process a point in 0.02s, this result in velocity not exceeding the speed limit.


### 2,3&4. Generate Predictions from Sensor Fusion Data

The sensor fusion data received from the simulator in each iteration is used for each of the other cars on the road. It determines whether the vehicle is close to other vehicle in the same lane. If so, then it determines whether it can safely change lane to left/right lane. Otherwise, it reduces speed of the vehicle so that a safe distance can be maintained from the next vehicle in the lane.


### 5. Produce New Path

The simulator returns the list of points from previously generated path that are not traversed. This is used to project the car's state into the future. This helps to generate smoother transitions. From there a spline is generated beginning with the last two points of the previous path that have been kept (or the current position, heading, and velocity if no current path exists), and ending with three points 30, 60 and 90 meters ahead and in the target lane. This produces a smooth x and y trajectory. To prevent excessive acceleration and jerk, the velocity is only allowed increment or decrement by a small amount.

## Conclusion

The resulting path planner works well, but not perfectly. It has managed to accumulate incident-free runs. However, there are a few performance issues. For example conside the case: if the vehicle is in the left lane and the next right lane is blocked, but right most lane is free. Then vehicle should immediately make change to right lane (by slowing down) so that in the next step it can transition to rightmost lane which is free. However, in the simulation it is not capable of such complex behavior and waits till it gets sufficient clearance to transition to right lane.


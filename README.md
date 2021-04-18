# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

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

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

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
* Eigen library

## Details

I started off researching some previous implementations of this project. I settled on a technique that uses a [spline](https://github.com/udacity/CarND-Path-Planning-Project) and based my submission on that implementation.

The car uses a controller visits every point it recieves in the list every .02 seconds. The car should not have an acceleration over 10 m/s^2 or a jerk over 50 m/s^3 in total. This requires that a distance increment be calculated such that the path planner does not exceed these terms or the speed limit. This was determined by specifying three waypoints ahead of the car at intervals of a parameter known as the "safe following distance." From these values a spline was created. This spline was then applied to a single interval of the safe following distance. This yielded a target (x,y) coordinate pair and the distance increment to be covered. This allowed me to interpolate new points over a distance of one interval create a valid path.

This required incorporation of the sensor fusion data to detemine where the path should go. There were three boolean variables that served as decision flags. The first check was to see if there was a car too close to me and ahead of me in the same lane. This was done by checking the cars Frenent coordinates to make sure that the s was greater than my car, the difference in s is less than or equal to the following distance, and the d was within the bounds of \[4 + 4\*lane, 4\*lane\].

The next check was to see if the lane to the left or right of my car was occupied and whether or not it would be valid to change lanes to that lane. There was a lane to left if the current lane minus one was greater than or equal to zero. For a lane to the right: the current lane plus one was less than or equal to two. Then it was checked to see if the detected car was in that lane by comparing the d coordinate similarly as before. If all of these were true, the lane was determined to be occupied or invalid to switch to. 

Once these flags were set, a second check was used to determin the action to be taken. The first flag checked was if the detected car was too close. If false and the car's current speed was less than 50 MPH the reference velocity was increased causing the car to speed up. If true the flags checking the left lane, then the right lane were checked. The first one to be true determined the lane to switch to.

The simulation was run a few times to tune parameters. First was the incremental value for increasing/decreasing the reference velocity. This was tuned until changes in speed resulted in acceleration and jerk values that were within limits. Next, the following distance was tuned to avoid collisions. Approximately 20m tended to avoid collisions, but resulted in highly jerky movement. 35m was largly safe, but occasionally resulted in collisions in congested traffic. It ended up that 30 m worked best.

## Conclusions
Using a spline and sensor fusion data with a rough map can be used for successfully defining a path for highway driving. A perfect controller can also make the drive smooth comfortable and safe. One notable problem was the issue with being too close and not having a valid lane to which arose in congested traffic. This led to a yo-yoing effect where the car would speed up, get too close, then need to slow down again. A better behavior would be to simply maintian the safe following distance. This might be best addressed by reworking the simulation to directly control the linear velocity and yaw rate and controlling the systems dynamics as opposed to setting up an explicit path. 





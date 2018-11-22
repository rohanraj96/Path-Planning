# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

### Goals
The goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The simulator provides with the data of all other cars every 0.02 cycle, there is also a sparse map list of waypoints around the highway. The tries to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, in the environment where other cars try to change lanes too. The car avoids hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another through appropriate cost functions. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car tries to minimize jerk and total acceleration and jerk does not go over 10 m/s^2 and 10 m/s^3 respectively.

###

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

## States

I have used a finite state machine with 5 states: "KL"(keep lane) , "PLCL"(prepare lane change left),"PLCR"(prepare lane change right),"LCL"(lane change left) and "LCR"(lane change right).

## Kinematics

	```
      if (get_vehicle_ahead(predictions, lane, vehicle_ahead)) {

        if (get_vehicle_behind(predictions, lane, vehicle_behind))
            new_velocity = vehicle_ahead.v;

        else
        {
            double s = this->s + this->v;
            double max_velocity_front = (vehicle_ahead.s - s - this->preferred_buffer) + vehicle_ahead.v - 0.5 * (this->a);
            new_velocity = min(min(max_velocity_front, max_v_a_limit), this->target_speed);
        }

	```

In the get_kinematics function, I check if there is a vehicle in front and in the back of ego vehicle. In this case, I follow the lane traffic speed.
However, if there is no vehicle behind me but one ahead of me, then I also reduce my distance from the vehicle in front to satisfy the buffer condition.

Later, I take the jerk thresholds and speed limit into consideration to avoid red flags.
The cost function module is used to rank the trajectories predicted for all the states according to predefined crieteria in increasing preferance order:

### Costs

1. Crash cost

```
    double cost = crash_distance(car_s, trajectory, predictions, data["intended_lane"]);
    
    return cost;

```
The crash cost function primarily uses the crash distance function to calculate the cost.
It tries to capture the cost associated with a potential crash. More concretely:

```
    if(!found_vehicle_front && !found_vehicle_back)
        return 0;

    else if(found_vehicle_front || found_vehicle_back)
    {
        if(found_vehicle_front && !found_vehicle_back)
        {
            delta += (vehicle_front.s - duplicate.s);
            delta =  1 - delta/(horizon);
            return delta;
        }
        else if(!found_vehicle_front && found_vehicle_back)
        {
            delta += (duplicate.s - vehicle_back.s);
            delta = 1 - delta/(horizon);
            return delta;
        }
        else if(found_vehicle_front && found_vehicle_back)
        {
            delta += (vehicle_front.s - vehicle_back.s);
            delta = 1 - delta/(2 * horizon);
            return delta;
        }    
    }
    return delta;
```
Here, the cost is 0 when there is no vehicle in front or at the back of the ego vehicle.
Alternatively, there's a cost associated whenever there's a vehicle in front or behind the ego vehicle in the same lane.

2. Wrong Lane cost

```
    return abs(helper_data["final_lane"] - helper_data["intended_lane"]);
```
This cost just punishes the ego for not being in the desired lane. This value is later multiplied by a weight.

## Trajectory Generation

I have used cubic spline library to generate trajectory as follows and then I transform all the coordinated from the world-coordinate system to the ego-coordinate system as follows:

```
            for (int i = 0; i < ptsx.size(); i++) {
              double shift_x = ptsx[i] - ref_x;
              double shift_y = ptsy[i] - ref_y;

              ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
              ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
            }

```

## Latency

The simulator is intelligent enough to deal with latency issue and I did not feel the need to add source code to deal with any latency.

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







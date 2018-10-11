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

## Kinematics and Cost

The car moves on the trajectory that is supplied every cycle to the simulator.The car then follows this trajectory.To minimize the jerk , the trajectory is generated in a manner that the acceleration in both lateral and longitudinal direction is below the threshold limit.The trajectory is generated looking at different parameters such as position of other cars and speed of different lanes.At every cycle,I predict next point of the car after 1 second.The kinematic equations help to map next point on the map.I will call this "next point" a state.At every instant ,I choose next states where there is a possibility of the car to go keeping the restrictions in mind ,such as the car already on the leftmost lane cannot go any further left.

The states are "KL"(keep lane) , "PLCL"(prepare lane change left),"PLCR"(prepare lane change right),"LCL"(lane change left) and "LCR"(lane change right).Every cycle,the state where the car can go are decided according to the strict rules.Please follow lines 92-115 in vehicle.cpp.After all the possible future states are in place ,the kinematics for every state is computed.The kinematics involves Newton's laws of motion to predict future state.Every state consists of vehicle information at a future point which are depicted downwards.

### State - (velocity,position,acceleration,lane,state string).

The kinematics returns the future state information to the cost function which in turns returns the best possible trajectory for the car to move on.

## Kinematics

	```
	if (get_vehicle_ahead(predictions, lane, vehicle_ahead)) {

        if (get_vehicle_behind(predictions, lane, vehicle_behind)) {
            new_velocity = vehicle_ahead.v; //must travel at the speed of traffic, regardless of preferred buffer
        } else {
        	double s = this->s + this->v;
            double max_velocity_in_front = (vehicle_ahead.s - s - this->preferred_buffer) + vehicle_ahead.v - 0.5 * (this->a);
            
            new_velocity = min(min(max_velocity_in_front, max_velocity_accel_limit), this->target_speed);
        }
    } else {
        new_velocity = min(max_velocity_accel_limit, this->target_speed);
    }
    
    new_accel = new_velocity - this->v; //Equation: (v_1 - v_0)/t = acceleration
    new_accel = min(max(-10.0,new_accel),10.0);
    new_position = this->s + new_velocity + new_accel / 2.0;
	```

The kinematic module first checks if any other vehicle is present in the front of the ego vehicle ,and tries to match its speed while maintaining a buffer distance of 8 metres in this case .If there is any vehcicle at the back ,then the vehicle mathces the speed of the vehicle at the back.If there is no vehicle in the front and at the back,then the ego vehicle tries to go at the max velocity keeping in mind the acceleration and velocity thresholds.

The cost function module is used to rank the trajectories predicted for all the states according to predefined crieteria in increasing preferance order:

1. Safety

The ego vehicle should not collide with any car while maintaining lane or changing lanes.
The penalty is high for collision while changing lanes to take into account the change in speeds of the cars in front or back.Also,the kinematics generated tries to maintain the buffer distance in the keep lane state.
```
	if(trajectory[i].state == "PLCL" || trajectory[i].state =="PLCR" || trajectory[i].state=="KL")
	{
	    if(trajectory[i].state =="KL")
	    {
	        cost+=700*collision_cost(vehicle.s,trajectory[i],predictions,trajectory_data);
	    }
	    else{
	        cost+=1000*collision_cost(vehicle.s,trajectory[i],predictions,trajectory_data);
	    }
	}

```

2. Efficiency
The car tries to choose the fastest lane possible from all the trajectories generated.The maximum velocity on which the car can run is 50MPH.

```
	double added_cost = 1;
    double proposed_speed_intended = lane_speed(trajectory,predictions, data["intended_lane"]);
    if (proposed_speed_intended < 0) {
        if((trajectory.state == "PLCL" && trajectory.lane ==0 )|| (trajectory.state =="PLCR" && trajectory.lane ==2))
        {
            proposed_speed_intended = 0;
            added_cost = 100;
        }
        else{
            proposed_speed_intended = trajectory.target_speed;
        }
    }

    double proposed_speed_final = lane_speed(trajectory,predictions, data["final_lane"]);
    if (proposed_speed_final < 0) {

        proposed_speed_final = trajectory.target_speed;
    }
    
    double cost = (2.0*trajectory.target_speed - proposed_speed_intended - proposed_speed_final)/trajectory.target_speed;
    cost*=added_cost;
    return cost;
```
The added cost here depicts that the car cannot go any further left when it is in the leftmost lane and same for the rightmost lane as well.

3. Change lane cost
It is assumed that changing lanes should not be done while the current lane is as good as other lanes.Threfore there is an inherent low cost added while changing lanes too.

```
double get_lane_cost(Vehicle &trajectory,map<string,float>&data)
{
    return abs(data["final_lane"] - data["intended_lane"]);
}
```

## Trajectory Generation

I have used cubic spline library to generate a three degree smooth curve between the predicted future state and the previous path's end point.I also add some points ahead in future in the same lane so that the coordinate space is not sparse.

```
	vector<double> next_wp0 = getXY(ref_s_1+30,(2+lane_number*4),map_waypoints_s,map_waypoints_x,map_waypoints_y);
    vector<double> next_wp1 = getXY(ref_s_2+60,(2+lane_number*4),map_waypoints_s,map_waypoints_x,map_waypoints_y);
    vector<double> next_wp2 = getXY(ref_s_3+90,(2+lane_number*4),map_waypoints_s,map_waypoints_x,map_waypoints_y);

    tk::spline s;

    s.set_points(pts_x,pts_y);
```

After the curve is generated ,I shift all the points in the trajectory from the original coordinate space to the car coordinates and rotate it according to the yaw of the car.
```
	for (int i = 0; i < pts_x.size(); i++) {
      // Shift car reference angle to 0 degrees
      double shift_x = pts_x[i] - ref_x;
      double shift_y = pts_y[i] - ref_y;

      pts_x[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
      pts_y[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
   	}
  ```

## Latency

The code predicts the kinematics of the car for further 1 sec.The car tries to cover every waypoint every 0.02 seconds ,hence my trajectory consist of 50 points.Also,I assume that while the code runs and feed this information to the car again,the car must have covered some more waypoints.However ,the simulator takes care of this itself and before feeding it to the car.

```
for(int i=start;i<50;i++)
{
  // cout<<"inside"<<" "<<endl;
  double x_value = next_x_vals[i-1];
  double y_value = next_y_vals[i-1];
  

  double x_point = x_add_on + 0.02*ref_vel_list[i-1];
  x_add_on = x_point;
  double y_point = s(x_point);

  double x_ref = x_point;
  double y_ref = y_point;

  // Rotate back to normal after rotating it earlier
  x_point = (x_ref * cos(ref_yaw) - y_ref*sin(ref_yaw));
  y_point = (x_ref * sin(ref_yaw) + y_ref*cos(ref_yaw));


  x_point += ref_x;
  y_point += ref_y;



  next_x_vals.push_back(x_point);
  next_y_vals.push_back(y_point);
}
```


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







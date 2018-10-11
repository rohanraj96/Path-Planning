#include "cost.h"
#include "vehicle.h"
#include <functional>
#include <iterator>
#include <map>
#include <math.h>

// const float REACH_GOAL = pow(10, 6);
// const float EFFICIENCY = pow(10, 5);


float goal_distance_cost( Vehicle & vehicle,  vector<Vehicle> & trajectory,  map<int, vector<Vehicle>> & predictions, map<string, float> & data) {
    /*
    Cost increases based on distance of intended lane (for planning a lane change) and final lane of trajectory.
    Cost of being out of goal lane also becomes larger as vehicle approaches goal distance.
    */
    float cost;
    float distance = data["distance_to_goal"];
    if (distance > 0) {
        cost = 1 - 2*exp(-(abs(2.0*vehicle.goal_lane - data["intended_lane"] - data["final_lane"]) / distance));
    } else {
        cost = 1;
    }
    return cost;
}

double inefficiency_cost(Vehicle & trajectory, map<int, vector<Vehicle>> & predictions, map<string, float> & data) {
    /*
    Cost becomes higher for trajectories with intended lane and final lane that have traffic slower than vehicle's target speed. 
    */
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
        // cout<<"I am here"<<endl;
        
    }

    double proposed_speed_final = lane_speed(trajectory,predictions, data["final_lane"]);
    if (proposed_speed_final < 0) {

        proposed_speed_final = trajectory.target_speed;
    }
    
    double cost = (2.0*trajectory.target_speed - proposed_speed_intended - proposed_speed_final)/trajectory.target_speed;

    cost*=added_cost;
    return cost;
}

double lane_speed(Vehicle &vehicle, map<int, vector<Vehicle>> & predictions, int lane) {
    /*
    All non ego vehicles in a lane have the same speed, so to get the speed limit for a lane,
    we can just find one vehicle in that lane.
    */
    // for (map<int, vector<Vehicle>>::const_iterator it = predictions.begin(); it != predictions.end(); ++it) {
    //     int key = it->first;
    //     Vehicle vehicle = it->second[0];
    //     if (vehicle.lane == lane) {
    //         return vehicle.v;
    //     }
    // }

    Vehicle front_vehicle ;
    bool found_vehicle = vehicle.get_vehicle_in_lane_ahead(predictions,lane,front_vehicle);
    if(found_vehicle)
    {
        // cout<<"yes,found vehicle"<<endl;

        // if(front_vehicle.s-vehicle.s>20)
        // {
        //     return -1;
        // }
        if(front_vehicle.s > vehicle.s + vehicle.v)
        {
            return -1;
        }
        else return front_vehicle.v;
    }
    Vehicle rear_vehicle;
    found_vehicle = vehicle.get_vehicle_in_lane_behind(predictions,lane,rear_vehicle);
    if(found_vehicle)
    {
        // if(vehicle.s - rear_vehicle.s>20)
        // {
        //     return -1;
        // }
        if(vehicle.s>rear_vehicle.s+rear_vehicle.v)
        {
            return -1;
        }
        else return rear_vehicle.v;
    }
    //Found no vehicle in the lane
    return -1.0;
}


double collision_distance(double present_car_s,Vehicle &trajectory, map<int, vector<Vehicle>> & predictions,int lane)
{
    std::vector<double> collision_data;
    // bool k = 0;
    double dist = 0;
    Vehicle front_vehicle;
    Vehicle rear_vehicle;
    Vehicle copy = trajectory;
    copy.s = present_car_s;
    // cout<<"printing this"<<" "<<trajectory.s<<" "<<copy.s<<endl;

    bool found_vehicle_front = copy.get_vehicle_ahead(predictions,lane,front_vehicle);
    bool found_vehicle_back  = copy.get_vehicle_behind(predictions,lane,rear_vehicle);

    if(!found_vehicle_front && !found_vehicle_back)
    {
        return 0;
    }

    else if(found_vehicle_front || found_vehicle_back)
    {
        if(found_vehicle_front && !found_vehicle_back)
        {
            dist+=(front_vehicle.s - copy.s);
            dist =  1 - dist/(horizon);
            return dist;
        }
        else if(!found_vehicle_front && found_vehicle_back)
        {
            dist+=(copy.s - rear_vehicle.s);
            dist = 1 - dist/(horizon);
            return dist;
        }
        else if(found_vehicle_front && found_vehicle_back)
        {
            dist+=(front_vehicle.s - rear_vehicle.s);
            dist = 1 - dist/(2*horizon);
            return dist;
        }    
    }
    return dist;
}


double collision_cost(double present_car_s,Vehicle &trajectory,map<int, vector<Vehicle>> & predictions, map<string, float> & data)
{
    double cost = collision_distance(present_car_s,trajectory,predictions,data["intended_lane"]);
    
    // if(proposed_dist_intended == 0)
    // {
    //     proposed_dist_intended = 2*horizon;
    // }
    // double proposed_dist_final = collision_distance(trajectory,predictions,data["final_lane"]);
    // if(proposed_dist_final ==0 )
    // {
    //     proposed_dist_final = horizon;
    // }
    // double dist_collision = (2*horizon - proposed_dist_intended)/(2*horizon);
    
    return cost;
}


double get_lane_cost(Vehicle &trajectory,map<string,float>&data)
{
    return abs(data["final_lane"] - data["intended_lane"]);
}

// double collision_cost_sideways(Vehicle & vehicle,double car_s,map<int, vector<Vehicle>> & predictions,int lane)
// {

// }

// double max_s(Vehicle &trajectory,map<int, vector<Vehicle>> & predictions, map<string, float> & data)
// {
//     Vehicle front_vehicle;
//     bool found_vehicle_front = trajectory.get_vehicle_ahead(predictions,lane,front_vehicle);
//     double cost = collision_distance(trajectory,predictions,data["intended_lane"]);
// }


double calculate_cost( Vehicle & vehicle,  map<int, vector<Vehicle>> & predictions,  vector<Vehicle> & trajectory) {
    // cout<<"printing"<<" "<<vehicle.s<<endl;
    /*
    Sum weighted cost functions to get total cost for trajectory.
    */
    // map<string, float> trajectory_data = get_helper_data(vehicle, trajectory, predictions);
    // float cost = 0.0;
    // std::vector<double> speed_cost;
    // std::vector<double> collision_cost;
    // std::std::vector<double> lane_cost;

    // double best_cost = 100000;
    // int best_cost_index =0;
    double cost = 0;

    for(int i=0;i<trajectory.size();i++)
    {
        map<string,float>trajectory_data = get_helper_data(vehicle,trajectory[i],predictions);
        // cout<<trajectory[i].state<<" "<<trajectory_data["final_lane"]<<" "<<trajectory_data["intended_lane"]<<endl;
        // double cost = 0;
        cost+=500*inefficiency_cost(trajectory[i],predictions,trajectory_data);
        // cost+=10000*(1-collision_cost(trajectory[i],predictions,trajectory_data)/1000);
        cost+=100*get_lane_cost(trajectory[i],trajectory_data);
        if(trajectory[i].state == "PLCL" || trajectory[i].state =="PLCR" || trajectory[i].state=="KL")
        {
            if(trajectory[i].state =="KL")
            {
                cost+=700*collision_cost(vehicle.s,trajectory[i],predictions,trajectory_data);
            }
            else{
                cost+=1500*collision_cost(vehicle.s,trajectory[i],predictions,trajectory_data);
                // cost+=600 * collision_cost_sideways(trajectory[i],car_s,predictions,trajectory_data["intended_lane"]);
                // cost+=100*max_s(trajectory[i],predictions,trajectory_data);
            }
        }
    }
    return cost;




    // float inefficiency_cost = inefficiency_cost(vehicle,trajectory,predictions,data);


    //Add additional cost functions here.
    // vector< function<float(const Vehicle & , const vector<Vehicle> &, const map<int, vector<Vehicle>> &, map<string, float> &)>> cf_list = {goal_distance_cost, inefficiency_cost};
    // vector<float> weight_list = {REACH_GOAL, EFFICIENCY};
    
    // for (int i = 0; i < cf_list.size(); i++) {
    //     float new_cost = weight_list[i]*cf_list[i](vehicle, trajectory, predictions, trajectory_data);
    //     cost += new_cost;
    // }

    // return cost;

}

// map<string, float> get_helper_data(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions) {
//     /*
//     Generate helper data to use in cost functions:
//     indended_lane: the current lane +/- 1 if vehicle is planning or executing a lane change.
//     final_lane: the lane of the vehicle at the end of the trajectory.
//     distance_to_goal: the distance of the vehicle to the goal.

//     Note that indended_lane and final_lane are both included to help differentiate between planning and executing
//     a lane change in the cost functions.
//     */
//     map<string, float> trajectory_data;
//     Vehicle trajectory_last = trajectory[1];
//     float intended_lane;

//     if (trajectory_last.state.compare("PLCL") == 0) {
//         intended_lane = trajectory_last.lane + 1;
//     } else if (trajectory_last.state.compare("PLCR") == 0) {
//         intended_lane = trajectory_last.lane - 1;
//     } else {
//         intended_lane = trajectory_last.lane;
//     }

//     // float distance_to_goal = vehicle.goal_s - trajectory_last.s;
//     float final_lane = trajectory_last.lane;
//     trajectory_data["intended_lane"] = intended_lane;
//     trajectory_data["final_lane"] = final_lane;
//     trajectory_data["distance_to_goal"] = distance_to_goal;
//     return trajectory_data;
// }

map<string,float>get_helper_data( Vehicle & vehicle,  Vehicle & trajectory,  map<int, vector<Vehicle>> & predictions)
{
    map<string,float>trajectory_data;
    int intended_lane;

    if(trajectory.state.compare("PLCL")==0)
    {
        intended_lane = trajectory.lane-1;
    }
    else if(trajectory.state.compare("PLCR")==0)
    {
        intended_lane = trajectory.lane+1;
    }
    else intended_lane = trajectory.lane;

    int final_lane = trajectory.lane;

    trajectory_data["intended_lane"] = intended_lane;
    trajectory_data["final_lane"] = final_lane;

    return trajectory_data;
}


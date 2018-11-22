#include "cost.h"
#include "vehicle.h"
#include <functional>
#include <iterator>
#include <map>
#include <math.h>


double goal_distance_cost( Vehicle & vehicle,  vector<Vehicle> & trajectory,  map<int, vector<Vehicle>> & predictions, map<string, double> & data) {
 
    double cost;
    double distance = data["distance_to_goal"];
    
    if (distance > 0)
        cost = 1 - 2*exp(-(abs(2.0*vehicle.goal_lane - data["intended_lane"] - data["final_lane"]) / distance));
    
    else
        cost = 1;

    return cost;
}

double inefficiency_cost(Vehicle & trajectory, map<int, vector<Vehicle>> & predictions, map<string, double> & data) {

    double cost_extra = 1;

    double proposed_speed_intended = lane_speed(trajectory, predictions, data["intended_lane"]);
    
    if (proposed_speed_intended < 0) 
    {
        if((trajectory.state == "PLCL" && trajectory.lane == 0)|| (trajectory.state =="PLCR" && trajectory.lane == 2))
        {
            proposed_speed_intended = 0;
            cost_extra = 100;
        }
        else
            proposed_speed_intended = trajectory.target_speed;
    }

    double proposed_speed_final = lane_speed(trajectory, predictions, data["final_lane"]);
    if (proposed_speed_final < 0) {

        proposed_speed_final = trajectory.target_speed;
    }
    
    double cost = (2.0 * trajectory.target_speed - proposed_speed_intended - proposed_speed_final)/trajectory.target_speed;

    cost *= cost_extra;
    return cost;
}

double lane_speed(Vehicle &elektra, map<int, vector<Vehicle>> & predictions, int lane) {
   
    Vehicle vehicle_front;
    bool vehicle_ahead = elektra.get_vehicle_ahead(predictions, lane, vehicle_front);
    if(vehicle_ahead)
    {
        if(vehicle_front.s > elektra.s + elektra.v)
            return -1;
        else 
            return vehicle_front.v;
    }

    Vehicle vehicle_back;
    bool vehicle_behind = elektra.get_vehicle_behind(predictions, lane, vehicle_back);
    if(vehicle_behind)
    {
        if(elektra.s > vehicle_back.s + vehicle_back.v)
            return -1;
        else 
            return vehicle_back.v;
    }

    return -1;
}


double crash_distance(double car_s,Vehicle &trajectory, map<int, vector<Vehicle>> & predictions,int lane)
{

    Vehicle vehicle_front;
    Vehicle vehicle_back;
    Vehicle duplicate = trajectory;
    duplicate.s = car_s;
    double delta = 0;


    bool found_vehicle_front = duplicate.get_vehicle_ahead(predictions, lane, vehicle_front);
    bool found_vehicle_back  = duplicate.get_vehicle_behind(predictions, lane, vehicle_back);

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
}


double crash_cost(double car_s, Vehicle &trajectory, map<int, vector<Vehicle>> &predictions, map<string, double>& data)
{
    double cost = crash_distance(car_s, trajectory, predictions, data["intended_lane"]);
    
    return cost;
}


double wrong_lane_cost(Vehicle &trajectory,map<string,double> &helper_data)
{
    return abs(helper_data["final_lane"] - helper_data["intended_lane"]);
}

double calculate_cost(Vehicle & vehicle, map<int, vector<Vehicle>> & predictions, vector<Vehicle> & trajectory) 
{
    double cost = 0;

    for(int i = 0; i < trajectory.size() ; i++)
    {
        map<string,double> helper_data = get_helper_data(vehicle,trajectory[i],predictions);
        
        cost += 550 * inefficiency_cost(trajectory[i],predictions,helper_data);
        cost += 20 * wrong_lane_cost(trajectory[i],helper_data);
        if(trajectory[i].state == "PLCL" || trajectory[i].state == "PLCR" || trajectory[i].state== "KL")
        {
            if(trajectory[i].state =="KL")
                cost += exp(9 * crash_cost(vehicle.s, trajectory[i], predictions, helper_data));
            else
                cost += exp(13 * crash_cost(vehicle.s, trajectory[i], predictions, helper_data));
        }
    }
    return cost;
}

map<string,double> get_helper_data(Vehicle & vehicle, Vehicle & trajectory, map<int, vector<Vehicle>> & predictions){
    
    map<string, double>trajectory_data;
    int intended_lane;

    if(trajectory.state.compare("PLCL") == 0)
    {
        intended_lane = trajectory.lane - 1;
    }
    else if(trajectory.state.compare("PLCR") == 0)
    {
        intended_lane = trajectory.lane + 1;
    }
    else intended_lane = trajectory.lane;

    int final_lane = trajectory.lane;

    trajectory_data["intended_lane"] = intended_lane;
    trajectory_data["final_lane"] = final_lane;

    return trajectory_data;
}


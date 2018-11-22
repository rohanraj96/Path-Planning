#ifndef COST_H
#define COST_H
#include "vehicle.h"



using namespace std;
static double horizon = 30;

double calculate_cost(Vehicle & vehicle,  map<int, vector<Vehicle>> & predictions, vector<Vehicle> & trajectory);

double goal_distance_cost( Vehicle & vehicle,  vector<Vehicle> & trajectory,   map<int, vector<Vehicle>> & predictions, map<string, double> & data);

double inefficiency_cost( Vehicle & vehicle,  vector<Vehicle> & trajectory, map<int, vector<Vehicle>> & predictions, map<string, double> & data);

double lane_speed(Vehicle &vehicle, map<int, vector<Vehicle>> & predictions, int lane);

double wrong_lane_cost( Vehicle &trajectory,map<string,double>&data);

double crash_cost( Vehicle &trajectory, map<int, vector<Vehicle>> & predictions, map<string, double> & data);

double crash_distance( Vehicle &trajectory,map<int, vector<Vehicle>> & predictions, int lane);

map<string, double> get_helper_data(Vehicle & vehicle, Vehicle & trajectory, map<int, vector<Vehicle>> & predictions);

#endif
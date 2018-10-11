#include <math.h>
#include <fstream>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "vehicle.h"
#include "cost.h"



using namespace std;



// for convenience
using json = nlohmann::json;

static double max_s = 6945.554;
// static double horizon = 20;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

vector<pair<double,double>> pairsort(vector<double>a, vector<double>b, int n) 
{ 
    vector<pair<double,double>> vect; 
  
    // Storing the respective array 
    // elements in pairs. 

    // cout<<"printing n"<<" "<<n<<endl;
    for (int i = 0; i < n; i++)  
    { 
        vect.push_back(make_pair(a[i],b[i]));
    } 
  
    // Sorting the pair array. 
    sort(vect.begin(), vect.end());
    
    return vect;
    }

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

tk::spline generate_spline(vector<double> x, vector<double> y)
{
  tk::spline x_to_y;
  x_to_y.set_points(x, y);
  return x_to_y;
}

bool sort_by_s(vector<double> i, vector<double> j){return (i[2] < j[2]);}

void sort_map_waypoints(vector<double> &maps_x, vector<double> &maps_y, vector<double> &maps_s)
{
  vector<vector<double>> map_wps;
  for(int i = 0; i<maps_x.size(); i++)
  {
    map_wps.push_back({maps_x[i], maps_y[i], maps_s[i]});
  }
  sort(map_wps.begin(), map_wps.end(), sort_by_s);

  maps_x.clear();
  maps_y.clear();
  maps_s.clear();

  for(int i = 0; i<map_wps.size(); i++)
  {
    maps_x.push_back(map_wps[i][0]);
    maps_y.push_back(map_wps[i][1]);
    maps_s.push_back(map_wps[i][2]);
  }
}

vector<vector<double>> upsample_map_waypoints(vector<double> maps_x, vector<double> maps_y, vector<double> maps_s)
{
  sort_map_waypoints(maps_x, maps_y, maps_s);
  tk::spline s_to_x = generate_spline(maps_s, maps_x);
  tk::spline s_to_y = generate_spline(maps_s, maps_y);

  vector<double> x_upsampled, y_upsampled, s_upsampled;
  for(double i = 0; i<=max_s; i++)// resampling every 1 meter.
  { 
    x_upsampled.push_back(s_to_x(i));
    y_upsampled.push_back(s_to_y(i));
    s_upsampled.push_back(double(i));
  }
  return {x_upsampled, y_upsampled, s_upsampled};
}



// vector<Vehicle> generate_trajectory(int state,auto sensor_fusion)
// {
//   vector<Vehicle> trajectory;
//   if(state.compare("KL")==0)
//   {
//     trajectory = keep_lane_trajectory(sensor_fusion);
//   }
//   else if(state.compare("LCL")==0 || state.compare("LCR")==0)
//   {
//     trajectory = lane_change_trajectory(sensor_fusion);
//   }
//   else if(state.compare(""))
// }

int main() {
  uWS::Hub h;

  // vector<double>ref_vel_list;
  // double buffer_distance = 5;
  // bool first_time= true;
  // tk::spline s_prev;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;
  vector<double> ref_vel_list;
  Vehicle ego = Vehicle();

  // vector<double>ref_vel_list;
  double buffer_distance = 10;
  bool first_time= true;
  tk::spline s_prev;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  // global max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  // vector<vector<double>> upsampled_map_waypoints_x_y_s = upsample_map_waypoints(map_waypoints_x, map_waypoints_y, map_waypoints_s);
  // vector<double> map_waypoints_x_upsampled = upsampled_map_waypoints_x_y_s[0];
  // vector<double> map_waypoints_y_upsampled = upsampled_map_waypoints_x_y_s[1];
  // vector<double> map_waypoints_s_upsampled = upsampled_map_waypoints_x_y_s[2];

  // cout<<map_waypoints_y.size()<<endl;
  // map_waypoints_x.clear();
  // map_waypoints_y.clear();
  // map_waypoints_s.clear();

  // map_waypoints_x = map_waypoints_x_upsampled;
  // map_waypoints_y = map_waypoints_y_upsampled;
  // map_waypoints_s = map_waypoints_s_upsampled;
  // cout<<map_waypoints_y.size()<<endl;



  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&buffer_distance,&ref_vel_list,&first_time,&s_prev,&ego](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {

    // Vehicle ego = Vehicle();

    

    // cout<<"vv";
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      // cout<<"t4g4t";
      auto s = hasData(data);

      if (s != "") {
        // cout<<"GT$g";
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {

          // j[1] is the data JSON object
          
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

            

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];

            // cout<<"printing the vector size"<<endl;
            // cout<<previous_path_x.size()<<endl;
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

            // cout<<"printing the sensor fusion data"<<endl;
            // auto sensor_fusion_list;
            

          




          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
            double dist = 0.5;
            double next_s;
            double next_d = 6;
            double lane_number = 1;

            int prev_size = previous_path_x.size();
            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);
            double ref_s;

            double ref_vel = 49.5;

            double future_car_s = end_path_s;
            

            vector<double>pts_x;
            vector<double>pts_y;
            // std::vector<double> pts_s;

            // vector<std::vector<double>>spline_points;
            
            // start with the state keep lane and get next states possible.Get the cost of all the lanes ,keep lane ,and other lanes as well.
            // if the present lane is slow and it is safe to change the lane,got to prepare lane change.when inside prepare lane change ,infuse keep lane 
            // and then prep

            if(prev_size<2)
            {

              cout<<"first"<<end_path_s<<" "<<car_s<<endl;
              // map<int,vector<Vehicle>>predictions;

              ego = Vehicle(car_speed*0.447,car_s,0,1,"KL");

              // std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
              // std::chrono::steady_clock::time_point end= std::chrono::steady_clock::now();

              // std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() <<std::endl;
              // std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::nanoseconds> (end - begin).count() <<std::endl
              vector<Vehicle> preds = ego.generate_predictions(sensor_fusion,prev_size,car_s,horizon);

              map<int,vector<Vehicle>> predictions;
              for(int i=0;i<preds.size();i++)
              {
                predictions[i].push_back(preds[i]);
              }

              Vehicle future_car = ego.choose_next_state(predictions);
              // cout<<"printing the lane"<<" "<<future_car.state<<endl;
              // cout<<future_car.state<<endl;
              ego = future_car;
              lane_number = future_car.lane;

              double prev_car_x = car_x - cos(car_yaw);
              double prev_car_y = car_y - sin(car_yaw);

              //new
              double prev_car_s = getFrenet(prev_car_x,prev_car_y,car_yaw,map_waypoints_x,map_waypoints_y)[0];


              // spline_points.push_back({prev_car_x,prev_car_y,prev_car_s});
              // spline_points.push_back({car_x,car_y,car_s});

              //old

              pts_x.push_back(prev_car_x);
              pts_x.push_back(car_x);

              pts_y.push_back(prev_car_y);
              pts_y.push_back(car_y);

              


              vector<double> future_car_xy = getXY(future_car.s+30,(2+lane_number*4),map_waypoints_s,map_waypoints_x,map_waypoints_y);
              pts_x.push_back(future_car_xy[0]);
              pts_y.push_back(future_car_xy[1]);

              // spline_points.push_back({future_car_xy[0],future_car_xy[1],future_car.s});




              double vel = car_speed*0.447;
              double a = future_car.a;

              for(int i=0;i<50;i++)
              {
                vel+=0.02*a;
                vel = min(48*0.447,vel);
                ref_vel_list.push_back(vel);
              }

              ref_s = car_s;
              // cout<<"The acceleration is"<<" "<<a<<endl;
              // cout<<"prtinting velocities"<<endl;
              // for(int i=0;i<50;i++)
              // {
              //   cout<<ref_vel_list[i]<<endl;
              // }

            }
            else{

              string present_state = ego.state;
              double present_a = ego.a;
              int present_lane = ego.lane;



              ego = Vehicle(car_speed*0.447,car_s,present_a,present_lane,present_state);

              vector<Vehicle> preds = ego.generate_predictions(sensor_fusion,prev_size,car_s,horizon);

              map<int,vector<Vehicle>> predictions;
              for(int i=0;i<preds.size();i++)
              {
                predictions[i].push_back(preds[i]);
              }

              Vehicle future_car = ego.choose_next_state(predictions);
              // cout<<"printing the lane"<<" "<<future_car.state<<endl;
              ego = future_car;
              lane_number = future_car.lane;

              ref_x = previous_path_x[prev_size-1];
              ref_y = previous_path_y[prev_size-1];

              double ref_x_prev = previous_path_x[prev_size-2];
              double ref_y_prev = previous_path_y[prev_size-2];
              // cout<<"yaw_x"<<" "<<ref_x - ref_x_prev<<endl;
              // cout<<"yaw_y"<<" "<<ref_y - ref_y_prev<<endl;
              // cout<<car_s<<endl;
              // cout<<endl;
              double t = ref_x - ref_x_prev;
              // if(abs(t)<0.001)
              // {
              //   if(t<0)
              //   {
              //     t = -0.001;
              //   }
              //   else t = 0.001;
              // }
              ref_yaw = atan2((ref_y-ref_y_prev),t);




              //old
              pts_x.push_back(ref_x_prev);
              pts_x.push_back(ref_x);

              pts_y.push_back(ref_y_prev);
              pts_y.push_back(ref_y);




              vector<double> future_car_xy = getXY(future_car.s+30,(2+lane_number*4),map_waypoints_s,map_waypoints_x,map_waypoints_y);
              pts_x.push_back(future_car_xy[0]);
              pts_y.push_back(future_car_xy[1]);

              // spline_points.push_back({future_car_xy[0],future_car_xy[1],future_car.s});

              double vel = car_speed*0.447;
              // cout<<"printing the velocity of the car"<<" "<<vel<<endl;
              double a = future_car.a;

              for(int i=0;i<prev_size;i++)
              {
                ref_vel_list[i] = ref_vel_list[i+50-prev_size-1];
              }

              for(int i=prev_size;i<50;i++)
              {
                vel  = ref_vel_list[i-1]+0.02*a;
                vel = min(vel,48*0.447);
                ref_vel_list[i] = vel;
              }

              ref_s = end_path_s;

              // cout<<"The acceleration is"<<" "<<a<<endl;
              // cout<<"prtinting velocities"<<endl;
              // for(int i=0;i<50;i++)
              // {
              //   cout<<ref_vel_list[i]<<endl;
              // }

            }



            

            vector<double>ref_waypoint_s = getFrenet(ref_x,ref_y,ref_yaw,map_waypoints_x,map_waypoints_y);
            double ref_s_ = ref_waypoint_s[0];

            double ref_s_1 = ref_s_;
            double ref_s_2 = ref_s_;
            double ref_s_3 = ref_s_;

            if(ref_s_ + 30 >max_s)
            {
              ref_s_1  = max_s - ref_s_;
            }
            if(ref_s_ + 60 > max_s)
            {
              ref_s_2  = max_s - ref_s_;
            }
            if(ref_s_ + 90 >max_s)
            {
              ref_s_3  = max_s - ref_s_;
            }

            if(prev_size>0)
            {
              car_s = end_path_s;
            }

            vector<double> next_wp0 = getXY(ref_s_1+30,(2+lane_number*4),map_waypoints_s,map_waypoints_x,map_waypoints_y);
            // spline_points.push_back({next_wp0[0],next_wp0[1],ref_s+30});

            vector<double> next_wp1 = getXY(ref_s_2+60,(2+lane_number*4),map_waypoints_s,map_waypoints_x,map_waypoints_y);
            // spline_points.push_back({next_wp1[0],next_wp1[1],ref_s+60});

            vector<double> next_wp2 = getXY(ref_s_3+90,(2+lane_number*4),map_waypoints_s,map_waypoints_x,map_waypoints_y);
            // spline_points.push_back({next_wp2[0],next_wp2[1],ref_s+90});

            


            



            pts_x.push_back(next_wp0[0]);
            pts_x.push_back(next_wp1[0]);
            pts_x.push_back(next_wp2[0]);

            pts_y.push_back(next_wp0[1]);
            pts_y.push_back(next_wp1[1]);
            pts_y.push_back(next_wp2[1]);

            tk::spline s;
            

            for (int i = 0; i < pts_x.size(); i++) {
              // Shift car reference angle to 0 degrees
              double shift_x = pts_x[i] - ref_x;
              double shift_y = pts_y[i] - ref_y;

              pts_x[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
              pts_y[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
            }


            // for(int i=0;i<pts_x.size();i++)
            // {
            //   cout<<pts_x[i]<<" "<<pts_y[i]<<endl;
            // }
            s.set_points(pts_x,pts_y);



            // cout<<"priting the famous"<<endl;
            int start;
             if(prev_size<2)
             {
              next_x_vals.push_back(car_x);
              next_y_vals.push_back(car_y);
              start = 1;
             }
             else{
              for(int i=0;i<prev_size;i++)
              {
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
              }
              start = prev_size;
             }
          
            // cout<<"The yaw is"<<" "<<ref_yaw<<" "<<cos(ref_yaw)<<endl;

            double x_add_on = 0;

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
            // cout<<"printing next_x_vals size"<<" "<<next_x_vals.size()<<endl;
            // cout<<"printing next x and y vals"<<endl;
            // for(int i=0;i<next_x_vals.size();i++)
            // {
            //   cout<<next_x_vals[i]<<" "<<next_y_vals[i]<<endl;
            // }
            s_prev.set_points(pts_x,pts_y);
            
            //END
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	// this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}

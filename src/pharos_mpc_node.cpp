#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "pharos_mpc.h"

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Vector3Stamped.h>

ros::Subscriber sub_path;
ros::Subscriber sub_pos;
ros::Publisher pub_cmd;

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// In vehicle coordinates the cross-track error error cte is 
// the intercept at x = 0
double evaluateCte(Eigen::VectorXd coeffs) {
  return polyeval(coeffs, 0);  
}

// In vehicle coordinates the orientation error epsi is 
// -atan(c1 + c2*x + c3* x^2), but the car is always at x=0.
double evaluateEpsi(Eigen::VectorXd coeffs) {
  return -atan(coeffs[1]);  
}

Eigen::MatrixXd transformGlobalToLocal(double x, double y, double psi, const vector<double> & ptsx, const vector<double> & ptsy) {

  assert(ptsx.size() == ptsy.size());
  unsigned len = ptsx.size();

  auto waypoints = Eigen::MatrixXd(2,len);

  for (auto i=0; i<len ; ++i){
    waypoints(0,i) =   cos(psi) * (ptsx[i] - x) + sin(psi) * (ptsy[i] - y);
    waypoints(1,i) =  -sin(psi) * (ptsx[i] - x) + cos(psi) * (ptsy[i] - y);  
  } 

  return waypoints;

}

void ref_path_Callback(const nav_msgs::Path::ConstPtr& msg)
{
  std::vector<double> vec_x;
  std::vector<double> vec_y;
  std::cout<<"x\ty - "<<sizeof(msg->poses)/sizeof(msg->poses.begin())<<std::endl;
  for(int i=0; i<sizeof(msg->poses)/sizeof(msg->poses.begin()); i++){
    vec_x.push_back(msg->poses[i].pose.position.x);
    vec_y.push_back(msg->poses[i].pose.position.y);
    std::cout<<vec_x[i]<<"\t"<<vec_y[i]<<std::endl;
  }
}

void vehicle_state_Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
  std::cout<<"vehicle_pose: "<<msg->pose.pose.position.x<<" "<<msg->pose.pose.position.y<<std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pharos_mpc_node");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  sub_path = nh.subscribe("/mpc/ref_path", 10, ref_path_Callback); //topic que function
  sub_pos = nh.subscribe("/odom/vehicle_frame", 10, vehicle_state_Callback); //topic que function
  pub_cmd = nh.advertise<geometry_msgs::Vector3Stamped>("/mpc/cmd", 10); //topic que

  ros::spinOnce();


  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    std::cout << sdata << std::endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];

          /**
           * TODO: Calculate steering angle and throttle using MPC.
           * Both are in between [-1, 1].
           */
          // Affine transformation. Translate to car coordinate system then rotate to the car's orientation. 
          // Local coordinates take capital letters. The reference trajectory in local coordinates:
          Eigen::MatrixXd waypoints = transformGlobalToLocal(px,py,psi,ptsx,ptsy);
          Eigen::VectorXd Ptsx = waypoints.row(0);
          Eigen::VectorXd Ptsy = waypoints.row(1);

          // fit a 3rd order polynomial to the waypoints
          auto coeffs = polyfit(Ptsx, Ptsy, 3);

          // get cross-track error from fit 
          double cte = evaluateCte(coeffs);

          // get orientation error from fit
          double epsi = evaluateEpsi(coeffs);

          // state in vehicle coordinates: x,y and orientation are always zero
          Eigen::VectorXd state(6);
          state << 0, 0, 0, v, cte, epsi;

          ROS_WARN("MPC SOLVING...");
          // compute the optimal trajectory          
          Solution sol = mpc.Solve(state, coeffs);
          ROS_WARN("MPC SOLVED...");

          double steer_value = sol.Delta.at(latency_ind);
          double throttle_value= sol.A.at(latency_ind);
          mpc.delta_prev = steer_value;
          mpc.a_prev = throttle_value;

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the 
          //   steering value back. Otherwise the values will be in between 
          //   [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = -steer_value/0.436332; 
          msgJson["throttle"] = throttle_value;

          // Display the MPC predicted trajectory 
          vector<double> mpc_x_vals = sol.X;
          vector<double> mpc_y_vals = sol.Y;

          /**
           * TODO: add (x,y) points to list here, points are in reference to 
           *   the vehicle's coordinate system the points in the simulator are 
           *   connected by a Green line
           */

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          // Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: add (x,y) points to list here, points are in reference to 
           *   the vehicle's coordinate system the points in the simulator are 
           *   connected by a Yellow line
           */

          for (unsigned i=0 ; i < ptsx.size(); ++i) {
            next_x_vals.push_back(Ptsx(i));
            next_y_vals.push_back(Ptsy(i));
          }
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          //   the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          //   around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE SUBMITTING.
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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
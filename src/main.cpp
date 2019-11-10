#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include <iostream>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "MPC.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;



int main() {
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

          double a = j[1]["throttle"];
          double delta = j[1]["steering_angle"];

          // Waypoints in car coordinate system
          Eigen::VectorXd ptsx_car(ptsx.size());
          Eigen::VectorXd ptsy_car(ptsy.size());
          
          // Transform the points to the vehicle's orientation
          for (int i = 0; i < ptsx.size(); ++i) {
            // Translation of the waypoints so that the car is in the origin
            double x = ptsx[i] - px;
            double y = ptsy[i] - py;

            // Rotate the point by psi around the origin
            ptsx_car[i] = x * cos(-psi) - y * sin(-psi);
            ptsy_car[i] = x * sin(-psi) + y * cos(-psi);
          }

          VectorXd coeffs = polyfit(ptsx_car, ptsy_car, 3);



          // Current cross-track error and psi error
          // psi is now 0 after transformation
          //double epsi = 0.0 - atan(f0_diff); 
          //double cte = f0 - 0.0;

          const double Lf = 2.67;

          // Function value and its derivative
          // We now assume that the car is at x=0.0 and y=0.0
          double f0 = polyeval(coeffs, 0.0);
          double f0_diff = coeffs[1] + 2. * coeffs[2] * 0.0 + 3. * coeffs[3] * 0.0 * 0.0;

          double delay = 0.1;

          double cte = f0 - 0.0;
          double epsi = 0.0 - atan(f0_diff);

          // Add delay to the initial state
          double x_delayed = 0.0 + v * cos(0.0) * delay;
          double y_delayed = 0.0 + v * sin(0.0) * delay;
          double v_delayed = v + a * delay;
          double psi_delayed = 0.0 - (v/Lf) * delta * delay;
          double epsi_delayed = epsi - (v/Lf) * delta * delay;
          double cte_delayed = cte + (v * sin(epsi) * delay);
          
          // construct a state vector
          Eigen::VectorXd state(6);
          state << x_delayed, y_delayed, v_delayed, psi_delayed, cte_delayed, epsi_delayed;

          /**
           * Calculate steering angle and throttle using MPC.
           * Both are in between [-1, 1].
           */
          double steer_value;
          double throttle_value;

          auto vars = mpc.Solve(state, coeffs);

          steer_value = vars[0] / (deg2rad(25));
          throttle_value = vars[1];

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the 
          //   steering value back. Otherwise the values will be in between 
          //   [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          // Display the MPC predicted trajectory 

          /**
           * Add (x,y) points to list here, points are in reference to 
           *   the vehicle's coordinate system the points in the simulator are 
           *   connected by a Green line
           */
          vector<double> mpc_x_vals = {state[0]};
          vector<double> mpc_y_vals = {state[1]};
          for (int i = 2; i < vars.size(); i+=2) {
            mpc_x_vals.push_back(vars[i]);
            mpc_y_vals.push_back(vars[i+1]);
          }

          std::vector<char> vec;
          std::string str(mpc_x_vals.begin(), mpc_x_vals.end());

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          // add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
          // Uses the initially received waypoints that were converted to vehicle
          // coordinate system

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          for (double i = 0; i < 120; i += 2){
            next_x_vals.push_back(i);
            next_y_vals.push_back(polyeval(coeffs, i));
          }
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
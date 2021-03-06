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
#include "MPC.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
const double Lf = 2.67;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

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
          double delta = j[1]["steering_angle"];
          double acceleration = j[1]["throttle"];


          //Consider the latency, the calculation will start after the time lapse

          double time_lapse = 0.1;

          //Predict the state after the latency
          double px_adv = px + (v * cos(psi) * time_lapse);
          double py_adv = py + (v * sin(psi) * time_lapse);
          double psi_adv = psi - ((v * delta * time_lapse)/Lf);
          double v_adv = v + (acceleration * time_lapse);
          /**
           * TODO: Calculate steering angle and throttle using MPC.
           * Both are in between [-1, 1].
           */
          // Transform waypoints to car's perspective
          Eigen::VectorXd ptsx_transform(ptsx.size());
          ptsx_transform.fill(0.0);
          Eigen::VectorXd ptsy_transform(ptsy.size());
          ptsy_transform.fill(0.0);

          for (int i = 0; i < ptsx.size(); i++) 
		  {
		    // shift car reference angle to 90 degrees
            double shift_x = ptsx[i]-px_adv;
            double shift_y = ptsy[i]-py_adv;
            ptsx_transform[i] = shift_x * cos(-psi_adv) - shift_y * sin(-psi_adv);
            ptsy_transform[i] = shift_x * sin(-psi_adv) + shift_y * cos(-psi_adv);
          }

          //Fit data using 3rd polinomial
          auto coeffs = polyfit(ptsx_transform, ptsy_transform, 3);

          //Calculate cte and epsi. cte is the horizontal line
          double cte = polyeval(coeffs, 0);
          double epsi = -atan(coeffs[1]);

          //Create the state vector
          Eigen::VectorXd state(6);
          state << 0, 0, 0, v_adv, cte, epsi;

          //MPC solve
          auto vars = mpc.Solve(state, coeffs);

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = -1.0 * vars[0]; 
          msgJson["throttle"] = vars[1];

          // Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          for (int i = 2; i < vars.size(); i++)
          {
            if(i%2 == 0)
            {
              mpc_x_vals.push_back(vars[i]);
            }
            else
            {
              mpc_y_vals.push_back(vars[i]);
            }
          }



          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;



          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          for (int i = 0; i < ptsx_transform.size(); i++) 
		  {
            next_x_vals.push_back(ptsx_transform[i]);
            next_y_vals.push_back(ptsy_transform[i]);
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
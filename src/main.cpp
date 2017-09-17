#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include "MPC.h"
#include "json.hpp"
#include "kinematic.h"
#include "tools.h"


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
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {

          // j[1] is the data JSON object
          DataPackage Data(j[1]);

          static bool DebugInfo = false;

          if (DebugInfo){

          for (size_t k = 0; k < Data.WayPointX.size(); ++k){
            cout << "WayX " << Data.WayPointX[k] << endl;
            cout << "WayY " << Data.WayPointY[k] << endl;
          }
          cout << "x "   << Data.Input[PX] << endl;
          cout << "y "   << Data.Input[PY] << endl;
          cout << "psi " << Data.Input[PSI] << endl;
          cout << "vel " << Data.Input[VEL] << endl;
          cout << "acc " << Data.Input[ACC] << endl;
          cout << "str " << Data.Input[STR] << endl;
          }

          VectorXd ptsx_transform = VectorXd::Constant(Data.WayPointX.size(), 0.0);
          VectorXd ptsy_transform = VectorXd::Constant(Data.WayPointY.size(), 0.0);
          TransformWayPoint(Data, ptsx_transform, ptsy_transform);
          
          if (DebugInfo) cout << "ptsx\n" << ptsx_transform << endl;
          if (DebugInfo) cout << "ptsy\n" << ptsy_transform << endl;

          auto coefficients = polyfit(ptsx_transform, ptsy_transform, 3);

          if (DebugInfo) cout << "coeffs\n" << coefficients << endl;

          double Latency = 0.1;
          auto State = GetState(Data, coefficients, Latency);

          if (DebugInfo) cout << "State \n" << State << endl;

          auto Solution = mpc.Solve(State, coefficients);

          double steer_value    = CalculateSteer(Solution[0]);
          double throttle_value = CalculateThrottle(Solution[1]);

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          for (size_t i = 2; i < Solution.size(); ++i){
            if (i%2 == 0){
              mpc_x_vals.push_back(Solution[i]);
            } else {
              mpc_y_vals.push_back(Solution[i]);
            }
          }

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          double poly_inc = 2.5;
          int num_points = 25;

          for (int i = 1; i < num_points; ++i){
            next_x_vals.push_back(poly_inc*i);
            next_y_vals.push_back(polyeval(coefficients, poly_inc*i));
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.

          this_thread::sleep_for(chrono::milliseconds(100));
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

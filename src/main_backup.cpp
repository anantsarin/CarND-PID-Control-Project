#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;

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
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

std::vector<double> doTwiddle(double cte){

  double dp[] = {.5, 0.05, 0.15};
  double p[] = {.5, 0.5, 1.5};
  double tolerance = 0.0002;
  int n = 1;
  int n_max = 500;
  double sum_ = dp[0] + dp[1] + dp[2];
  double best_err = 10000.0;

  while((sum_ > tolerance) & (n < n_max)){
  //if(n > n_max){
    for(int i=0; i<3; i++){
      p[i] += dp[i];
      if(cte < best_err){
        best_err = cte;
        dp[i] *= 1.1;
      }
      else{
        p[i] -=2*dp[i];
        if(cte < best_err){
          best_err = cte;
          dp[i] *= 1.1;
        }
        else{
          p[i] += dp[i];
          dp[i] *= 0.9;
        }

      }
    }
    sum_ = dp[0] + dp[1] + dp[2];
    n +=1;
  }

  std::cout << "p =  " << p[0] << p[1] << p[2] << std::endl;
  std::vector<double> p_final;
  p_final[0] = p[0];
  p_final[1] = p[1];
  p_final[2] = p[2];
  return p_final;
}

int main() {
  uWS::Hub h;

  PID pid;
  /**
   * TODO: Initialize the pid variable.
   */
  // pid.Init(0.05, 0.0001, 1.5);

  bool twiddle = true;
  double p[] = {0.05, 0.0001, 1.5};
  double dp[] = {.001, 0.0001, 0.1};

  // int max_n = 600;
  // double total_cte = 0.0;
  // double error = 0.0;
  // double best_error = 10000.00;
  // double tol = 0.001;
  // int p_iterator = 0;
  // int total_iterator = 0;
  // int sub_move = 0;
  // bool first = true;
  // bool second = true;
  double best_p[3] = {p[0],p[1],p[2]};
  if(twiddle == true) {
    pid.Init(p[0],p[1],p[2]);
  }//else {
  //   pid.Init(0.06, 0.00031, 1.29);
  //   //pid.Init(0.05, 0.0001, 1.5);
  // }

  h.onMessage([&pid, &twiddle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
          pid.UpdateError(cte);
          if(twiddle == true){
            std::vector<double> p_final =  doTwiddle(cte);
            pid.Init(p_final[0], p_final[1], p_final[2]);
            pid.UpdateError(cte);
            twiddle = false;
          }
          steer_value = pid.TotalError();




          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value
                    << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
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
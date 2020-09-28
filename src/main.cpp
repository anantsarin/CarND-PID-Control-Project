0.0966306 0.000290158 1.4865

0.0966306 0.000290158 1.4865

0.0966306 0.000290158 1.4865
0.0966306 0.000290158 1.4865
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

bool twiddle = false;
bool init_flag = true;
bool second = true;

//double p[] = {0.5, 0.01, 0.5};
//double p[] = {0.05, 0.001, 0.5};
//double p[] = {0.05, 0.0001, 1.0};
//double p[3] = {0.06, 0.0002, 1.5};
double p[3] = {0.0966306, 0.000290158, 1.4865};
double dp[3] = {p[0]/10.0,p[1]/10.0,p[2]/10.0};
double total_cte = 0.0;
double error = 0.0;
double best_error = 10000.00;
double tol = 0.001;
double best_p[3] = {p[0],p[1],p[2]};

int n = 0;
int max_n = 400;
int p_index = 0;
int iterator = 0;
int sub_move = 0;



int main()
{
  uWS::Hub h;

  PID pid;

  pid.Init(p[0],p[1],p[2]);

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          double throttle_value = 0.3;
          json msgJson;

          if (twiddle == true){
            total_cte = total_cte + pow(cte,2);
            if(n==0){

              pid.Init(p[0],p[1],p[2]);
            }
            pid.UpdateError(cte);
            steer_value = pid.TotalError();

            // DEBUG
            n = n+1;
            if (n > max_n){
              if(init_flag == true) {
                p[p_index] += dp[p_index];
                init_flag = false;
              }
              else{
                error = total_cte/max_n;

                if(error < best_error && second == true) {
                    best_error = error;
                    best_p[0] = p[0];
                    best_p[1] = p[1];
                    best_p[2] = p[2];
                    dp[p_index] *= 1.1;
                    sub_move += 1;
                }
                else{
                  if(second == true) {
                    p[p_index] -= 2 * dp[p_index];
                    second = false;
                  }
                  else {
                    if(error < best_error) {
                        best_error = error;
                        best_p[0] = p[0];
                        best_p[1] = p[1];
                        best_p[2] = p[2];
                        dp[p_index] *= 1.1;
                        sub_move += 1;
                    }
                    else {
                        p[p_index] += dp[p_index];
                        dp[p_index] *= 0.9;
                        sub_move += 1;
                    }
                    std::cout << "error: " << error << " ";
                    std::cout << "best_error: " << best_error << " ";
                    std::cout << "Best p[0] p[1] p[2]: " << best_p[0] << " " << best_p[1] << " " << best_p[2] << " ";
                  }
                }
              }


              if(sub_move > 0) {
                p_index = (p_index+1)%3;
                init_flag = true;
                second = true;
                sub_move = 0;
              }
              total_cte = 0.0;
              n = 0;
              iterator = iterator+1;

              double sumdp = dp[0]+dp[1]+dp[2];
              if(sumdp < tol) {
                std::cout << "Best p[0] p[1] p[2]: " << best_p[0] << best_p[1] << best_p[2] << " ";
              } else {
                string reset_msg = "42[\"reset\",{}]";
                ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
              }

            } else {
              msgJson["steering_angle"] = steer_value;
              msgJson["throttle"] = throttle_value;
              auto msg = "42[\"steer\"," + msgJson.dump() + "]";
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }

          } else { //twiddle if
            pid.UpdateError(cte);
            steer_value = pid.TotalError();

            // DEBUG
            std::cout << "CTE: " << cte << " Steering Value: " << steer_value << " Throttle Value: " << throttle_value << " Count: " << n << std::endl;
            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = throttle_value;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            std::cout << msg << std::endl;
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          } //twiddle else
        }//telemtery
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}


// Connected!!!
// Intermediate p[0] p[1] p[2]: 0.05 0.001 0.5 Connected!!!
// iteration: 1 p_index: 0 p[0] p[1] p[2]: 0.06 0.001 0.5 error: 0.390555 best_error: 0.390555 Best p[0] p[1] p[2]: 0.06 0.001 0.5 Connected!!!
// Intermediate p[0] p[1] p[2]: 0.06 0.001 0.5 Connected!!!
// Intermediate p[0] p[1] p[2]: 0.06 0.0011 0.5 Connected!!!
// iteration: 4 p_index: 1 p[0] p[1] p[2]: 0.06 0.0009 0.5 error: 0.336699 best_error: 0.336699 Best p[0] p[1] p[2]: 0.06 0.0009 0.5 Connected!!!
// Intermediate p[0] p[1] p[2]: 0.06 0.0009 0.5 Connected!!!
// iteration: 6 p_index: 2 p[0] p[1] p[2]: 0.06 0.0009 0.6 error: 0.317876 best_error: 0.317876 Best p[0] p[1] p[2]: 0.06 0.0009 0.6 Connected!!!
// Intermediate p[0] p[1] p[2]: 0.06 0.0009 0.6 Connected!!!
// iteration: 8 p_index: 0 p[0] p[1] p[2]: 0.071 0.0009 0.6 error: 0.28027 best_error: 0.28027 Best p[0] p[1] p[2]: 0.071 0.0009 0.6 Connected!!!
// Intermediate p[0] p[1] p[2]: 0.071 0.0009 0.6 Connected!!!
// Intermediate p[0] p[1] p[2]: 0.071 0.00101 0.6 Connected!!!
// iteration: 11 p_index: 1 p[0] p[1] p[2]: 0.071 0.00079 0.6 error: 0.256135 best_error: 0.256135 Best p[0] p[1] p[2]: 0.071 0.00079 0.6 Connected!!!
// Intermediate p[0] p[1] p[2]: 0.071 0.00079 0.6 Connected!!!
// iteration: 13 p_index: 2 p[0] p[1] p[2]: 0.071 0.00079 0.71 error: 0.253778 best_error: 0.253778 Best p[0] p[1] p[2]: 0.071 0.00079 0.71 Connected!!!
// Intermediate p[0] p[1] p[2]: 0.071 0.00079 0.71 Connected!!!
// iteration: 15 p_index: 0 p[0] p[1] p[2]: 0.0831 0.00079 0.71 error: 0.236275 best_error: 0.236275 Best p[0] p[1] p[2]: 0.0831 0.00079 0.71 Connected!!!
// Intermediate p[0] p[1] p[2]: 0.0831 0.00079 0.71 Connected!!!
// Intermediate p[0] p[1] p[2]: 0.0831 0.000911 0.71 Connected!!!
// iteration: 18 p_index: 1 p[0] p[1] p[2]: 0.0831 0.000669 0.71 error: 0.219529 best_error: 0.219529 Best p[0] p[1] p[2]: 0.0831 0.000669 0.71 Connected!!!
// Intermediate p[0] p[1] p[2]: 0.0831 0.000669 0.71 Connected!!!
// iteration: 20 p_index: 2 p[0] p[1] p[2]: 0.0831 0.000669 0.831 error: 0.217496 best_error: 0.217496 Best p[0] p[1] p[2]: 0.0831 0.000669 0.831 Connected!!!
// Intermediate p[0] p[1] p[2]: 0.0831 0.000669 0.831 Connected!!!



// error: 0.139486 best_error: 0.139486 Best p[0] p[1] p[2]: 0.07986 0.0002662 1.4865 Connected!!!
// Connected!!!
// Connected!!!
// Connected!!!
// Connected!!!
// error: 0.139038 best_error: 0.13293 Best p[0] p[1] p[2]: 0.087846 0.0002662 1.4865 Connected!!!
// Connected!!!
// Connected!!!
// error: 0.134062 best_error: 0.13293 Best p[0] p[1] p[2]: 0.087846 0.0002662 1.4865 Connected!!!
// Connected!!!
// Connected!!!
// Connected!!!
// Connected!!!
// Connected!!!
// Connected!!!
// error: 0.125738 best_error: 0.124465 Best p[0] p[1] p[2]: 0.0966306 0.000290158 1.4865 Connected!!!
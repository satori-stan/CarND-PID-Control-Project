#include <iostream>
#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include "json.hpp"
#include "twiddler.h"
#include "PID.h"

// for convenience
using json = nlohmann::json;

// for portability of M_PI (Vis Studio, MinGW, etc.)
#ifndef M_PI
const double M_PI = 3.14159265358979323846;
#endif

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";

  } else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main(int argc, char** argv) {
  uWS::Hub h;

  PID steering_pid(-1.0, 1.0);
  //PID throttle_pid(-1.0, 1.0);

  double sum_cte_2 = 0.0;
  //RunningAverage max_speed;
  //int steps = 0;

  bool twiddle_mode = argc > 1;
  double p, i, d;

  std::ifstream config_file("params.txt");
  if (config_file.is_open()) {
    // Read the pid controller parameters from file
    config_file >> p >> i >> d;
    //throttle_pid.Init(p, i, d);
    config_file.close();
  } else {
    // Initialize the pid controllers with sensible values.
    p = 0.2;
    i = 3.0;
    d = 0.004;
    //throttle_pid.Init(1.0, 3.0, 0.004);
  }

  steering_pid.Init(p, i, d);

  std::vector<double> parameters;
  std::vector<double> delta {1.0, 1.0, 1.0};

  if (twiddle_mode) {
    parameters.push_back(p);
    parameters.push_back(i);
    parameters.push_back(d);
  }

  Twiddler twiddler {2000, 0.0001, parameters, delta};

  h.onMessage([&twiddle_mode, &steering_pid, &sum_cte_2, &twiddler](uWS::WebSocket<uWS::SERVER>* ws, char *data,
                              size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value = steering_pid.Correct(cte);
          double throttle_value = 0.2; //1 - throttle_pid.Correct(cte);
          
          // DEBUG
          //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          sum_cte_2 += std::pow(cte, 2);
          //std::cout << "Sum CTE^2: " << sum_cte_2
              //<< " Average Speed: " << max_speed.Add(speed)
              //<< " Throttle: " << throttle_value << std::endl;


          if (twiddle_mode && twiddler.HasFinishedSteps()) {
            if (twiddler.IsInTolerance()) {
              // TODO: Write parameters to file
              std::cout << "Kp: " << steering_pid.Kp_
                  << " Ki: " << steering_pid.Ki_
                  << " Kd: " << steering_pid.Kd_ << std::endl;
              // TODO: Stop simulation
              // For now just stop car
              json msgJson;
              msgJson["steering_angle"] = 0;
              msgJson["throttle"] = 0;
              auto msg = "42[\"steer\"," + msgJson.dump() + "]";
              ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            } else {
              std::vector<double> new_params = twiddler.Twiddle(sum_cte_2);
              steering_pid.Init(new_params[0], new_params[1], new_params[2]);
              sum_cte_2 = 0.0;
              // Reset simulator
              std::string msg = "42[\"reset\",{}]";
              ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
          } else {
            json msgJson;
            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = throttle_value;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            //std::cout << msg << std::endl;
            ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program doesn't compile :-(
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

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER>* ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER>* ws, int code,
                         char *message, size_t length) {
    ws->close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  auto host = "127.0.0.1";
  if (h.listen(host, port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}

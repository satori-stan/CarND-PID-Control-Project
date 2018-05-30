#include <iostream>
#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include "json.hpp"
#include "twiddler.h"
#include "running_average.h"
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

  // Our controllers with their limits
  PID steering_pid(-1.0, 1.0);
  PID throttle_pid(-1.0, 1.0);
  const double kMaxThrottle = 0.3;

  // Variables where we will be storing running values
  double sum_cte_2 = 0.0;
  double previous_angle = 0.0;
  RunningAverage avg_speed;
  RunningAverage avg_throttle;

  // Decide if we are in tunning mode or not
  bool twiddle_mode = false;
  int twiddle_steps = 0;
  if (argc > 1) {
    twiddle_mode = true;
    // TODO: Protect against non-numerical values
    twiddle_steps = atoi(argv[1]);
  }

  // Variables used in tunning mode
  std::vector<double> parameters;
  std::vector<double> delta;
  double p, i, d;

  // Read controller parameters (and tunning delta values) from file (if found)
  std::ifstream config_file("params.txt");
  if (config_file.is_open()) {
    config_file >> p >> i >> d;
    steering_pid.Init(p, i, d);
    parameters.push_back(p);
    parameters.push_back(i);
    parameters.push_back(d);

    config_file >> p >> i >> d;
    throttle_pid.Init(p, i, d);
    parameters.push_back(p);
    parameters.push_back(i);
    parameters.push_back(d);

    // Now the deltas
    if (twiddle_mode) {
      config_file >> p >> i >> d;
      delta.push_back(p);
      delta.push_back(i);
      delta.push_back(d);

      config_file >> p >> i >> d;
      delta.push_back(p);
      delta.push_back(i);
      delta.push_back(d);
    }

    config_file.close();
  } else {
    // Initialize the pid controllers with sensible values.
    steering_pid.Init(0.745753, 0.0002893, 12.1355);
    throttle_pid.Init(0.814421, 0.000351174, 0.980042);
  }

  // The object that tracks the tunning, according to the twiddle logic.
  Twiddler twiddler {twiddle_steps, 0.0001, parameters, delta};

  // TODO: Maybe reduce the number of variables in the capture clause
  h.onMessage([&twiddle_mode,
              &steering_pid,
              &throttle_pid,
              &sum_cte_2,
              &twiddler,
              &avg_speed,
              &avg_throttle,
              &previous_angle,
              &kMaxThrottle](
              uWS::WebSocket<uWS::SERVER>* ws,
              char *data,
              size_t length,
              uWS::OpCode opCode) {
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
          //double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          // Since we are inducing a greater error by speeding up, factor in the
          // speed with the CTE.
          double steer_value = steering_pid.Correct(cte * (1 + speed / 100.0));
          // Here we use the full range of the PID from -1 to 1, but unlike the
          // steering, the direction of the error doesn't affect the type of
          // actuation (so we get only the absolute value). We also only want
          // the vehicle to use full throttle when the error is zero (we are on
          // track), so we are subtracting the result of the PID controller to
          // the maximum throttle we will allow. Finally, we combine the error
          // and the steering value: a small error and small steering angle mean
          // the car is on track and can speed up, if we either have to steer or
          // without steering get an increase in error, then we start to slow
          // down. We are effectively cascading the controllers.
          double throttle_value = kMaxThrottle - kMaxThrottle *
              std::abs(throttle_pid.Correct(steer_value * cte));

          // DEBUG
          //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          // Now, if we are in tunning mode, then we want to account the error
          // in a way that meets several objectives:
          // - Drive close to the expected trajectory (low CTE).
          // - Drive fast.
          // - Don't steer too hard.
          // - Use as much throttle as possible. This prevents from going into
          //   reverse.
          if (twiddle_mode) {
            avg_speed.Add(speed);
            avg_throttle.Add(throttle_value);
            sum_cte_2 += 2 * std::pow(cte, 2)
                + std::pow((100.0 - speed)/10, 2)
                + std::pow(steer_value - previous_angle, 2)
                + std::pow(kMaxThrottle - avg_throttle.average(), 2);
            previous_angle = steer_value;
          }

          if (twiddle_mode && twiddler.HasFinishedSteps()) {

            if (twiddler.IsInTolerance()) {
              // TODO: Write parameters to file
              std::cout << "Steering >> Kp: " << steering_pid.Kp_
                  << " Ki: " << steering_pid.Ki_
                  << " Kd: " << steering_pid.Kd_ << std::endl;
              std::cout << "Throttle >> Kp: " << throttle_pid.Kp_
                  << " Ki: " << throttle_pid.Ki_
                  << " Kd: " << throttle_pid.Kd_ << std::endl;
              // TODO: Stop simulation
              // For now just stop car
              json msgJson;
              msgJson["steering_angle"] = 0;
              msgJson["throttle"] = 0;
              ws->close();
            } else {
              std::cout <<
                 "Average speed: " << avg_speed.average() << std::endl <<
                 "------" << std::endl;
              // If we are tunning and have not yet reached the delta threshold,
              // "twiddle" the parameters and reset all values used to calculate
              // the error.
              std::vector<double> new_params = twiddler.Twiddle(sum_cte_2);
              steering_pid.Init(new_params[0], new_params[1], new_params[2]);
              throttle_pid.Init(new_params[3], new_params[4], new_params[5]);
              sum_cte_2 = 0.0;
              previous_angle = 0.0;
              avg_speed.Reset();
              avg_throttle.Reset();
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

#include <algorithm>
#include <iostream>
#include "PID.h"

using namespace std;
using Clock = chrono::high_resolution_clock;

PID::PID(double minimum_value, double maximum_value) :
    has_previous_frame_data_(false),
    Kp_(1.0),
    Ki_(1.0),
    Kd_(1.0),
    integral_cte_(0.0),
    output_min_(minimum_value),
    output_max_(maximum_value) {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;
  cout << "Kp: " << Kp << " Ki: " << Ki << " Kd: " << Kd << endl;
}

/*
void PID::UpdateError(double cte) {
}

double PID::TotalError() {
}
*/

double PID::Correct(double cte) {
  auto now = Clock::now();  // TODO: Define what to do if high_resolution_clock::is_steady != true
  double result = 0.0;
  if (has_previous_frame_data_) {
    double dt = chrono::duration_cast<chrono::milliseconds>(now - previous_timestamp_).count();
    double differential_cte = 0.0;
    if (dt > 0.0001) {
      differential_cte = (cte - previous_cte_) / dt / 1000;
    }
    result = -(Kp_ * cte + Ki_ * integral_cte_ + Kd_ * differential_cte);
  } else {
    has_previous_frame_data_ = true;
  }
  previous_timestamp_ = now;
  previous_cte_ = cte;
  return std::min(std::max(output_min_, result), output_max_);
}
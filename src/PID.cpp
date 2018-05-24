#include <algorithm>
#include <iostream>
#include "PID.h"

using namespace std;
using Clock = chrono::high_resolution_clock;

PID::PID(const double minimum_value, const double maximum_value) :
    Kp_(1.0),
    Ki_(1.0),
    Kd_(1.0),
    has_previous_frame_data_(false),
    integral_cte_(0.0),
    output_min_(minimum_value),
    output_max_(maximum_value) {}

PID::~PID() {}

void PID::Init(const double Kp, const double Ki, const double Kd) {
  has_previous_frame_data_ = false;
  integral_cte_ = 0.0;
  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;
  cout << "Kp: " << Kp << " Ki: " << Ki << " Kd: " << Kd << endl;
}

double PID::Correct(const double cte) {
  auto now = Clock::now();  // TODO: Define what to do if high_resolution_clock::is_steady != true
  integral_cte_ += cte;
  double result = -(Kp_ * cte + Ki_ * integral_cte_); // We can always at least apply the P and I terms
  if (has_previous_frame_data_) {
    double dt = chrono::duration_cast<chrono::milliseconds>(now - previous_timestamp_).count();
    double differential_cte = 0.0;
    if (dt > 0.0001) {
      differential_cte = (cte - previous_cte_) / dt / 1000;
    }
    result = result - Kd_ * differential_cte;
  } else {
    has_previous_frame_data_ = true;
  }
  previous_timestamp_ = now;
  previous_cte_ = cte;
  return std::min(std::max(output_min_, result), output_max_);
}
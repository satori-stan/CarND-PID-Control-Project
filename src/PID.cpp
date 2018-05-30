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
  // In case Init is called more than once, make sure the relevant variables are
  // reset.
  has_previous_frame_data_ = false;
  integral_cte_ = 0.0;
  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;
  cout << "Kp: " << Kp << " Ki: " << Ki << " Kd: " << Kd << endl;
}

double PID::Correct(const double cte) {
  // TODO: Define what to do if high_resolution_clock::is_steady != true
  auto now = Clock::now();
  // Calculate for the integral term.
  integral_cte_ += cte;
  // We can always at least apply the P and I terms, so we do that first.
  double result = -(Kp_ * cte + Ki_ * integral_cte_);
  // Then add the derivative term when it is available.
  if (has_previous_frame_data_) {
    double dt = chrono::duration_cast<chrono::milliseconds>(now - previous_timestamp_).count();
    double differential_cte = 0.0;
    if (dt > 0.0001) {
      // Measure the differential as change in error per second.
      differential_cte = (cte - previous_cte_) / dt / 1000;
    }
    result = result - Kd_ * differential_cte;
  } else {
    has_previous_frame_data_ = true;
  }
  // Set the lookback values for the next calculation.
  previous_timestamp_ = now;
  previous_cte_ = cte;
  // Return the output, but within the specified limits.
  return std::min(std::max(output_min_, result), output_max_);
}
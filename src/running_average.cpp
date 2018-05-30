#include "running_average.h"

RunningAverage::RunningAverage() : n_(0), average_(0.0) {}

RunningAverage::~RunningAverage() {}

double RunningAverage::Add(double new_value) {
  if (n_ == 0) {
    average_ = new_value;
    n_++;
  } else {
    int32_t n1 = n_ + 1;
    average_ = ((average_ * n_) + new_value) / n1;
    n_ = n1;
  }
  return average_;
}

double RunningAverage::average() {
  return average_;
}

void RunningAverage::Reset() {
  average_ = 0;
  n_ = 0;
}
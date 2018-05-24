#include <limits>
#include <numeric>
#include "twiddler.h"

Twiddler::Twiddler(int steps, double tolerance, std::vector<double> parameters,
                   std::vector<double> delta) :

                   target_steps_(steps),
                   executed_steps_(0),
                   tolerance_(tolerance),
                   parameters_(parameters),
                   delta_(delta),
                   best_error_(std::numeric_limits<double>::max()),
                   parameter_index_(0),
                   state_(0),
                   first_run_(true) {}
// TODO: Validate both parameters and delta are the same length

Twiddler::~Twiddler() {}

bool Twiddler::HasFinishedSteps() {
  return ++executed_steps_ >= target_steps_;
}

bool Twiddler::IsInTolerance() {
  return std::accumulate(delta_.begin(), delta_.end(), 0.0) <= tolerance_;
}

std::vector<double> Twiddler::Twiddle(double sum_error_squared) {
  double avg_error_squared = sum_error_squared / (2 * executed_steps_);
  switch (state_) {
    case 0:
      // Only the very first case lands here
      ApplyDelta();
      break;
    case 1:
      if (avg_error_squared < best_error_) {
        best_error_ = avg_error_squared;
        delta_[parameter_index_] *= kGOLDEN_RATIO_;
        NextParameter();
      }
      ApplyDelta();
      break;
    case 2:
      if (avg_error_squared < best_error_) {
        best_error_ = avg_error_squared;
        delta_[parameter_index_] *= kGOLDEN_RATIO_;
      } else {
        //parameters_[parameter_index_] += delta_[parameter_index_];
        ApplyDelta();
        delta_[parameter_index_] /= kGOLDEN_RATIO_;
      }
      NextParameter();
      ApplyDelta();
      break;
  }
  executed_steps_ = 0;
  return parameters_;
}

void Twiddler::ApplyDelta() {
  switch (state_) {
    case 0:
      parameters_[parameter_index_] += delta_[parameter_index_];
      state_++;
      break;
    case 1:
      parameters_[parameter_index_] -= 2 * delta_[parameter_index_];
      state_++;
      break;
    case 2:
      parameters_[parameter_index_] += delta_[parameter_index_];
      //delta_[parameter_index_] /= kGOLDEN_RATIO;
      state_ = 0;
      break;
  }
}
void Twiddler::NextParameter() {
  if (++parameter_index_ > (parameters_.size() - 1)) {
    parameter_index_ = 0;
  }
  state_ = 0;
}
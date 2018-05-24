#ifndef TWIDDLER_H_
#define TWIDDLER_H_

#include <vector>

class Twiddler {
 public:
  Twiddler(int steps, double tolerance, std::vector<double> parameters, std::vector<double> delta);
  virtual ~Twiddler();

  bool HasFinishedSteps();
  //void Step();
  bool IsInTolerance();
  std::vector<double> Twiddle(double sum_error_squared);

 private:
  const double kGOLDEN_RATIO_ =  	1.6180339887498948482;
  int target_steps_;
  int executed_steps_;
  double tolerance_;
  std::vector<double> parameters_;
  std::vector<double> delta_;
  double best_error_;
  unsigned int parameter_index_;
  unsigned int state_;
  bool first_run_;

  void ApplyDelta();
  void NextParameter();

};

#endif  // TWIDDLER_H_
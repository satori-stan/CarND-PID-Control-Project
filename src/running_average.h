#ifndef RUNNING_AVERAGE_H_
#define RUNNING_AVERAGE_H_

#include <cstdint>

class RunningAverage {
 public:
  
  RunningAverage();
  virtual ~RunningAverage();

  double Add(double new_value);

  double average();

 private:

  int32_t n_;
  double average_;

};

#endif  // RUNNING_AVERAGE_H_
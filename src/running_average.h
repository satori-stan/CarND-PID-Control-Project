#ifndef RUNNING_AVERAGE_H_
#define RUNNING_AVERAGE_H_

#include <cstdint>

/**
 * Keeps a running average calculation.
 */
class RunningAverage {
 public:

  RunningAverage();
  virtual ~RunningAverage();

  /**
   * Add a new value to the calculation of the average.
   * @param new_value The value to add
   * @returns The updated running average
   */
  double Add(double new_value);

  /**
   * Get the last value of running average calculated
   * @returns The stored running average
   */
  double average();

  /**
   * Reset the running average to zero.
   */
  void Reset();

 private:

  // The number of values so far added to the calculation
  int32_t n_;
  // The last value of the average
  double average_;

};

#endif  // RUNNING_AVERAGE_H_
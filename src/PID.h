#ifndef PID_H_
#define PID_H_

#include <chrono>
#include <cstdint>

class PID {
 public:
  /**
   * Coefficients
   */ 
  double Kp_;
  double Ki_;
  double Kd_;

  /**
   * Constructor
   */
  PID(const double minimum_value, const double maximum_value);

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   */
  void Init(const double Kp, const double Ki, const double Kd);

  /**
   * Calculate the controlled output
   */
  double Correct(const double cte);

 private:

  bool has_previous_frame_data_;

  // To calculate the differential
  double previous_cte_;
  std::chrono::time_point<std::chrono::high_resolution_clock> previous_timestamp_;

  // To calculate the integral
  double integral_cte_;

  // To limit the output
  double output_min_;
  double output_max_;
};

#endif /* PID_H_ */

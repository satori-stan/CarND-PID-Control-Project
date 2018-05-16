#ifndef PID_H_
#define PID_H_

#include <chrono>
#include <cstdint>

class PID {
 public:
  /**
   * Errors
   */
  double p_error_;
  double i_error_;
  double d_error_;

  /**
   * Coefficients
   */ 
  double Kp_;
  double Ki_;
  double Kd_;

  /**
   * Constructor
   */
  PID(double minimum_value, double maximum_value);

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   */
  void Init(double Kp, double Ki, double Kd);

  /**
   * Update the PID error variables given cross track error.
   *
  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   *
  double TotalError();
  */

  /**
   * Calculate the controlled output
   */
  double Correct(double cte);

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

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
   * @param minumum_value The minimum value to be emmited by the controller.
   * @param maximum_value The maximum value to be emmited by the controller.
   */
  PID(const double minimum_value, const double maximum_value);

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param Kp The coefficient for the proportional term.
   * @param Ki The coefficient for the integral term.
   * @param Kd The coefficient for the derivative term.
   */
  void Init(const double Kp, const double Ki, const double Kd);

  /**
   * Calculate the controlled output
   * @param cte The error to correct for.
   * @return The actuation value that corrects the error.
   */
  double Correct(const double cte);

 private:

  // To avoid calculating the derivative without enough information
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

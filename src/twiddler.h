#ifndef TWIDDLER_H_
#define TWIDDLER_H_

#include <vector>

/**
 * Implements the logic to fine tune a set of parameters.
 */
class Twiddler {
 public:
  /**
   * Constructor
   * @param steps The number of steps during which the error is accumulated.
   * @param tolerance The threshold under which parameter modifications are
   *        considered irrelevant, so the tunning is stopped.
   * @param parameters The vector of parameters to tune.
   * @param delta The vector of initial parameter delta values.
   */
  Twiddler(int steps, double tolerance, std::vector<double> parameters, std::vector<double> delta);
  virtual ~Twiddler();

  /**
   * Check if the number of accounting steps has elapsed.
   * WARNING!! Running this method will increase the number of already-run steps.
   * @returns Whether the desired number of accounting steps has already elapsed.
   */
  bool HasFinishedSteps();

  /**
   * Checks to see if it still makes sense to "twiddle" the paramter values.
   * @returns Whether the sum of deltas is under the tolerance.
   */
  bool IsInTolerance();

  /**
   * Get a new set of parameters to try given the error value from the previous run.
   * @param sum_error_squared The error value of this run.
   * @returns A new parameter vector with some deltas applied.
   */
  std::vector<double> Twiddle(double sum_error_squared);

 private:
  // The amount used to change the delta values
  const double kGOLDEN_RATIO_ =  	1.6180339887498948482;
  // The total number of steps to run before a new parameter set is needed
  int target_steps_;
  // The number of steps executed so far
  int executed_steps_;
  // The value under which, further modification of the parameters is considered irrelevant
  double tolerance_;
  // The parameter vector in its current form
  std::vector<double> parameters_;
  // The delta vector in its current form
  std::vector<double> delta_;
  // The lowest error value achieved
  double best_error_;
  // The index of the parameter we are currently "twiddling"
  unsigned int parameter_index_;
  // A status variable to know which operation we apply to the parameters
  unsigned int state_;

  /**
   * Function to modify the current parameter by its delta in the given state
   */
  void ApplyDelta();

  /**
   * Finds the next parameter index to modify
   * Basically loops the index around if we reach the end of the vector.
   */
  void NextParameter();

};

#endif  // TWIDDLER_H_
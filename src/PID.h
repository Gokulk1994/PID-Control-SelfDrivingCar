#ifndef PID_H
#define PID_H

#include <vector>
#include <string>

using std::vector;
using std::string;

class PID {
 public:
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();
  
  /**
  * Calculate the best gain values Kp, Kd, Ki using Twiddle algorithm explained in class
  * @output : No return; Updates the dp (delta in gain) parameters
  */
  
  void Twiddle(double cte);

 private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;
  
  
  vector<double> dp;
  vector<double> Gain_K;
  double tolerance;
  double best_err;
  int state;
  int twiddle_count;
  
};

#endif  // PID_H
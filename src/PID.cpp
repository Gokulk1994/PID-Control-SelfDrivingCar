#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

#include <vector>
#include <iostream>

using std::vector;


#define INIT           (int)0
#define INCREASE_CHECK (int)1
#define DECREASE_CHECK (int)2

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
 
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
  
  dp     = {1.0,1.0,1.0};
  
  Gain_K = {Kp_, Kd_, Ki_};
    
  tolerance = 0.2;
  best_err  = 2.0;
  state     = INIT;
  twiddle_count = 0;
  
  param_index = 0;

}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  
  if (twiddle_count == 0) p_error = cte;
  
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;

}



void PID::Twiddle(double cte) 
{
  double sum_delta = dp[0] + dp[1] + dp[2];
  
  if ((sum_delta > tolerance) & (twiddle_count < 5000))
  {   
      switch(state)
      {     
        case INIT:
          Gain_K[param_index] += dp[param_index];
          state = INCREASE_CHECK;
          break;
        
        case INCREASE_CHECK:
          if(cte < best_err)
          {
            best_err = cte;
            dp[param_index] *= 1.1;   
            param_index = (param_index + 1 ) % 3;
            state = INIT;
          }
          else
          {
          	Gain_K[param_index] -= 2 * dp[param_index];            
            state = DECREASE_CHECK;          
          }
          break;
           
        case DECREASE_CHECK:
          if(cte < best_err)
          {
            best_err = cte;
            dp[param_index] *= 1.1;                       
          }
          else
          {
          	Gain_K[param_index] += dp[param_index];            
            dp[param_index] *= 0.9;
          }
          state = INIT;
          param_index = (param_index + 1 ) % 3;
          break;
    
      }    
    twiddle_count += 1;      
  }
  else
  {
    std::cout<<"Twiddle Done : Sum of Delta - "<<sum_delta <<" & iterations - "<<twiddle_count<<std::endl;
  }
}


double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  
  double TotalErr = -1.0 * ((Gain_K[0] * p_error) + (Gain_K[1] * d_error) + (Gain_K[2] * i_error));
  std::cout<<twiddle_count<<" "<<TotalErr<<" "<<Gain_K[0] <<" "<<Gain_K[1] <<" "<<Gain_K[2] <<" | "<<p_error<< " "<<d_error<<" "<<i_error<<std::endl;
  if(TotalErr > 1.0)
    TotalErr = 1.0;
  
  if(TotalErr < -1.0)
    TotalErr = -1.0;
  
  return TotalErr; 
}
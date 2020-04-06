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
  Kp = Kp_;
  Kd = Kd_;
  Ki = Ki_;
  
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
  
  dp     = {1.0,1.0,1.0};
  
  Gain_K = {Kp, Kd, Ki};
    
  tolerance = 0.2;
  best_err  = 2.0;
  state     = INIT;
  twiddle_count = 0;

}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;

}



void PID::Twiddle(double cte) 
{
  //double sum_delta = dp[0] + dp[1] + dp[2];
  
  if ((dp[0] > tolerance) & (twiddle_count < 5000))
  {   
      std:: cout<<"state " <<state<<" ";
      switch(state)
      {     
        case INIT:
          Gain_K[0] += dp[0];
          Gain_K[1] += dp[1];
          Gain_K[2] += dp[2];  
          state      = INCREASE_CHECK;
          break;
        
        case INCREASE_CHECK:
          if(cte < best_err)
          {
            best_err = cte;
            dp[0] *= 1.1;
            dp[1] *= 1.1;
            dp[2] *= 1.1;
            
            state = INIT;
          }
          else
          {
          	Gain_K[0] -= 2 * dp[0];
          	Gain_K[1] -= 2 * dp[1];
          	Gain_K[2] -= 2 * dp[2]; 
            
            state = DECREASE_CHECK;
          
          }
          break;
           
        case DECREASE_CHECK:
          if(cte < best_err)
          {
            best_err = cte;
            dp[0] *= 1.1;
            dp[1] *= 1.1;
            dp[2] *= 1.1;                       
          }
          else
          {
          	Gain_K[0] += dp[0];
          	Gain_K[1] += dp[1];
          	Gain_K[2] += dp[2];
            
            dp[0] *= 0.9;
            dp[1] *= 0.9;
            dp[2] *= 0.9;
          }
          state = INIT;
          break;
    
      }    
    twiddle_count += 1;      
  }
  else
  {
    std::cout<<"out============="<<std::endl;
  }
}


double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  
  //double TotalErr = -1.0 * ((Gain_K[0] * p_error) + (Gain_K[1] * d_error) + (Gain_K[2] * i_error));
  double TotalErr = -1.0 * ((Gain_K[0] * p_error));
  std::cout<<TotalErr<<" "<<Gain_K[0] <<" "<<Gain_K[1] <<" "<<Gain_K[2] <<" "<<p_error<< " "<<d_error<<" "<<i_error<<std::endl;
  if(TotalErr > 1.0)
    TotalErr = 1.0;
  
  if(TotalErr < -1.0)
    TotalErr = -1.0;
  
  return TotalErr; 
}
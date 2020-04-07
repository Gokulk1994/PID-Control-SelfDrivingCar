#include "PID.h"

PID::PID() {}

PID::~PID() {}

#include <vector>
#include <iostream>

using std::vector;

#define INIT           (int)0    // Initial state of Twiddle
#define INCREASE_CHECK (int)1    // Increase the values of gain parameters state
#define DECREASE_CHECK (int)2    // Decrease the values of gain parameters state

/*
Initialize default values to all the parameters
*/

void PID::Init(double Kp_, double Ki_, double Kd_) {
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
  
  dp     = {0.1*Kp_, 0.1*Kd_, 0.1*Ki_};  // 10% of gain parameters is considered as upate value 
  //dp = {0.004,0.15,0.0001};
  Gain_K = {Kp_, Kd_, Ki_};   // Vector hold gain values ; Kp, Kd, Ki values are replaced with this vector variables
    
  tolerance = 0.05;  // tolerance for twiddle algorithm to terminate
  best_err  = 2.0; 
  state     = INIT;  // Initial state of Twiddle
  twiddle_count = 0;
  
  param_index = 0;  
  Twiddle_status = false;

}

/*
Updates the error during each cycle
Activates twiddle algorithm to find optimal gain values
*/

void PID::UpdateError(double cte) {
  
  d_error = cte - p_error;  // Derivative error
  p_error = cte;            // propotional error
  i_error += cte;           // Integral error
  
  // If twiddle algorithm is enabled to find tha otimal parameters , algorithm is enabled
  if(Twiddle_status == true)
  	Twiddle(cte); 

}

/*
Function to handle twiddle algorithm
Fidns the optimal gain parameters
*/

void PID::Twiddle(double cte) 
{
  double sum_delta = dp[0] + dp[1] + dp[2]; // Sum up all the delta values  
  
  if ((sum_delta > tolerance) & (twiddle_count < 2000))  // Terminate if updates are comparatively smaller or iterations are more than 2000 
  {   
      switch(state)
      {     
        case INIT:   // Initial state; just adds delta value
          Gain_K[param_index] += dp[param_index];    
          state = INCREASE_CHECK;
          break;
        
        case INCREASE_CHECK:  // Check if the increment done leads to better cte value
          if(cte < best_err)
          {
            best_err = cte;
            dp[param_index] *= 1.1; // update delta values
            param_index = (param_index + 1 ) % 3; // move to next parameter
            state = INIT;  // reset to initial state
          }
          else
          {
          	Gain_K[param_index] -= 2 * dp[param_index]; // Id incremente does not works, decrement the pararmeters
            state = DECREASE_CHECK;          
          }
          break;
           
        case DECREASE_CHECK:
          if(cte < best_err)  // check if decrement yield better cte values
          {
            best_err = cte;
            dp[param_index] *= 1.1;   // update delta values                    
          }
          else
          {
          	Gain_K[param_index] += dp[param_index];   // If nothing works remove all update and decrease delta values         
            dp[param_index] *= 0.9;
          }
          state = INIT;
          param_index = (param_index + 1 ) % 3;
          break;
    
      }    
      
    twiddle_count += 1;      
    //std::cout<<twiddle_count<<" "<<Gain_K[0]<<" "<<Gain_K[1]<<" "<<Gain_K[2]<<" | "<<p_error<< " "<<d_error<<" "<<i_error<<std::endl;
  }
  else // Display Optimal values after algorithm terminates. Use this value to udpate the gains 
  {
    std::cout<<"Twiddle Done : Sum of Delta - "<<sum_delta <<" & iterations - "<<twiddle_count<<std::endl;
    std::cout<<"Final Kp Ki Kd after twiddle {"<<Gain_K[0]<<","<<Gain_K[2]<<","<<Gain_K[1]<<"};"<<std::endl;
    Twiddle_status = false;
  }
  
}


/*
calcaulte the total error based on Kp,Kd,Ki and all 3 error values
Total error should be within the range [-1,+1]
*/

double PID::TotalError() {
      
  
  // calculate the total error 
  double TotalErr = -1.0 * ((Gain_K[0] * p_error) + (Gain_K[1] * d_error) + (Gain_K[2] * i_error));
  
  // Limit error to 1.0
  if(TotalErr > 1.0)
    TotalErr = 1.0;
  
  // Limit error to -1.0
  if(TotalErr < -1.0)
    TotalErr = -1.0;
  
  
  return TotalErr; 
}
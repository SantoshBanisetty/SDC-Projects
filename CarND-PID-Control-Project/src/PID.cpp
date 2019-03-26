#include "PID.h"
#include <iostream>
using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  //Initialize gains
  std::cout << Kp << Ki << Kd << std::endl;
  K_p = Kp;
  K_d = Kd;
  K_i = Ki;
  std::cout << K_p << K_i << K_d << std::endl;
  p_error = 0;
  i_error = 0;
  d_error = 0;
}

void PID::UpdateError(double cte) {
  //Use error (cte) to compute the required variables for PID
  
  d_error = cte - p_error; // t - t-1
  i_error = i_error + cte; // Error over time (sum)
  p_error = cte; // Error at time t
}

double PID::TotalError() {
  std::cout << K_p << K_i << K_d << std::endl;
  return -((K_p * p_error) + (K_d * d_error) + (K_i * i_error));
}


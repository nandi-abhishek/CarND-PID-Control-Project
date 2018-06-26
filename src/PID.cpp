#include <iostream>
#include <math.h>
#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  p_error = 0; 
  i_error = 0; 
  d_error = 0; 
  total_error = 0;
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  iter = 0;
}

void PID::UpdateError(double cte) {
	d_error = cte - p_error;
	p_error = cte;
	i_error += cte;
}

double PID::TotalError() {
	total_error += p_error * p_error;
	iter++;
	std::cout << "ITER: " << iter << " RMSE: " << sqrt(total_error / iter) << std::endl;
	double angle = -Kp * p_error - Ki * i_error - Kd * d_error;
	return angle;
}


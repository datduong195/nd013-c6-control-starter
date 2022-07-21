/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <math.h>
#include <iostream>
#include <vector>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, double output_lim_max,
               double output_lim_min) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   **/
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  this->output_lim_max = output_lim_max;
  this->output_lim_min = output_lim_min;
  error_sum = 0.0;
  prev_cte = 0.0;
  delta_time = 1;
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   **/
  diff_cte = cte - prev_cte;
  error_sum += cte * delta_time;
  prev_cte = cte;
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   * The code should return a value in the interval [output_lim_min,
   * output_lim_max]
   */
  double der_err = 0;
  if (delta_time == 0) {
    der_err = 0;
  } else {
    der_err = Kd * diff_cte / delta_time;
  }
  double steer_control = Kp * prev_cte + der_err + Ki * error_sum;

  // Keep output within bounds
  if (steer_control > output_lim_max) {
    steer_control = output_lim_max;
  } else if (steer_control < output_lim_min) {
    steer_control = output_lim_min;
  }
  return steer_control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
  /**
   * TODO: Update the delta time with new value
   */
  delta_time = new_delta_time;
}
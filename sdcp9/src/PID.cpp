/*
##############################################
## AUTHOR: James Beasley                    ##
## DATE: July 23, 2017                      ##
## UDACITY SDC: Project 9 (PID Controllers) ##
##############################################
*/

#include "PID.h"

using namespace std;

//constructor
PID::PID() {}

//destructor
PID::~PID() {}

//set control gain for each term
void PID::Init(double Kp, double Ki, double Kd)
{
    //set gains
    this->Kp = Kp; //set proportional gain
    this->Ki = Ki; //set integral gain
    this->Kd = Kd; //set derivative gain

    //zero out error terms
    p_error = 0;
    i_error = 0;
    d_error = 0;
}

//compute error values to be used in steering computation later
void PID::UpdateError(double cte)
{
    //the error for the derivative term will be the difference between the current cross track error and the previous cross track error (from the last time step)
    //p_error contains our previous cte so by calculating derivative error first we don't need to track previous cte as a separate variable
    d_error = cte - p_error;
    //the error for the proportional term will be the cross track error (distance from y position to our target trajectory)
    p_error = cte;
    //the error for the integral term will be the sum of the cross track errors over time
    i_error += cte;
}

//compute the steering angle (alpha) using the proportional, integral, and derivative control laws
double PID::TotalError()
{
    //alpha = (-tau_p * cte) - (tau_d * diff_cte) - (tau_i * total_cte)
    return (-Kp * p_error) - (Kd * d_error) - (Ki * i_error);
}

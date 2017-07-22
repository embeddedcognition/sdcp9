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

    //zero errors
    p_error = 0;
    i_error = 0;
    d_error = 0;
}


void PID::UpdateError(double cte)
{
    d_error = cte - p_error; //p_error contains our previous cte so by calculating derivative term first we don't need to track previous cte as a separate variable
    p_error = cte;
    i_error += cte;
}


double PID::TotalError()
{
    //(-tau_p * cte) - (tau_d * diff_cte) - (tau_i * total_cte)
    return (-Kp * p_error) - (Kd * d_error) - (Ki * i_error);
}


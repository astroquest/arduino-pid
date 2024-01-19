#include "Arduino.h"
#include <Pid.h>

Pid::Pid(double cycle_time_ms, double kp, double ki, double kd, double taud, double limit_min, double limit_max) {
    this->cycle_time = cycle_time_ms/1000;
    this->limit_min = limit_min;
    this->limit_max = limit_max;

    setGains(kp, ki, kd, taud);
    setCoefficients();
    resetController();
}

void Pid::setGains(double kp, double ki, double kd, double taud) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->taud = taud/1000;
}

void Pid::resetController() {
    prev_process_output = 0;
    prev_error = 0;
    prev_integral_action = 0;
    prev_derivative_action = 0;
}

void Pid::setCoefficients() {
    b_int = ki * cycle_time / 2;

    double div_der = 1 / (2 * taud + cycle_time);
    a_der = (2 * taud - cycle_time) * div_der;
    b_der = - 2 * kd * div_der;
}

double Pid::getAction(double setpoint, double process_output) {
    double error = setpoint - process_output;

    double integral_action = getIntegralAction(error);
    double derivative_action = getDerivativeAction(process_output);

    action = kp * error + integral_action + derivative_action;
    saturated_action = checkSaturation(action);

    prev_process_output = process_output;
    prev_error = error;
    prev_integral_action = integral_action;
    prev_derivative_action = derivative_action;

    return saturated_action;
}

double Pid::getIntegralAction(double error){
    if ((saturated_action != action) && (action * error >= 0)) {
        return prev_integral_action;  // integral clamping
    }
    else {
        return checkSaturation(prev_integral_action + b_int * (error + prev_error));
    }
}

double Pid::getDerivativeAction(double process_output){
    return a_der * prev_derivative_action + b_der * (process_output - prev_process_output);
}

double Pid::checkSaturation(double action){
    if (action > limit_max) {
        return limit_max;
    }
    else if (action < limit_min) {
        return limit_min;
    }
    else {
        return action;
    }
}
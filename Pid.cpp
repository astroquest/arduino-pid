#include <Pid.h>

Pid::Pid(double cycle_time_ms, double kp, double ki, double kd, double taud, double limit_min, double limit_max) {
    this->cycle_time = cycle_time_ms/1000;
    this->limit_min = limit_min;
    this->limit_max = limit_max;

    prev_process_output = 0;
    prev_error = 0;
    prev_integral_action = 0;
    prev_derivative_action = 0;

    setGains(kp, ki, kd, taud);
}

void Pid::setGains(double kp, double ki, double kd, double taud) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
    this->taud = taud;
}

double Pid::getAction(double setpoint, double process_output) {
    error = setpoint - process_output;

    integral_action = getIntegralAction(error);
    derivative_action = getDerivativeAction(process_output);

    action = kp + integral_action + derivative_action;

    prev_process_output = process_output;
    prev_error = error;
    prev_integral_action = integral_action;
    prev_derivative_action = derivative_action;

    return checkSaturation(action, limit_min, limit_max);
}

double Pid::getIntegralAction(double error){
    return prev_integral_action + ki * cycle_time / 2 * (error + prev_error);
}

double Pid::getDerivativeAction(double process_output){
    return ((2 * taud - cycle_time) * prev_derivative_action
            - 2 * kd * (process_output - prev_process_output)) / (2 * taud + cycle_time);
}

double Pid::checkSaturation(double action, double limit_min, double limit_max){
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

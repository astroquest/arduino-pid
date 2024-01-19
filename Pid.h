#ifndef PID_H
#define PID_H

class Pid {
    public:
        Pid(double, double, double, double, double, double, double);

        void setGains(double, double, double, double);

        double getAction(double, double);

        double kp;
        double ki;
        double kd;
        double taud;

    private:
        double getIntegralAction(double);
        double getDerivativeAction(double);
        double checkSaturation(double, double, double);

        double cycle_time;

        double process_output;
        double error;
        double integral_action;
        double derivative_action;
        double action;

        double prev_process_output;
        double prev_error;
        double prev_integral_action;
        double prev_derivative_action;

        double limit_min;
        double limit_max;
};

#endif
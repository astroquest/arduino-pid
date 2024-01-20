#ifndef PID_H
#define PID_H

class Pid {
    public:
        Pid(double, double, double, double, double, double, double);

        void setGains(double, double, double, double);
        void setLimits(double, double);
        void resetController();

        double getAction(double, double);

        double kp;
        double ki;
        double kd;
        double taud;

    private:
        void setCoefficients();

        double getIntegralAction(double);
        double getDerivativeAction(double);
        double checkSaturation(double);

        double cycle_time;

        double b_int;
        double a_der;
        double b_der;

        double action;
        double saturated_action;

        double prev_process_output;
        double prev_error;
        double prev_integral_action;
        double prev_derivative_action;
        
        double limit_min;
        double limit_max;
};

#endif

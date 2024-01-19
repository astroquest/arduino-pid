#include <Pid.h>

double cycle_time = 1000;
double kp = 0.1;
double ki = 0.01;
double kd = 1;
double taud = 1000;
double limit_min = -1;
double limit_max = 1;

Pid pid(cycle_time, kp, ki, kd, taud, limit_min, limit_max);

void setup() {
  Serial.begin(9600);
}

double process_output = 0;
double setpoint = 1;

void loop() {
  double action = pid.getAction(setpoint, process_output);
  process_output = 0.9 * process_output + action;
  
  Serial.print(setpoint);
  Serial.print(", ");
  Serial.println(process_output);

  delay(1000);
}

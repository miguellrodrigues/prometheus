//
// Created by miguel on 01/03/25.
//

#include <pid.h>

PID_t *initPID(double kp, double ki, double kd, double tf, double ksi, double dt, double satUp, double satDown) {
    PID_t *pid = (PID_t *) malloc(sizeof(PID_t));

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->tf = tf;
    pid->ksi = ksi;

    pid->dt = dt;
    pid->satUp = satUp;
    pid->satDown = satDown;

    pid->error = 0;
    pid->pre_error = 0;
    pid->pre_ud = 0;
    pid->integral = 0;

    return pid;
}

double clip(double n, double lower, double upper) {
    return fmax(lower, fmin(n, upper));
}

double updatePID(PID_t *pid, double setpoint, double y) {
    double error = setpoint - y;

    pid->pre_error = pid->error;
    pid->error = error;

    double u_proportional = pid->kp * error;

    double u_derivative =
            (1 / pid->tf) * (pid->kd * (pid->error - pid->pre_error) - pid->pre_ud * pid->dt) + pid->pre_ud;
    pid->pre_ud = u_derivative;

    double u_integral = pid->integral;

    double computedU = u_proportional + u_derivative + u_integral;
    double satU = clip(computedU, pid->satDown, pid->satUp);

    double trapErr = 0.5 * pid->ki * (pid->error + pid->pre_error);
    double antiWindup = (1 / pid->ksi) * (satU - computedU);

    bool isInLinearRegion = (computedU >= pid->satDown) && (computedU <= pid->satUp);

    pid->integral += (isInLinearRegion ? trapErr : trapErr + antiWindup) * pid->dt;

    return satU;
}

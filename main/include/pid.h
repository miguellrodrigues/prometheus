#include <stdlib.h>
#include <math.h>
#include <stdbool.h>


// PID controller
typedef struct {
    double kp;
    double ki;
    double kd;
    double tf;
    double ksi;

    double dt;
    double satUp;
    double satDown;

    double error;
    double pre_error;
    double pre_ud;

    double integral;
} PID_t;


PID_t *initPID(double kp, double ki, double kd, double tf, double ksi, double dt, double satUp, double satDown);

double clip(double n, double lower, double upper);

double updatePID(PID_t *pid, double setpoint, double y);
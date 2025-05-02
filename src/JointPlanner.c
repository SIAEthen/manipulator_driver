#include "JointPlanner.h"



int calQuinticPolyNominal(double q0,double qt,double v0,double vt,double ac0,double act,double t, double* coeff){
    double a0 = coeff[0];
    double a1 = coeff[1];
    double a2 = coeff[2];
    double a3 = coeff[3];
    double a4 = coeff[4];
    double a5 = coeff[5];

    coeff[0] = q0;
    coeff[1] = v0;
    coeff[2] = ac0/2;
    double t2 = t*t;
    double t3 = t*t2;
    double t4 = t3*t;
    double t5 = t4*t;
    coeff[3] = (20*qt - 20*q0 - (8*vt+12*v0)*t - (3*ac0-act)*t2) /2/t3;
    coeff[4] = (30*q0-30*qt + (14*vt+16*v0)*t + (3*ac0-2*act)*t2) /2/t4;
    coeff[5] = (12*qt-12*q0 - (6*vt+6*v0)*t - (ac0-act)*t2) /2/t5;
    return 1;
}


double calQuinticPolyNominalTrajectory(double q0, double t, double* coeff){
    double a0 = coeff[0];
    double a1 = coeff[1];
    double a2 = coeff[2];
    double a3 = coeff[3];
    double a4 = coeff[4];
    double a5 = coeff[5];
    return q0 + a0 + a1*t + a2*t*t + a3*t*t*t + a4*t*t*t*t + a5*t*t*t*t*t;
}
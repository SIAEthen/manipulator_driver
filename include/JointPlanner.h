#ifndef _JOINT_PLANNER_H_
#define _JOINT_PLANNER_H_


int calQuinticPolyNominal(double q0,double qt,double v0,double vt,double ac0,double act,double t, 
                                double* coeff);
double calQuinticPolyNominalTrajectory(double q0, double t, double* coeff);                                


#endif
#ifndef __KINEMATICS_CONTROLLER__H__
#define __KINEMATICS_CONTROLLER__H__
#include <cmath>
#include "defines.h"


class PosKinController
{

public:
    PosKinController(double *kp,double *acc,double *initPos,double runningRate,double *maxVel,double *upperLimits, double *lowerLimits);
    ~PosKinController(){};
    
    int updateJointPositionCmd(double *desPosition,double *curPosition,double *cmdPosition);

private:
    double m_Kp[6];
    double m_acc[6];
    double m_PosCmd[6];
    double m_VelCmd[6];
    double m_maxVel[6];
    double m_jointUpperLimit[6];
    double m_jointLowerLimit[6];
    double m_runningRate;
};

#endif
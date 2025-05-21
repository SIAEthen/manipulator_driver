#ifndef __KINEMATICS_CONTROLLER__H__
#define __KINEMATICS_CONTROLLER__H__
#include <cmath>
#include "defines.h"
#ifndef JointNum
#define JointNum 6
#endif


class PosKinController
{

public:
    PosKinController(double *kp,double *acc,double *initPos,double runningRate,double *maxVel,double *upperLimits, double *lowerLimits);
    ~PosKinController(){};
    
    int updateJointPositionCmd(double *desPosition,double *curPosition,double *cmdPosition);

private:
    double m_Kp[JointNum];
    double m_acc[JointNum];
    double m_PosCmd[JointNum];
    double m_VelCmd[JointNum];
    double m_maxVel[JointNum];
    double m_jointUpperLimit[JointNum];
    double m_jointLowerLimit[JointNum];
    double m_runningRate;
};

#endif
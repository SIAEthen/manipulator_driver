#ifndef MANIPULATORPLANNER_H
#define MANIPULATORPLANNER_H
#include "ManipulatorInterface.h"
#include "defines.h"
#include "JointPlanner.h"

typedef enum{
    ManipulatorMode_Disable = 1,
    ManipulatorMode_AutoVel,
    ManipulatorMode_AutoPos,
    ManipulatorMode_P2P
}ManipulatorModeTypeDef;

class ManipulatorPlanner
{   friend class ManipulatorDriver;
    public:
        ManipulatorPlanner(ManipulatorInterface& manipulator);
        ~ManipulatorPlanner() {} ;
        
        void AutoVelModeCmdInit();
        void AutoVelModeInit(); 
        void AutoVelModeRun(); 
        void AutoPosVelModeInit(); 
        void AutoPosModeRun(); 
        void P2PModeInit(); 
        void P2PModeRun(); 
        void run(); 
        
        void setDesiredVelocity(double* dq);
        void getCmdVelocity();

    private:

        ManipulatorInterface& _manipulator;
        ManipulatorModeTypeDef m_Mode;
        double m_posCmd[6] = {0};
        double m_maxVelocity[6] = {0.3,0.3,0.5, 0.5,0.5,0.5};
        double m_acc[6] = {0.3,0.3,0.5, 0.5,0.5,0.5};
    protected:
        double m_velCmd[6] = {0};
        double m_velDes[6] = {0};


};




#endif // MANIPULATORDRIVER_H
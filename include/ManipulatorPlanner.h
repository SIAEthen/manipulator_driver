#ifndef MANIPULATORPLANNER_H
#define MANIPULATORPLANNER_H
#include "ManipulatorInterface.h"
#include "defines.h"
#include "JointPlanner.h"
#include "kinematicsController.h"



class ManipulatorPlanner
{   friend class ManipulatorDriver;
    public:
        ManipulatorPlanner(ManipulatorInterface& manipulator);
        ~ManipulatorPlanner() {
            if(m_Mode==ManipulatorMode_IP_Pos)
            delete m_posController;
            } ;
        
        void IPVelModeCmdInit();
        void IPVelModeInit(); 
        void IPVelModeRun(); 
        void IPPosModeInit(); 
        void IPPosModeRun(); 
        void P2PModeInit(); 
        void P2PModeRun(); 
        void PVModeRun();
        void PVModeInit(); 
        void run(); 
        
        void setDesiredVelocity(double* dq);
        void getCmdVelocity();
        void setDesiredPosition(double* q);

    private:
        ManipulatorInterface& _manipulator;
        ManipulatorModeTypeDef m_Mode = ManipulatorMode_Disable;
        PosKinController *m_posController = nullptr;
        double m_maxVelocity[6] = {MaxVel1,MaxVel2,MaxVel3,MaxVel4,MaxVel5,MaxVel6};
        double m_acc[6] = {KinAcc1,KinAcc2,KinAcc3,KinAcc4,KinAcc5,KinAcc6};
    
    
    private:
        //pos mode variables
        double m_posModePositionDes[6] = {0};
    private:
        //velocity mode variables
        double m_velDes[6] = {0};
    protected:
        double m_velCmd[6] = {0};
        double m_posCmd[6] = {0};
};




#endif // MANIPULATORDRIVER_H

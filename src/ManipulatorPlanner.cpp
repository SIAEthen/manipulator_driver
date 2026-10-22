#include "ManipulatorPlanner.h"
#include <stdexcept>

ManipulatorPlanner::ManipulatorPlanner(ManipulatorInterface& manipulator) :
_manipulator(manipulator)
{
    
    
}
void ManipulatorPlanner::IPVelModeCmdInit(){
    for(unsigned char i=0;i<_manipulator.m_jointnum;i++){
        m_posCmd[i] = _manipulator.m_jointposition[i];
    }
}
void ManipulatorPlanner::IPVelModeInit(){
    IPVelModeCmdInit();
}
void ManipulatorPlanner::IPVelModeRun(){
    // 按加速度约束平滑速度，再按 dt = 1 / 控制频率积分为位置（rad）。
    getCmdVelocity();
    const double dt = 1.0 / _manipulator.m_controlfrequency;
    for (uint8_t i = 0; i < _manipulator.m_jointnum; ++i) {
        m_posCmd[i] += m_velCmd[i] * dt;
    }
    _manipulator.sendPosCmd(m_posCmd);
}

void ManipulatorPlanner::IPPosModeInit(){
    double kp[6] = {KinKp1,KinKp2,KinKp3,KinKp4,KinKp5,KinKp6};
    m_posController = PosKinController(kp,m_acc,_manipulator.m_jointposition,_manipulator.m_controlfrequency,m_maxVelocity,
                                            _manipulator.m_jointUpperLimits,_manipulator.m_jointLowerLimits);
    for(unsigned char i=0;i<ManipulatorState.NumOfJoint;i++){
        m_posCmd[i] = _manipulator.m_jointposition[i];
        m_posModePositionDes[i] = _manipulator.m_jointposition[i];
    }
} 
void ManipulatorPlanner::IPPosModeRun(){
    m_posController.updateJointPositionCmd(m_posModePositionDes,_manipulator.m_jointposition,m_posCmd);
    _manipulator.sendPosCmd(m_posCmd);
    return;
} 

void ManipulatorPlanner::P2PModeInit(){
} 
void ManipulatorPlanner::P2PModeRun(){

}

void ManipulatorPlanner::PVModeRun(){
    _manipulator.sendPVCmdPDO(m_velDes);
}
void ManipulatorPlanner::PVModeInit(){
    for (double &velocity : m_velDes) {
        velocity = 0.0;
    }
}
void ManipulatorPlanner::init(){
    switch (MANIPULATOR_MODE_DEFINE)
    {
    case ManipulatorModeTypeDef::ManipulatorMode_IP_Vel:
        IPVelModeInit();
        return;
    case ManipulatorModeTypeDef::ManipulatorMode_IP_Pos:
        IPPosModeInit();
        return;
    case ManipulatorModeTypeDef::ManipulatorMode_IP_P2P:
        /* code */
        P2PModeInit();
        break;
    case ManipulatorModeTypeDef::ManipulatorMode_PV:
        PVModeInit();
        return;
    default:
      break;
    }

}
        
void ManipulatorPlanner::run(){
    switch (MANIPULATOR_MODE_DEFINE)
    {
    case ManipulatorModeTypeDef::ManipulatorMode_IP_Vel:
        /* code */
        IPVelModeRun();
        break;
    case ManipulatorModeTypeDef::ManipulatorMode_IP_Pos:
        /* code */
        IPPosModeRun();
        break;
    case ManipulatorModeTypeDef::ManipulatorMode_IP_P2P:
        /* code */
        P2PModeRun();
        break;
    case ManipulatorModeTypeDef::ManipulatorMode_PV:
        /* code */
        PVModeRun();
        break;
    case ManipulatorMode_Disable:
        throw std::runtime_error("Manipulator planner is disabled; initialize a planning mode before calling run().");
    default:
        break;
    }
}

//设置期望速度
void ManipulatorPlanner::setDesiredVelocity(double* dq){
    //对速度进行限制
    for(unsigned char i=0;i<_manipulator.m_jointnum;i++){
        if(fabs(dq[i])>m_maxVelocity[i]){
            m_velDes[i] = fabs(dq[i])/dq[i] * m_maxVelocity[i];
        }else{
             m_velDes[i] = dq[i];
        }
    }
}

void ManipulatorPlanner::setDesiredPosition(double* q){

    //对速度进行限制
    for(unsigned char i=0;i<ManipulatorState.NumOfJoint;i++){
        m_posModePositionDes[i] = q[i];
    }
}

//根据期望速度与当前速度命令，结合设定的加速度值，给出当前指令应该有的速度
void ManipulatorPlanner::getCmdVelocity(){
    //对速度进行限制
    for(unsigned char i=0;i<_manipulator.m_jointnum;i++){
        double delta_vel = m_velDes[i] - m_velCmd[i];
        double cmd = 0;
        if(delta_vel>0){
            // speed up
            cmd = m_velCmd[i] + m_acc[i]/_manipulator.m_controlfrequency; //全加速
            if(cmd>m_velDes[i]) cmd = m_velDes[i]; //在这个周期如果加速加过头，那就直接把Des当成cmd
        }else{
            // slow down
            // speed up
            cmd = m_velCmd[i] - m_acc[i]/_manipulator.m_controlfrequency; //全加速
            if(cmd<m_velDes[i]) cmd = m_velDes[i];
        }
        m_velCmd[i] = cmd;
    }
}

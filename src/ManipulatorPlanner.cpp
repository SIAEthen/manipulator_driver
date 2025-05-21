#include "ManipulatorPlanner.h"

ManipulatorPlanner::ManipulatorPlanner(ManipulatorInterface& manipulator) :
_manipulator(manipulator)
{
    
    
}
void ManipulatorPlanner::AutoVelModeCmdInit(){
    for(unsigned char i=0;i<_manipulator.m_jointnum;i++){
        m_posCmd[i] = _manipulator.m_jointposition[i];
    }
}
void ManipulatorPlanner::AutoVelModeInit(){
    m_Mode = ManipulatorMode_AutoVel;
    AutoVelModeCmdInit();
}
void ManipulatorPlanner::AutoVelModeRun(){
    //根据期望速度，和当前指令，以及加速度，给出一个缓和的速度
    getCmdVelocity();
    //速度积分得到位置
    for(unsigned char i=0;i<_manipulator.m_jointnum;i++){
        m_posCmd[i] = m_posCmd[i] + m_velCmd[i]/_manipulator.m_controlfrequency;
    }
    //根据位置限制，对位置进行限位
    for(unsigned char i=0;i<_manipulator.m_jointnum;i++){
        //第六关节连续旋转，因此将其映射到-pi ~pi
        if(i==5){
            if(m_posCmd[i]<_manipulator.m_jointLowerLimits[i]) m_posCmd[i] += 2 * PI;
            if(m_posCmd[i]>_manipulator.m_jointUpperLimits[i]) m_posCmd[i] -= 2 * PI;
        }else{
            // 1-5关节不连续，因此就在自己限位内就OK
            if(m_posCmd[i]<_manipulator.m_jointLowerLimits[i]) m_posCmd[i] = _manipulator.m_jointLowerLimits[i];
            if(m_posCmd[i]>_manipulator.m_jointUpperLimits[i]) m_posCmd[i] = _manipulator.m_jointUpperLimits[i];
        }
    }
    
    _manipulator.sendPosCmd(m_posCmd); 
    return;
} 

void ManipulatorPlanner::AutoPosModeInit(){
    m_Mode = ManipulatorMode_AutoPos;
    double kp[6] = {1,1,1,1,1,1};
    m_posController = new PosKinController(kp,m_acc,_manipulator.m_jointposition,_manipulator.m_controlfrequency,m_maxVelocity,
                                            _manipulator.m_jointUpperLimits,_manipulator.m_jointLowerLimits);
    for(unsigned char i=0;i<JointNum;i++){
        m_posCmd[i] = _manipulator.m_jointposition[i];
        m_posModePositionDes[i] = _manipulator.m_jointposition[i];
    }
} 
void ManipulatorPlanner::AutoPosModeRun(){
    m_posController->updateJointPositionCmd(m_posModePositionDes,_manipulator.m_jointposition,m_posCmd);
    _manipulator.sendPosCmd(m_posCmd); 
    return;
} 

void ManipulatorPlanner::P2PModeInit(){
    m_Mode = ManipulatorMode_P2P;
} 
void ManipulatorPlanner::P2PModeRun(){

}
        
void ManipulatorPlanner::run(){
    switch (m_Mode)
    {
    case ManipulatorMode_AutoVel:
        /* code */
        AutoVelModeRun();
        break;
    case ManipulatorMode_AutoPos:
        /* code */
        AutoPosModeRun();
        break;
    case ManipulatorMode_P2P:
        /* code */
        P2PModeRun();
        break;
    case ManipulatorMode_Disable:
        /* code */
        
        break;
    default:
        break;
    }
}

//设置期望速度
void ManipulatorPlanner::setDesiredVelocity(double* dq){
    //对速度进行限制
    for(unsigned char i=0;i<_manipulator.m_jointnum;i++){
        if(fabs(dq[i]>m_maxVelocity[i])){
            m_velDes[i] = fabs(dq[i])/dq[i] * m_maxVelocity[i];
        }else{
             m_velDes[i] = dq[i];
        }
    }
}

void ManipulatorPlanner::setDesiredPosition(double* q){

    //对速度进行限制
    for(unsigned char i=0;i<JointNum;i++){
        m_posModePositionDes[i] = q[i];
    }
}

//根据期望速度与当前命令的速度，结合设定的加速度值，给出当前指令应该有的速度
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
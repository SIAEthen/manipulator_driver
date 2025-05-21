#include "kinematicsController.h"

// PosKinController::PosKinController

PosKinController::PosKinController(double *kp,double *acc,double *initPos,double runningRate,
                                            double *maxVel,double *upperLimits, double *lowerLimits){
    for(unsigned char i=0;i<JointNum;i++){
        m_Kp[i] = kp[i];
        m_acc[i] = acc[i];
        m_PosCmd[i] = initPos[i];
        m_VelCmd[i] = 0;
        m_maxVel[i] = maxVel[i];
        m_jointUpperLimit[i] = upperLimits[i];
        m_jointLowerLimit[i] = lowerLimits[i];
    }
    m_runningRate = runningRate;
}
int PosKinController::updateJointPositionCmd(double *desPosition,double *curPosition,double *cmdPosition){
    //根据运动学，根据期望位置获得所有关节的期望速度。
    double desVelocity[JointNum] = {0};
    for(unsigned char i=0;i<JointNum;i++){
        desVelocity[i] = m_Kp[i] * (desPosition[i] - curPosition[i]);
    };
    //这里curPosition换成上一时刻的命令是否更好点呢？

    
    //对速度进行限制,最值限制
    for(unsigned char i=0;i<JointNum;i++){
        if(fabs(desVelocity[i]>m_maxVel[i])){
            desVelocity[i] = fabs(desVelocity[i])/desVelocity[i] * m_maxVel[i];
        }else{
             desVelocity[i] = desVelocity[i];
        }
    }
    
    //根据最大加速度限制，以及上次的cmd速度，获得当前合理的cmd速度。
    //对速度进行限制
    for(unsigned char i=0;i<JointNum;i++){
        double delta_vel = desVelocity[i] - m_VelCmd[i];
        double cmd = 0;
        if(delta_vel>0){
            // speed up
            cmd = m_VelCmd[i] + m_acc[i]/m_runningRate; //全加速
            if(cmd>desVelocity[i]) cmd = desVelocity[i]; //在这个周期如果加速加过头，那就直接把Des当成cmd
        }else{
            // slow down
            // speed up
            cmd = m_VelCmd[i] - m_acc[i]/m_runningRate; //全加速
            if(cmd<desVelocity[i]) cmd = desVelocity[i];
        }
        m_VelCmd[i] = cmd;
    }

    #ifdef OnLinePlanning
    //当前合理的cmd速度，与当前实际的位置积分，获得下一时刻的位置指令。
    for(unsigned char i=0;i<JointNum;i++){
        m_PosCmd[i] = curPosition[i] + m_VelCmd[i]/m_runningRate;
        cmdPosition[i] = m_PosCmd[i];
    }
    #else
    //当前合理的cmd速度，与上一时刻的位置命令，获得下一时刻的位置指令。
    for(unsigned char i=0;i<JointNum;i++){
        m_PosCmd[i] = m_PosCmd[i] + m_VelCmd[i]/m_runningRate;
        cmdPosition[i] = m_PosCmd[i];
    }
    #endif

    //对最后的cmdPosition进行限制
    for(unsigned char i=0;i<JointNum;i++){
        if(i==5){
            //第六关节连续旋转，因此将其映射到-pi ~pi
            if(cmdPosition[i]<m_jointLowerLimit[i]) cmdPosition[i] += 2 * PI;
            if(cmdPosition[i]>m_jointUpperLimit[i]) cmdPosition[i] -= 2 * PI;
        }else{
            //其他关节保证在自己的限位
            if(cmdPosition[i]>m_jointUpperLimit[i]) cmdPosition[i] = m_jointUpperLimit[i];
            if(cmdPosition[i]<m_jointLowerLimit[i]) cmdPosition[i] = m_jointLowerLimit[i];

        }
        
    }

    return 1;
}
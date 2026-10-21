#include "kinematicsController.h"
#include "GlobalVariables.h"

// PosKinController::PosKinController

PosKinController::PosKinController(double *kp,double *acc,double *initPos,double runningRate,
                                            double *maxVel,double *upperLimits, double *lowerLimits){
    for(unsigned char i=0;i<ManipulatorState.NumOfJoint;i++){
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
    double desVelocity[6] = {0};
    for(unsigned char i=0;i<ManipulatorState.NumOfJoint;i++){
        // desVelocity[i] = m_Kp[i] * (desPosition[i] - curPosition[i]); //这个有超调
        desVelocity[i] = m_Kp[i] * (desPosition[i] - m_PosCmd[i]);
    };
    //这里curPosition换成上一时刻的命令是否更好点呢？就不会出现超调

    
    //对速度进行限制,最值限制
    for(unsigned char i=0;i<ManipulatorState.NumOfJoint;i++){
        if(fabs(desVelocity[i])>m_maxVel[i])
        {
            desVelocity[i] = fabs(desVelocity[i])/desVelocity[i] * m_maxVel[i];
        }else{
             desVelocity[i] = desVelocity[i];
        }
    }
    
    //根据最大加速度限制，以及上次的cmd速度，获得当前合理的cmd速度。
    //对速度进行限制
    for(unsigned char i=0;i<ManipulatorState.NumOfJoint;i++){
        double delta_vel = desVelocity[i] - m_VelCmd[i];
        double cmd = 0;
        if(delta_vel>0){
            // speed up
            cmd = m_VelCmd[i] + m_acc[i]/m_runningRate; //全加速
            if(cmd>desVelocity[i]) cmd = desVelocity[i]; //在这个周期如果加速加过头，那就直接把Des当成cmd
        }else{
            // slow down
            // speed up
            cmd = m_VelCmd[i] - m_acc[i]/m_runningRate; //全减速
            if(cmd<desVelocity[i]) cmd = desVelocity[i];
        }
        m_VelCmd[i] = cmd;
    }

    #ifdef OnLinePlanning //OnLinePlanning与当前的位置有关
    //当前合理的cmd速度，与当前实际的位置积分，获得下一时刻的位置指令。
    for(unsigned char i=0;i<ManipulatorState.NumOfJoint;i++){
        m_PosCmd[i] = curPosition[i] + m_VelCmd[i]/m_runningRate;
        cmdPosition[i] = m_PosCmd[i];
    }
    #else
    //当前合理的cmd速度，与上一时刻的位置命令，获得下一时刻的位置指令。
    for(unsigned char i=0;i<ManipulatorState.NumOfJoint;i++){
        m_PosCmd[i] = m_PosCmd[i] + m_VelCmd[i]/m_runningRate;
    }
    #endif

    //对最后的m_PosCmd进行限制
    for(unsigned char i=0;i<ManipulatorState.NumOfJoint;i++){
        if(i==5){
            //第六关节连续旋转，因此将其映射到-pi ~pi
            if(m_PosCmd[i]<m_jointLowerLimit[i]) m_PosCmd[i] += 2 * PI;
            if(m_PosCmd[i]>m_jointUpperLimit[i]) m_PosCmd[i] -= 2 * PI;
        }else{
            //其他关节保证在自己的限位
            if(m_PosCmd[i]>m_jointUpperLimit[i]) m_PosCmd[i] = m_jointUpperLimit[i];
            if(m_PosCmd[i]<m_jointLowerLimit[i]) m_PosCmd[i] = m_jointLowerLimit[i];

        }
        
    }
    //对最后的m_PosCmd进行限制，自碰撞限位，主要针对5关节产生的限位
    //略去

    //将m_PosCmd值给传出去
    for(unsigned char i=0;i<ManipulatorState.NumOfJoint;i++){
        cmdPosition[i] = m_PosCmd[i];
    }
    return 1;
}
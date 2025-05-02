#include "ManipulatorInterface.h"

ManipulatorInterface::ManipulatorInterface(int32_t *joint_offset,uint8_t joint_num, uint8_t control_frequency, canDriver &driver) :
_m_driver(driver),
m_jointnum(joint_num),
m_controlfrequency(control_frequency)
{
    memset(m_jointposition,0, 6*sizeof(double));
    memset(m_jointvelocity,0, 6*sizeof(double));
    memset(m_jointcurrent,0, 6*sizeof(double));
    memset(m_jointacceleration,0, 6*sizeof(double)); 


    for(uint8_t i=0;i<6;i++){
        m_jointoffset[i] = joint_offset[i];
    }
   
}

void ManipulatorInterface::updateState()
{
    for(uint8_t i=0;i<m_jointnum;i++){
        m_jointposition[i] = (ManipulatorState.ActualPosition[i]-m_jointoffset[i])*PENCODER2RAD*m_jointDirection[i];
        if(i==5){
            //对腕部关节进行特殊处理，首先如果速度过小，则认为没有速度。速度超过一定值后进行速度滤波处理
            if(std::abs(m_jointvelocity[i])<0.005){
                m_jointvelocity[i] = 0;
            }else{
                m_jointvelocity[i] = 0.99*m_jointvelocity[i] + 0.01* (ManipulatorState.ActualVelocity[i])*VENCODER2RADS*m_jointDirection[i];
            }
        }else{
            m_jointvelocity[i] = 0.8*m_jointvelocity[i] + 0.2* (ManipulatorState.ActualVelocity[i])*VENCODER2RADS*m_jointDirection[i];
        }

        m_jointcurrent[i]  = 0.9*m_jointcurrent[i] + 0.1* ManipulatorState.ActualCurrent[i]*m_jointDirection[i]/1000.0 * m_ratedCurrent[i];

        //hnz_add 加速度微分
        m_jointacceleration[i] = 0.99* m_jointacceleration[i] + 0.01*(m_jointvelocity[i]-m_previousVelocity[i])*double(m_controlfrequency);   //hnz_add
        m_previousVelocity[i] = m_jointvelocity[i];    //hnz_add
    }
    for(uint8_t i=0;i<m_jointnum;i++){
        if(m_jointposition[i]<-1*PI){ m_jointposition[i] +=2*PI;   }
        if(m_jointposition[i]>PI){  m_jointposition[i] -=2*PI;     }
    }
}

void    ManipulatorInterface::sendPosCmd(const double *q){
    double cmd[6]= {0};
    for(unsigned int i=0;i<m_jointnum;i++){
        cmd[i] = q[i];
        // if(q[i]<m_jointLowerLimits[i]) cmd[i] = m_jointLowerLimits[i];
        // if(q[i]>m_jointUpperLimits[i]) cmd[i] = m_jointUpperLimits[i];
    }
    int posCmd[6] = {0};
    convertJointCmd2DriverCmd(cmd,posCmd);
    IP_Mode_SendCommandPos(&_m_driver,posCmd);
}

void    ManipulatorInterface::sendFdbRequest(){
    IP_Mode_ReceieveState(&_m_driver);
}

void    ManipulatorInterface::handleCANFdb(){
    IP_Mode_ReceieveTPDO(&_m_driver);
}

void ManipulatorInterface::convertJointCmd2DriverCmd(double *q, double *dq, int *posCmd, int *velCmd){
    // 把Trajectory double转为CMD INT类型
    for(uint8_t i=0;i<m_jointnum;i++){
            posCmd[i] = int32_t(q[i]/double(PENCODER2RAD*m_jointDirection[i]))+ m_jointoffset[i];
            velCmd[i] = int32_t(dq[i]/double(VENCODER2RADS*m_jointDirection[i]));
    }
}
void ManipulatorInterface::convertJointCmd2DriverCmd(double *q, int *posCmd){
    // 把Trajectory double转为CMD INT类型
    for(uint8_t i=0;i<m_jointnum;i++){
            posCmd[i] = int32_t(q[i]/double(PENCODER2RAD*m_jointDirection[i]))+ m_jointoffset[i];
    }
}
void ManipulatorInterface::convertDriverFdb2JointFdb(int *posFdb, int *velFdb, int *curFdb,double *pos, double *vel, double *cur){
    for(uint8_t i=0;i<m_jointnum;i++){
        pos[i] = (posFdb[i]-m_jointoffset[i])*PENCODER2RAD*m_jointDirection[i];
        vel[i] = (velFdb[i])*VENCODER2RADS*m_jointDirection[i];
        cur[i]  = curFdb[i]*m_jointDirection[i]/1000.0 * m_ratedCurrent[i];
    }
    for(uint8_t i=0;i<m_jointnum;i++){
        if(pos[i]<-1*PI){ pos[i] +=2*PI;   }
        if(pos[i]>PI){  pos[i] -=2*PI;     }
    }
}

void ManipulatorInterface::IPPosModeInit(){
    
    if(IP_Mode_Init_Pos(&_m_driver)){
        //加入显示
    }
    for(uint i=0;i<5;i++){
        sendFdbRequest();
        handleCANFdb();
        updateState();
        usleep(30000);
    }
    // 把目标值设置为当前的位置，防止上电后突然加速跑快快
    // enable后即开始发布关节位置指令，应避免关节位置指令与当前位置差异较大而产生剧烈运动
    for(uint8_t i=0;i<m_jointnum;i++){
        m_jointPositionCmd[i] = ManipulatorState.ActualPosition[i];
        m_jointVelocityCmd[i] = ManipulatorState.ActualVelocity[i];
    }

    //读取电机的rated current

    
    
}

void ManipulatorInterface::IPPosModeEnable(){
    //使能插值，这个应该所有的都准备好，再搞
    if(IP_Mode_Enable(&_m_driver)){
    }
}
void    ManipulatorInterface::IPPosModeDisable(){
    IP_Mode_Disable(&_m_driver);
}
void ManipulatorInterface::IPCloseGripper()
{
    PDO_Set_TargetTorque(&_m_driver,int16_t(GripperTorque));//闭合
}
void ManipulatorInterface::IPOpenGripper()
{
    PDO_Set_TargetTorque(&_m_driver,int16_t(-1*GripperTorque));//open
}

double DEG2RAD(double deg)
{
    return deg/180.0*PI;
}
double RAD2DEG(double rad)
{
    return rad/PI*180.0;
}
int RAD2ENCODER(double rad){
    return int(rad/PI/2 *65536.0);
}
double ENCODER2Rad(int encoder){
    return double(encoder)/65536.0 *PI*2 ;
}






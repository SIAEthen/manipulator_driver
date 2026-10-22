#include "ManipulatorInterface.h"


// 机械臂接口：负责反馈换算与滤波、关节指令转换和发送、驱动器使能及夹爪控制。
// 运动规划由 ManipulatorPlanner 完成，本类通过传入的 canDriver 访问 CANopen 驱动器。

/**
 * 保存 CAN 驱动引用并初始化关节状态，不在构造函数中打开 CAN 或使能电机。
 * joint_offset：6 个关节的编码器零位偏移；joint_num：参与控制的关节数（最多 6）。
 * control_frequency：控制频率（Hz），用于速度差分计算加速度；driver：外部管理的 CAN 驱动。
 */
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

/**
 * 将全局 ManipulatorState 中的驱动器反馈换算为关节状态，本函数不接收 CAN 消息。
 * 位置：扣除编码器零位并修正方向，单位 rad；速度：换算并滤波，单位 rad/s。
 * 电流：按额定电流和方向换算并滤波，单位 mA；加速度：速度差分后滤波，单位 rad/s²。
 */
void ManipulatorInterface::updateState()
{
    for(uint8_t i=0;i<m_jointnum;i++){
        m_jointposition[i] = (ManipulatorState.ActualPosition[i]-m_jointoffset[i])*PENCODER2RAD*m_jointDirection[i];
        if(i==5){
            // 腕部关节使用更强的低通滤波。
            m_jointvelocity[i] = 0.99*m_jointvelocity[i] + 0.01* (ManipulatorState.ActualVelocity[i])*VENCODER2RADS*m_jointDirection[i];
        }else{
            // 其余关节：保留 80% 的旧速度，加入 20% 的当前反馈。
            m_jointvelocity[i] = 0.8*m_jointvelocity[i] + 0.2* (ManipulatorState.ActualVelocity[i])*VENCODER2RADS*m_jointDirection[i];
        }
        if(std::abs(m_jointvelocity[i]) < 1e-5){
            m_jointvelocity[i] = 0.0;
        }

        // 驱动器电流反馈以额定电流的千分比表示，换算为 mA 后进行低通滤波。
        m_jointcurrent[i]  = 0.9*m_jointcurrent[i] + 0.1* ManipulatorState.ActualCurrent[i]*m_jointDirection[i]/1000.0 * m_ratedCurrent[i];
        if(std::abs(m_jointcurrent[i]) < 1e-5){
            m_jointcurrent[i] = 0.0;
        }

        // 用控制频率近似 1/dt，计算速度差分并低通滤波，再保存本周期速度。
        m_jointacceleration[i] = 0.99* m_jointacceleration[i] + 0.01*(m_jointvelocity[i]-m_previousVelocity[i])*double(m_controlfrequency);   //hnz_add
        m_previousVelocity[i] = m_jointvelocity[i];    //hnz_add
    }
    // 对超出 [-PI, PI] 的位置加减一次 2*PI；不处理跨越多圈的任意角度。
    for(uint8_t i=0;i<m_jointnum;i++){
        if(m_jointposition[i]<-1*PI){ m_jointposition[i] +=2*PI;   }
        if(m_jointposition[i]>PI){  m_jointposition[i] -=2*PI;     }
    }

    computeGripperState();
}

// 保存外部标定的全开、全闭编码器位置，不发送夹爪运动指令。
void ManipulatorInterface::initGripperPosition(const int openPosition, const int closePosition)
{
    m_gripperOpenPosition = openPosition;
    m_gripperClosePosition = closePosition;
}

// 节点 7（数组下标 6）为夹爪，读取已解析到全局状态中的位置反馈。
void ManipulatorInterface::computeGripperState()
{
    int gripperEncoderPosition = ManipulatorState.ActualPosition[6];
    
    const double encoderRange = static_cast<double>(m_gripperOpenPosition)
                              - m_gripperClosePosition;
    if (encoderRange == 0.0) {
        m_gripperAngle = 0.0;
        return;
    }

    const double opening = (static_cast<double>(gripperEncoderPosition)
                            - m_gripperClosePosition) / encoderRange;
    m_gripperAngle = (opening < 0.0 ? 0.0 : (opening > 1.0 ? 1.0 : opening))
                   * m_gripperOpenAngle;
    m_gripperCurrent = ManipulatorState.ActualCurrent[6];
    return;
}

/**
 * 将 q 中的关节位置（rad，至少 m_jointnum 项）转换为编码器整数，通过 RPDO 下发。
 * 此处的位置限幅已注释，调用方需提供合理目标；本函数不发送 SYNC。
 */
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

void    ManipulatorInterface::sendPVCmdPDO(const double *dq_radps){
    for (uint8_t i = 0; i < m_jointnum; ++i) {
            const int32_t velocityCmd = static_cast<int32_t>(dq_radps[i] * 1000.0 * m_jointDirection[i]);
            if (PV_Set_Target_Velocity_PDO(&_m_driver, ManipulatorState.NodeID[i], velocityCmd) != 1)
                return;
            m_jointVelocityCmd[i] = velocityCmd;
    }
}

// 发送 SYNC，触发同步指令执行及反馈；将已有 TPDO 缓存解析到全局 ManipulatorState。
// 此调用不等待本次 SYNC 的新反馈，实际接收由 handleCANFdb() 完成。
void    ManipulatorInterface::sendFdbRequest(){
    IP_Mode_ReceieveState(&_m_driver);
}

// 轮询接收 CAN 消息并更新 TPDO 缓存，主循环中通常以高于控制循环的频率调用。
// 缓存经 sendFdbRequest() 解析后，再由 updateState() 换算为本类的关节状态。
// PDO开始后，can接收消息不等待，没有消息立马返回。
void    ManipulatorInterface::handleCANFdb(){
    IP_Mode_ReceieveTPDO(&_m_driver);
}

// 将 q（rad）、dq（rad/s）换算为驱动器整数位置和速度；各数组至少包含 m_jointnum 项。
// 位置除以转换系数和方向后加编码器零位，速度仅修正比例和方向，不发送 CAN 消息。
void ManipulatorInterface::convertJointCmd2DriverCmd(double *q, double *dq, int *posCmd, int *velCmd){
    for(uint8_t i=0;i<m_jointnum;i++){
            posCmd[i] = int32_t(q[i]/double(PENCODER2RAD*m_jointDirection[i]))+ m_jointoffset[i];
            velCmd[i] = int32_t(dq[i]/double(VENCODER2RADS*m_jointDirection[i]));
    }
}
// 仅转换位置的重载：q 为 rad，posCmd 为包含零位偏移和方向修正的编码器计数。
void ManipulatorInterface::convertJointCmd2DriverCmd(double *q, int *posCmd){
    for(uint8_t i=0;i<m_jointnum;i++){
            posCmd[i] = int32_t(q[i]/double(PENCODER2RAD*m_jointDirection[i]))+ m_jointoffset[i];
    }
}
// 将传入的驱动器整数反馈换算为 pos（rad）、vel（rad/s）、cur（mA）。
// 各数组至少包含 m_jointnum 项；仅做单位、零位、方向及角度调整，不滤波、不更新成员状态。
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


int ManipulatorInterface::initMotionMode(){
    switch (MANIPULATOR_MODE_DEFINE)
    {
    case ManipulatorModeTypeDef::ManipulatorMode_IP_Vel:
        return initIPPosMode();
    case ManipulatorModeTypeDef::ManipulatorMode_IP_Pos:
        return initIPPosMode();
    case ManipulatorModeTypeDef::ManipulatorMode_PV:
        return initPVMode();
    default:
      break;
    }
}
int ManipulatorInterface::enableMotionMode(){
    switch (MANIPULATOR_MODE_DEFINE)
    {
    case ManipulatorModeTypeDef::ManipulatorMode_IP_Vel:
        return enableIPPosMode();
    case ManipulatorModeTypeDef::ManipulatorMode_IP_Pos:
        return enableIPPosMode();
    case ManipulatorModeTypeDef::ManipulatorMode_PV:
        return enablePVMode();
    default:
      break;
    }
}
int ManipulatorInterface::disableMotionMode(){
    switch (MANIPULATOR_MODE_DEFINE)
    {
    case ManipulatorModeTypeDef::ManipulatorMode_IP_Vel:
        return disableIPPosMode();
    case ManipulatorModeTypeDef::ManipulatorMode_IP_Pos:
        return disableIPPosMode();
    case ManipulatorModeTypeDef::ManipulatorMode_PV:
        return disablePVMode();
    default:
      break;
    }
}


/**
 * 配置 1～6 号关节为 IP 插补位置模式（8 ms 周期），7 号夹爪电机为力矩模式。
 * CAN 通道需已初始化；完成后仍需初始化规划器，再调用 enableIPPosMode() 使能关节。
 * 返回 1 表示底层初始化成功，0 表示失败。
 */
int ManipulatorInterface::initIPPosMode(){
    if (IP_Mode_Init_Pos(&_m_driver) != 1) return 0;
    // 连续进行 5 轮同步请求、CAN 接收和状态更新，每轮等待 30 ms。
    for(uint i=0;i<5;i++){
        sendFdbRequest(); // 1个syn信号
        handleCANFdb();   // 尝试读取14个数据
        updateState();
        usleep(30000);
    }
    // 将本类的整数命令数组设为当前反馈；此处仅保存数值，不下发命令。
    // 规划器的位置命令也应从当前关节位置初始化，避免使能后目标突变。
    for(uint8_t i=0;i<m_jointnum;i++){
        m_jointPositionCmd[i] = ManipulatorState.ActualPosition[i];
        m_jointVelocityCmd[i] = ManipulatorState.ActualVelocity[i];
    }

    // 额定电流目前使用头文件中的 m_ratedCurrent 配置，此处尚未实现从驱动器读取。
    return 1;
}

// 在驱动器配置和规划器初始命令准备完成后，使能 1～6 号关节及 IP 插补。
// 夹爪在底层初始化力矩模式时已使能；返回 1 表示使能成功，0 表示失败。
int ManipulatorInterface::enableIPPosMode(){
    return IP_Mode_Enable(&_m_driver) == 1 ? 1 : 0;
}
// 退出 IP 插补，对关节及夹爪驱动器去使能，并使 CANopen 节点进入预操作状态。
// 本函数不关闭 CAN 通道，通道由 canDriver 管理。
int ManipulatorInterface::disableIPPosMode(){
    return IP_Mode_Disable(&_m_driver) == 1 ? 1 : 0;
}

// 初始化关节 PV 模式，并设置默认速度监测参数（10、20 ms、10、20 ms）。
// 不使能运动，不修改驱动器现有的轮廓加减速参数。
int ManipulatorInterface::initPVMode()
{
    // 取消pdo，这时候可以使用SDO
    CAN_NMT_Switch_State(&_m_driver, 0x00, NMT_CMD_PRE_OP);
    sleep(1);
    if(_m_driver.flushCANBuffer())
        return 0;
    
    const bool previousReadWait = canReadWaitFlag;
    canReadWaitFlag = true;
    for (uint8_t i = 0; i < ManipulatorState.NumOfJoint; ++i) {
        const uint16_t nodeID = ManipulatorState.NodeID[i];
        if(PV_Config_PDO(&_m_driver, nodeID)!=1){
            canReadWaitFlag = previousReadWait;
            return 0;
        }
        if (PV_Init(&_m_driver, nodeID) != 1 ) {
            canReadWaitFlag = previousReadWait;
            return 0;
        }
        
        m_jointVelocityCmd[i] = 0;
    }
    //夹钳
    const uint16_t nodeID = ManipulatorState.NodeID[6];
    if(PT_Config_PDO(&_m_driver, nodeID)!=1){
            canReadWaitFlag = previousReadWait;
            return 0;
        }
    if (PT_Torque_Init(&_m_driver, nodeID) != 1) {
            canReadWaitFlag = previousReadWait;
            return 0;
        }
    //NMT start, enable PDO and SDO
    CAN_NMT_Switch_State(&_m_driver, 0x00, NMT_CMD_START);
    sleep(1);
    canReadWaitFlag = previousReadWait;
    return 1;
}

// 清零速度后逐个使能；任一失败时，尝试对所有关节去使能。
int ManipulatorInterface::enablePVMode()
{
    usleep(500000);
    if(_m_driver.flushCANBuffer())
        return 0;
    const bool previousReadWait = canReadWaitFlag;
    canReadWaitFlag = true;
    for (uint8_t i = 0; i < ManipulatorState.NumOfJoint; ++i) {
        if (PV_Enable(&_m_driver, ManipulatorState.NodeID[i]) != 1) {
            disablePVMode();
            canReadWaitFlag = previousReadWait;
            return 0;
        }
        m_jointVelocityCmd[i] = 0;
    }
    canReadWaitFlag = previousReadWait;
    return 1;
}

// 即使某个关节去使能失败，也继续处理其余关节；不改变夹钳状态。
int ManipulatorInterface::disablePVMode()
{
    usleep(500000);
    if(_m_driver.flushCANBuffer())
        return 0;
    const bool previousReadWait = canReadWaitFlag;
    canReadWaitFlag = true;
    int success = 1;
    for (uint8_t i = 0; i < ManipulatorState.NumOfJoint; ++i) {
        if (PV_Disable(&_m_driver, ManipulatorState.NodeID[i]) != 1)
            success = 0;
        else
            m_jointVelocityCmd[i] = 0;
    }
    canReadWaitFlag = previousReadWait;
    return success;
}

// 按配置的夹爪方向，通过 RPDO 发送目标力矩以闭合夹爪。
// GripperTorque 使用驱动器力矩模式的目标值单位，并非直接以 N·m 表示。
int ManipulatorInterface::closeGripperPDOCmd()
{
    return PT_Set_Torque_Current_PDO(&_m_driver,ManipulatorState.NodeID[6],int16_t(m_gripperTorqueDirection * GripperTorque));
    // return PDO_Set_TargetTorque(&_m_driver,int16_t(m_gripperTorqueDirection * GripperTorque));
}
// 按闭合的相反方向打开夹爪；torque 为额定电流的千分比。
int ManipulatorInterface::openGripperPDOCmd()
{
    return PT_Set_Torque_Current_PDO(&_m_driver,ManipulatorState.NodeID[6],int16_t(-m_gripperTorqueDirection * GripperTorque));
    // return PDO_Set_TargetTorque(&_m_driver,int16_t(-m_gripperTorqueDirection * GripperTorque));
}

// 以下为全局辅助函数，不属于 ManipulatorInterface 成员。
// 角度（°）转弧度（rad）。
double DEG2RAD(double deg)
{
    return deg/180.0*PI;
}
// 弧度（rad）转角度（°）。
double RAD2DEG(double rad)
{
    return rad/PI*180.0;
}
// 弧度转编码器计数：每圈按 65536 计数，转为整数时截断；不包含关节零位和方向。
int RAD2ENCODER(double rad){
    return int(rad/PI/2 *65536.0);
}
// 编码器计数转弧度：每圈按 65536 计数，不包含关节零位和方向。
double ENCODER2Rad(int encoder){
    return double(encoder)/65536.0 *PI*2 ;
}






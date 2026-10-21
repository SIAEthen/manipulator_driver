#ifndef __MANIPULATOR_STATE__H__
#define __MANIPULATOR_STATE__H__
#include <stdint.h>
#include <string.h>
#include <vector>
#include <cmath>
#include "canopen.h"
#include "InterPosition.h"
#include "defines.h"
#include "GlobalVariables.h"


double DEG2RAD(double deg);
double RAD2DEG(double rad);
int RAD2ENCODER(double rad);
double ENCODER2Rad(int encoder);


class ManipulatorInterface
{
    friend class    ManipulatorPlanner;
    friend class    ManipulatorDriver;
    public:
        ManipulatorInterface(int32_t *joint_offset,uint8_t joint_num, uint8_t control_frequency, canDriver &driver);
        // 析构函数为空；驱动器去使能和 CAN 通道关闭由外部生命周期管理。
        ~ManipulatorInterface(){

        }

        // 读取全局 ManipulatorState 并换算关节状态，对速度、电流和加速度滤波，不接收 CAN 消息。
        void    updateState();
        // 将关节位置 q（rad）换算为编码器计数，通过 CANopen RPDO 下发，不发送 SYNC。
        void    sendPosCmd(const double *q);
        // 发送 SYNC，并将已有 TPDO1和TPDO2数据 缓存解析到全局 ManipulatorState，不等待新反馈。
        void    sendFdbRequest();
        // 高频轮询 CAN 接收并更新 TPDO 缓存，后续由 sendFdbRequest() 解析反馈。
        void    handleCANFdb();



        void    convertJointCmd2DriverCmd(double *position, double *velocity, int *posCmd, int *velCmd);
        void    convertJointCmd2DriverCmd(double *q, int *posCmd);
        void    convertDriverFdb2JointFdb(int *posFdb, int *velFdb, int *curFdb,double *pos, double *vel, double *cur);

        int     IPPosModeInit();
        int     IPPosModeEnable();
        int     IPPosModeDisable();
        int     IPCloseGripper();
        int     IPOpenGripper();

        void    computeGripperState();
        void    initGripperPosition(const int openPosition, const int closePosition);
        void    setGripperOpenAngle(double angle) { m_gripperOpenAngle = angle; } // rad

    private:

        
        // double m_jointUpperLimits[6] = {DEG2RAD(90),DEG2RAD(170),DEG2RAD(120),DEG2RAD(50),DEG2RAD(90),DEG2RAD(180)};
        // // -0.5是2关节的机械极限， -0.126是3关节的机械极限
        // double m_jointLowerLimits[6] = {DEG2RAD(-90),-0.5, -0.126, DEG2RAD(-50),DEG2RAD(-90),DEG2RAD(-180)};
        
        double m_jointUpperLimits[6] = {DEG2RAD(180),DEG2RAD(180),DEG2RAD(180),DEG2RAD(180),DEG2RAD(180),DEG2RAD(180)};
        // -0.5是2关节的机械极限， -0.126是3关节的机械极限
        double m_jointLowerLimits[6] = {DEG2RAD(-180),DEG2RAD(-180), DEG2RAD(-180), DEG2RAD(-180),DEG2RAD(-180),DEG2RAD(-180)};
        
        double m_jointposition[6];   // rad
        double m_jointvelocity[6];   // rad/s
        double m_jointacceleration[6];   // rad/s^2
        double m_jointcurrent[6];    // mA

        int m_jointPositionCmd[6];   // 驱动器位置指令，编码器计数
        int m_jointVelocityCmd[6];   // 驱动器速度指令，原始整数单位
        int m_jointCurrentCmd[6];   

        int m_gripperOpenPosition = 0;
        int m_gripperClosePosition = 0;
        double m_gripperOpenAngle = PI / 2.0;
        double m_gripperTorqueDirection = 1.0;
        double m_gripperJointDirection = 1.0;
        double m_gripperAngle = 0;

        double m_jointDirection[6] = {-1.0,1.0,1.0,1.0,1.0,1.0};
        int32_t m_jointoffset[6];
        
        canDriver& _m_driver;
        uint8_t m_jointnum;
        uint8_t m_controlfrequency;

        //hnzhnzhnzhnzhnzhnzhnzhnzhnzhnzhnzhnzhnzhnzhnzhnzhnzhnzhnzhnzhnzhnzhnzhnzhnzhnzhnzhnzhnzhnzhnzhnzhnzhnzhnzhnzhnzhnzhnzhnzhnzhnz
    private:
        
        double m_previousVelocity[6] = {0.0,0.0,0.0,0.0,0.0,0.0};   //初始化为0
    public:
        double m_estimatedTorque[6] = {0};
        double m_measuredTorque[6] = {0};
        double m_estimatedDisturbanceTorque[6] = {0};
        double m_forceTorqueBsedFrame[6] = {0};
        double m_driverGain[6] = {0.083*160, 0.083*160, 0.093*100,0.093*100, 0.093*100, 0.8091};
//        Motor1 6000 Motor2 14140 Motor3 8490 Motor 4 8490 Motor5 8490 Motor6 5660
        double m_ratedCurrent[6] = {6000.0,14140.0,8490.0,8490.0,8490.0,5660.0}; //mA
        bool m_runningFlag = false; //如果发布了运动指令，则该值为真，否则该值为假。
        bool m_collisionFlag = false;
};


#endif

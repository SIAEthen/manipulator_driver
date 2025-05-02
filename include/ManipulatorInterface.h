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
        ~ManipulatorInterface(){

        }

        //全局变量ManipulatorState结构体储存着机械臂各关节的信息，该结构体由主循环中的CAN通信相关函数自动更新
        //该变量传入 updateState函数后，更新成员变量 m_jointposition ~ m_jointcurrent
        void    updateState();
        //有了joint的指令之后，调用该函数发送指令到CAN BUS
        void    sendPosCmd(const double *q);
        //在每个循环都应该调用该程序，发送一贞同步信息，再从全局变量中获得机械臂的当前状态
        void    sendFdbRequest();
        // 应该以更高的频率运行该程序，循环接收CAN的消息，收到消息就把数据存储在全局变量中
        void    handleCANFdb();

        void    convertJointCmd2DriverCmd(double *position, double *velocity, int *posCmd, int *velCmd);
        void    convertJointCmd2DriverCmd(double *q, int *posCmd);
        void    convertDriverFdb2JointFdb(int *posFdb, int *velFdb, int *curFdb,double *pos, double *vel, double *cur);

        void    IPPosModeInit();
        void    IPPosModeEnable();
        void    IPPosModeDisable();
        void    IPCloseGripper();
        void    IPOpenGripper();

    private:
        
        double m_jointUpperLimits[6] = {DEG2RAD(90),DEG2RAD(170),DEG2RAD(120),DEG2RAD(50),DEG2RAD(90),DEG2RAD(180)};
        // -0.5是2关节的机械极限， -0.126是3关节的机械极限
        double m_jointLowerLimits[6] = {DEG2RAD(-90),-0.5, -0.126, DEG2RAD(-50),DEG2RAD(-90),DEG2RAD(-180)};
        
        double m_jointposition[6];   // rad
        double m_jointvelocity[6];   // rad/s
        double m_jointacceleration[6];   // rad/s^2
        double m_jointcurrent[6];    // mA

        int m_jointPositionCmd[6];   // rad
        int m_jointVelocityCmd[6];   // rad/s
        int m_jointCurrentCmd[6];   

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

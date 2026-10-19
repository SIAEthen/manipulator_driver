#ifndef _DEFINES_H_
#define _DEFINES_H_

#define PI 3.1415926

#define PENCODER2RAD PI*2.0/65536.0
#define VENCODER2RADS 0.001
#define JOINTQUENNUM 2000
#define CANBuadrate 500
#define CANChannel 0
#define GripperTorque 300
#define JointNum 6

// 1 vel mode
// 2 pos mode
#define ManipulatorMode 2
#define DebugMode 0

//kinematics paramaters of each joint dq = kp*delta_q
#define KinKp1 1.0
#define KinKp2 1.0
#define KinKp3 1.0
#define KinKp4 1.0
#define KinKp5 1.0
#define KinKp6 1.0

#define KinAcc1 1.0 
#define KinAcc2 1.0 
#define KinAcc3 2.0
#define KinAcc4 2.0
#define KinAcc5 2.0
#define KinAcc6 2.0


#define MaxVel1 0.1
#define MaxVel2 0.1
#define MaxVel3 0.2
#define MaxVel4 0.2
#define MaxVel5 0.2
#define MaxVel6 0.5
//define, pos_cmd = pos_cur + vel_cmd * dt
// not define, pos_cmd = last_pos_cmd + + vel_cmd * dt
// #define OnLinePlanning // 测试后，发现巨抖

#endif
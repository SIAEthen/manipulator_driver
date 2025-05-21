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

//define, pos_cmd = pos_cur + vel_cmd * dt
// not define, pos_cmd = last_pos_cmd + + vel_cmd * dt
// #define OnLinePlanning // 测试后，发现巨抖

#endif
#ifndef _DEFINES_H_
#define _DEFINES_H_

#define PI 3.1415926

#define PENCODER2RAD PI*2.0/65536.0
#define VENCODER2RADS 0.001
#define JOINTQUENNUM 2000
#define CANBuadrate 500
#define CANChannel 0
#define GripperTorque 300

/* 机械臂关节控制模式，与 ManipulatorModeTypeDef 的枚举值对应。
 * 2：IP 速度规划模式（ManipulatorMode_IP_Vel）
 *    输入 ~/joint_vel_rads_cmd，6 个关节速度，单位 rad/s。
 *    上位机按加速度约束平滑速度，再以 dt=1/控制频率积分为位置，
 *    通过 IP 位置 PDO 下发；驱动器实际运行在 IP 位置模式。
 * 3：IP 位置规划模式（ManipulatorMode_IP_Pos）
 *    输入 ~/joint_pos_rad_cmd，6 个关节目标位置，单位 rad。
 *    上位机根据位置误差生成速度，经速度、加速度约束后积分为位置，
 *    通过 IP 位置 PDO 下发。
 * 5：PV 速度模式（ManipulatorMode_PV）
 *    输入 ~/joint_vel_rads_cmd，6 个关节速度，单位 rad/s。
 *    上位机进行速度限幅和 rad/s -> mrad/s 换算，通过 PV 速度 PDO 下发；
 *    驱动器按其 Profile acceleration/deceleration 配置执行加减速。
 * 输入数组按关节 1～6 排列；~/ 表示当前驱动节点的私有话题路径。
 * 夹爪独立使用 PT 力矩模式，不受此选项影响。
 * 1 为 Disable，4 为尚未实现的 IP P2P，不用于当前正常控制。
 * 此项为编译期配置，修改后需重新编译并重启驱动。
 */
#define MANIPULATOR_MODE_DEFINE 5

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

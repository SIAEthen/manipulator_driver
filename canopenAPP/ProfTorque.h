/*
 * profTorque.h
 *
 *	Profie Torque - TP 模式 也就是电流控制模式 力矩和电流在这里是一个东西
 *
 *  Created on: 2021年10月25日
 *      Author: maxiufeng
 */

#ifndef APP_INC_PROFTORQUE_H_
#define APP_INC_PROFTORQUE_H_

#include "canopen.h"

/* PT profile-torque 模式初始化
 *   设置运动模式对象为 TP 模式 */
int PT_Torque_Init(canDriver *driver, uint16_t NodeID);

/* Pt 模式 - 输入的目标扭矩、电流 以mA为单位 需要进行转换 参照一下公式进行 得到6071 target-Torque 的设置值
 *   【设置值】=【给定值mA】*1000/【Motor rated current 6075】 */
int PT_Set_Torque_Current(canDriver *driver, uint16_t NodeID, int16_t tarVal);

/* PT 模式 - 终止、停止运动功能 使用控制字的 Halt 位 */
int PT_Halt(canDriver *driver, uint16_t NodeID);

/* 退出 PT 运动模式 */
int PT_Disable(canDriver *driver, uint16_t NodeID);

/* 通过SDO获取驱动器设置的Rated Torque */
int PT_RatedTorque(canDriver *driver, uint16_t NodeID, uint16_t *resVal);

/* 参考 IP 夹爪配置 TPDO1=状态字+电流、TPDO2=位置+速度、RPDO1=控制字、RPDO2=目标力矩。
 * 调用前需进入 NMT Pre-Operational 并开启 canReadWaitFlag；本函数不切换 NMT 或使能电机。
 * 均为同步传输，TPDO2 禁止时间沿用 6000（100 us 单位，即 600 ms）。
 * 返回 1 表示全部配置成功，0 表示失败。
 */
int PT_Config_PDO(canDriver *driver, uint16_t NodeID);

/* RPDO2 下发 2 字节小端目标力矩，tarVal 为 0x6071 原始千分比值，不是 mA。
 * 不执行 PT_Set_Torque_Current 的额定电流换算，不读取 SDO。
 * 需先配置 PDO、进入 NMT Operational；下次 SYNC 生效，本函数不发送 SYNC。
 * 返回 1 表示发送成功，0 表示失败，不代表机械动作完成。
 */
int PT_Set_Torque_Current_PDO(canDriver *driver, uint16_t NodeID, int16_t tarVal);

#endif /* APP_INC_PROFTORQUE_H_ */

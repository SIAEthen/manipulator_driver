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

#endif /* APP_INC_PROFTORQUE_H_ */

/*
 * profVelocity.h
 *
 *	Profie Velocity - PV 模式 也就是速度控制模式
 *
 *  Created on: 2026年10月22日
 *      Author: xinhui
 */

#ifndef APP_INC_PROFVELOCITY_H_
#define APP_INC_PROFVELOCITY_H_

#include "canopen.h"


// 以下接口通过 SDO 通信，调用时需开启 canReadWaitFlag。
// 返回 1 表示所有 SDO 操作成功，0 表示失败；不代表机械运动已完成。

// 清除目标速度、配置 PV 模式并切换到 Switch On，尚不使能运动。
int PV_Init(canDriver *driver, uint16_t NodeID);

// 将目标速度清零，再使能运动；也可在 Halt 后以零目标速度重新启动。
int PV_Enable(canDriver *driver, uint16_t NodeID);

// 配置轮廓加速度、减速度、快速停止减速度，使用驱动器原始单位，均需大于 0。
int PV_Set_Profile(canDriver *driver, uint16_t NodeID,
                   uint32_t acceleration, uint32_t deceleration,
                   uint32_t quickStopDeceleration);

/* 配置速度到达和零速判定参数，各参数范围均为 0～65535（允许为 0）。
 * velocityWindow：0x606D，速度到达窗口，单位为驱动器配置的速度单位。
 * velocityWindowTimeMs：0x606E，速度到达窗口持续时间，单位 ms。
 * velocityThreshold：0x606F，零速判定阈值，单位为驱动器配置的速度单位。
 * velocityThresholdTimeMs：0x6070，零速判定持续时间，单位 ms。
 * 默认窗口和阈值为 10，两个持续时间均为 20 ms。
 * 不进行 rad/s 换算；任一步写入失败即返回 0，已写入的参数不回滚。
 */
int PV_Set_Velocity_Monitoring(canDriver *driver, uint16_t NodeID,
                              uint16_t velocityWindow = 10, uint16_t velocityWindowTimeMs = 20,
                              uint16_t velocityThreshold = 10, uint16_t velocityThresholdTimeMs = 20);

// 设置有符号目标速度（0x60FF），单位由驱动器配置决定，不是直接传 rad/s。
// 不改变使能或 Halt 状态。
int PV_Set_Target_Velocity(canDriver *driver, uint16_t NodeID, int32_t tarVal);

// 目标速度设为 0，按轮廓减速度停车，保持当前使能状态。
int PV_Stop(canDriver *driver, uint16_t NodeID);

// 设置控制字 Halt 位，保持使能；调用 PV_Enable 清零目标并解除 Halt。
int PV_Halt(canDriver *driver, uint16_t NodeID);

// 清零目标速度并执行 Shutdown；不等待机械停车，不复位故障。
int PV_Disable(canDriver *driver, uint16_t NodeID);

int PV_Config_PDO(canDriver *driver, uint16_t NodeID);

int PV_Set_Target_Velocity_PDO(canDriver *driver, uint16_t NodeID, int32_t tarVal);

#endif /* APP_INC_PROFVELOCITY_H_ */

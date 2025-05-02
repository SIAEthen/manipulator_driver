/*
 * ipMode.h
 *
 *  Created on: 2021年10月27日
 *      Author: maxiufeng
 */

#ifndef APP_INC_IPMODE_H_
#define APP_INC_IPMODE_H_

#include "canopen.h"
#include "ProfTorque.h"

typedef enum {
	IP_Flag_Clc  		= (uint8_t)(0b00000000),
	IP_Flag_Init 		= (uint8_t)(0b00000001),
	IP_Flag_Run_Ready 	= (uint8_t)(0b10000011),
	IP_Flag_Run  		= (uint8_t)(0b00000011),
	IP_Flag_Run_XYZ_Traj_Ready 	= (uint8_t)(0b10001100),
	IP_Flag_Run_XYZ_Traj	 	= (uint8_t)(0b00001100),
	IP_Flag_Home_Ready  = (uint8_t)(0b10110000),
	IP_Flag_Home 		= (uint8_t)(0b00110000),
	IP_Flag_Run_XYZ_Line_Ready	=  (uint8_t)(0b10011000),
	IP_Flag_run_XYZ_Line		=  (uint8_t)(0b00011000),
}IPFlagValueTypeDef;


/* IP 模式初始化函数
 *  映射PDO 设置运动参数 设置IP周期等参数
 * 设置NMT启动节点 PDO同步方式设置控制字进入EN_OP 选中IP模式 设置IP子模式 控制字使能IP模式 */
int IP_Mode_Init(canDriver *driver);
void IP_Mode_SendCommandPosVel(canDriver *driver,int32_t *posCMD,int32_t *velCMD);
/* IP 模式初始化函数
 *  映射PDO 设置运动参数 设置IP周期等参数
 * 设置NMT启动节点 PDO同步方式设置控制字进入EN_OP 选中IP模式 设置IP子模式 控制字使能IP模式 */
int IP_Mode_Init_Pos(canDriver *driver);
void IP_Mode_SendCommandPos(canDriver *driver,int32_t *posCMD);


/* IP 状态下驱动器去使能
 * 写状态字清错误标志
 * 驱动器进入到Ready to Switch On 状态下 */
int IP_Mode_Disable(canDriver *driver);

void IP_Mode_ReceieveState(canDriver *driver);
void IP_Mode_ReceieveTPDO(canDriver *driver);

int IP_Mode_Enable(canDriver *driver);

void PDO_Set_TargetTorque(canDriver *driver, int16_t targetTorque);
#endif /* APP_INC_IPMODE_H_ */

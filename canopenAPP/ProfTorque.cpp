/*
 * profTorque.c
 *
 *	Profie Torque - TP 模式 也就是电流控制模式 力矩和电流在这里是一个东西
 *
 *  Created on: 2021年10月25日
 *      Author: maxiufeng
 */

#include "ProfTorque.h"

/* PT profile-torque 模式初始化
 *   设置运动模式对象为 TP 模式 */
int PT_Torque_Init(canDriver *driver, uint16_t NodeID)
{
	uint32_t resVal[1] = {0};
	uint16_t resVal_1[1] = {0};
	// 清空目标值 这个对象
	if (1 != CAN_SDO_PT_Target_Torque_Current(driver, NodeID, 0, resVal)) return 0;

	/* 设置运动模式 选用Profile Torque 模式 */
	if ( 1!= CAN_SDO_Motion_Mode(driver, NodeID, MOTION_MODE_TP, resVal))	return 0;

	/* 控制器切换状态 到 operation enable MO=1 */
	//1.清 控制字
	if ( 1!= CAN_SDO_ControlWord(driver, NodeID, CW_CLEAR, resVal_1))	return 0;
	//2.错误 复位操作
	if ( 1!= CAN_SDO_ControlWord(driver, NodeID, CW_FAULT_RESELT, resVal_1))	return 0;
	//3.切换状态到 ready to switch on
	if ( 1!= CAN_SDO_ControlWord(driver, NodeID, CW_SHUT_DOWN, resVal_1))	return 0;
	//4.切换状态到 switch on
	if ( 1!= CAN_SDO_ControlWord(driver, NodeID, CW_SWITCH_ON, resVal_1))	return 0;
	//5.切换状态到 operation enable
	if ( 1!= CAN_SDO_ControlWord(driver, NodeID, CW_EN_OP, resVal_1))	return 0;

	/* 无错误 返回 1 */
	return 1;
}

/* Pt 模式 - 输入的目标扭矩、电流 以mA为单位 需要进行转换 参照一下公式进行 得到6071 target-Torque 的设置值
 *   【设置值】=【给定值mA】*1000/【Motor rated current 6075】 */
int PT_Set_Torque_Current(canDriver *driver, uint16_t NodeID, int16_t tarVal)
{
	uint32_t resVal[1] = {0};
	// 读电机的额定电流 mA
	uint32_t motorRatedCurrent = 0;
	if (1 != CAN_SDO_PT_Motor_Rated_Current_Value(driver, NodeID, &motorRatedCurrent))	return 0;

	int16_t targetValue = tarVal * 1000 / (int16_t)(motorRatedCurrent);

	// 设置目标扭矩、电流 写入到驱动器
	if (1 != CAN_SDO_PT_Target_Torque_Current(driver, NodeID, targetValue, resVal))	return 0;

	/* 无错误 返回 1 */
	return 1;
}

/* PT 模式 - 终止、停止运动功能 使用控制字的 Halt 位 */
int PT_Halt(canDriver *driver, uint16_t NodeID)
{
	uint16_t resVal_1[1] = {0};
	if (1 != CAN_SDO_ControlWord(driver, NodeID, static_cast<CWValTypeDef>(CW_PT_HALT|CW_EN_OP), resVal_1))	return 0;

	uint32_t resVal[1] = {0};
	if (1 != CAN_SDO_PT_Target_Torque_Current(driver, NodeID, 0, resVal))	return 0;

	if (1 != CAN_SDO_ControlWord(driver, NodeID, CW_EN_OP, resVal_1))	return 0;

	return 1;
}

/* 退出 PT 运动模式 */
int PT_Disable(canDriver *driver, uint16_t NodeID)
{
	uint16_t resVal_1[1] = {0};
	/* 执行cw_en_op，清除 */
	if (1 != CAN_SDO_ControlWord(driver, NodeID, CW_EN_OP,     resVal_1))	return 0;
	/* 切换到switch on状态 */
	if (1 != CAN_SDO_ControlWord(driver, NodeID, CW_SWITCH_ON, resVal_1))	return 0;
	/* 检查状态字决定是否执行错误复位 */
	CAN_SDO_StatusWord(driver, NodeID, resVal_1, PRINT_INFO_OFF);
	if ( resVal_1[0] & SW_FAULT)
		if (1 != CAN_SDO_ControlWord(driver, NodeID, CW_FAULT_RESELT, resVal_1)) return 0;
	/* 切换到ready to switch on状态 */
	if (1 != CAN_SDO_ControlWord(driver, NodeID, CW_SHUT_DOWN, resVal_1))	return 0;

	// 清空目标值 这个对象
	uint32_t resVal[1] = {0};
	if (1 != CAN_SDO_PT_Target_Torque_Current(driver, NodeID, 0, resVal)) return 0;

	/* 过程无误 返回 1 */
	return 1;
}




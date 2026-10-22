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



int PT_Config_PDO(canDriver *driver, uint16_t NodeID)
{
    if (driver == nullptr || NodeID < 1 || NodeID > 127) return 0;

    // 调用方先将节点切换到 NMT Pre-Operational，配置完成后再切回 Operational。
    // TPDO1：状态字（16 bit）+ 实际电流（16 bit），与 IP 模式相同。
    if (CAN_PDO_Set_ComParam(driver, PDO_T1, NodeID, PDO_FUNCTION_DISABLE, PDO_Tx_1SYNC, 0, 0) != 1) return 0;
    ODOBJTypeDef tpdo1Objects[2] = {OBJ_Status_Word, OBJ_Current_Actual_Value};
    if (CAN_PDO_Set_MapParam(driver, PDO_T1, NodeID, 2, tpdo1Objects) != 1) return 0;
    if (CAN_PDO_Set_ComParam(driver, PDO_T1, NodeID, PDO_FUNCTION_SET_TRANS_TYPE, PDO_Tx_1SYNC, 0, 0) != 1) return 0;
    // TODO：TPDO1 inhibit time / event timer，如有需要在此配置。
    if (CAN_PDO_Set_ComParam(driver, PDO_T1, NodeID, PDO_FUNCTION_ENABLE, PDO_Tx_1SYNC, 0, 0) != 1) return 0;

    // TPDO2：实际位置（32 bit）+ 实际速度（32 bit），与现有反馈解析格式相同。
    if (CAN_PDO_Set_ComParam(driver, PDO_T2, NodeID, PDO_FUNCTION_DISABLE, PDO_Tx_1SYNC, 0, 0) != 1) return 0;
    ODOBJTypeDef tpdo2Objects[2] = {OBJ_Actual_Position_User, OBJ_Actual_Velocity_User};
    if (CAN_PDO_Set_MapParam(driver, PDO_T2, NodeID, 2, tpdo2Objects) != 1) return 0;
    if (CAN_PDO_Set_ComParam(driver, PDO_T2, NodeID, PDO_FUNCTION_SET_TRANS_TYPE, PDO_Tx_1SYNC, 0, 0) != 1) return 0;
    // 沿用 IP 夹爪配置：禁止时间 6000，单位 100 us，即 600 ms。
    if (CAN_PDO_Set_ComParam(driver, PDO_T2, NodeID, PDO_FUNCTION_SET_INHIBIT_TIME, PDO_Tx_1SYNC, 6000, 0) != 1) return 0;
    if (CAN_PDO_Set_ComParam(driver, PDO_T2, NodeID, PDO_FUNCTION_ENABLE, PDO_Tx_1SYNC, 0, 0) != 1) return 0;

    // RPDO1：控制字（16 bit），与 IP 模式相同。
    if (CAN_PDO_Set_ComParam(driver, PDO_R1, NodeID, PDO_FUNCTION_DISABLE, PDO_Rx_1_SYNC, 0, 0) != 1) return 0;
    ODOBJTypeDef rpdo1Objects[1] = {OBJ_Control_Word};
    if (CAN_PDO_Set_MapParam(driver, PDO_R1, NodeID, 1, rpdo1Objects) != 1) return 0;
    if (CAN_PDO_Set_ComParam(driver, PDO_R1, NodeID, PDO_FUNCTION_SET_TRANS_TYPE, PDO_Rx_1_SYNC, 0, 0) != 1) return 0;
    if (CAN_PDO_Set_ComParam(driver, PDO_R1, NodeID, PDO_FUNCTION_ENABLE, PDO_Rx_1_SYNC, 0, 0) != 1) return 0;

    // RPDO2：PT 目标力矩（0x6071，16 bit），与 IP 夹爪电机相同。
    if (CAN_PDO_Set_ComParam(driver, PDO_R2, NodeID, PDO_FUNCTION_DISABLE, PDO_Rx_1_SYNC, 0, 0) != 1) return 0;
    ODOBJTypeDef rpdo2Objects[1] = {OBJ_PT_Target_Torque};
    if (CAN_PDO_Set_MapParam(driver, PDO_R2, NodeID, 1, rpdo2Objects) != 1) return 0;
    if (CAN_PDO_Set_ComParam(driver, PDO_R2, NodeID, PDO_FUNCTION_SET_TRANS_TYPE, PDO_Rx_1_SYNC, 0, 0) != 1) return 0;
    if (CAN_PDO_Set_ComParam(driver, PDO_R2, NodeID, PDO_FUNCTION_ENABLE, PDO_Rx_1_SYNC, 0, 0) != 1) return 0;
    return 1;
}

// tarVal 为 0x6071 的原始目标力矩（额定力矩千分比），不是 mA，不做单位换算。
int PT_Set_Torque_Current_PDO(canDriver *driver, uint16_t NodeID, int16_t tarVal)
{
    if (driver == nullptr || NodeID < 1 || NodeID > 127) return 0;
    const uint16_t value = static_cast<uint16_t>(tarVal);
    uint8_t sendData[2] = {
        static_cast<uint8_t>(value & 0xFF),
        static_cast<uint8_t>((value >> 8) & 0xFF)
    };
    return CAN_PDO_Send_Data(driver, PDO_R2, NodeID, 2, sendData) == 1 ? 1 : 0;
}

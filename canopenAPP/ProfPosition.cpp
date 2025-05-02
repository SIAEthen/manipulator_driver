/*
 * profPosition.c
 *
 *	Profile Position - PP 模式 位置控制 点到点【位置和轮廓速度】
 *
 *  Created on: 2021年10月22日
 *      Author: maxiufeng
 */

#include "ProfPosition.h"

/* *********************************************************************************************************************************************** */

/* PP 单个设定点立即运动非混合初始化操作 【新点的执行是基于前一个点被舍弃的基础之上的 既新的点立即执行】。
 * [#] 设置运动模式 @CAN_SDO_Motion_Mode -》MotionModeTypeDef
 * [#] 设置定位选项代码 @POCTypeDef
 * [#] 设置加减速度 @CAN_SDO_PP_Profile_xxxxxx  @CAN_SDO_PP_Quick_Stop_Deceleration
 * [#] 设置控制字的值使驱动器处于 immediately=1，set-point=0， status：OP_EN
 * 		@CAN_SDO_ControlWord
 * 	*/
int PP_Single_SetPoint_Init(canDriver *driver, uint16_t NodeID)
{
	uint32_t resVal[1] = {0}; //这个值是啥意思？啥作用？  储存错误代码
	uint16_t resVal_1[1] = {0};
	/* 设置运动模式 选用Profile Position 模式 */
	if ( 1!= CAN_SDO_Motion_Mode(driver, NodeID, MOTION_MODE_PP, resVal))	return 0;
	/* 设置定位选项代码 rado rro cio :转动的方式-最短路径
	 * 		           immediatelyBit=1-直接覆盖立即执行@POC_CIO_IMMEDIATELY
	 * 		                            等待上个点完成再执行下一个点其实用到了缓存4个点@POC_CIO_CONTINUE_BLEND
	 * 		           驱动器内部强制自动将控制字的set-point bit 清 0 @POC_RRO_RELEASE_BUFFER */
	if ( 1!= CAN_SDO_Position_Option_Code(driver, NodeID, static_cast<POCTypeDef>(POC_CIO_IMMEDIATELY|POC_RRO_RELEASE_BUFFER|POC_RADO_SHORTEST_WAY), resVal))	return 0;
	/* 设置轮廓 加速度 的值 这里固定为 1000000 */
	if ( 1!= CAN_SDO_PP_Profile_Acceleration(driver, NodeID, 1000, resVal))	return 0;
	/* 设置轮廓 减速度 的值 这里固定为 1000000 */
	if ( 1!= CAN_SDO_PP_Profile_Deceleration(driver, NodeID, 1000, resVal))	return 0;
	/* 设置快速停止 减速度 的值 这里固定为 10000 */
	if ( 1!= CAN_SDO_PP_Quick_Stop_Deceleration(driver, NodeID, 1000, resVal))	return 0;

	/* 控制器切换状态 到 operation enable MO=1 */
	//1.清 控制字
	if ( 1!= CAN_SDO_ControlWord(driver, NodeID, CW_CLEAR, resVal_1))	return 0;
	//2.错误 复位操作                       	   0000 0001 0010 1000
	if ( 1!= CAN_SDO_ControlWord(driver, NodeID, CW_FAULT_RESELT, resVal_1))	return 0;
	//3.切换状态到 ready to switch on                        0000 0110
	if ( 1!= CAN_SDO_ControlWord(driver, NodeID, CW_SHUT_DOWN, resVal_1))	return 0;//transition2
	//4.切换状态到 switch on     							  0000 0111
	if ( 1!= CAN_SDO_ControlWord(driver, NodeID, CW_SWITCH_ON, resVal_1))	return 0;//transition3
	//5.切换状态到 operation enable  						  0001 0101
	if ( 1!= CAN_SDO_ControlWord(driver, NodeID, CW_EN_OP, resVal_1))	return 0;  //transition4  see DS402 P39
	//6. immediately=1，absolutely，non-blended
	if ( 1!= CAN_SDO_ControlWord(driver, NodeID, static_cast<CWValTypeDef>(CW_EN_OP|CW_PTP_ABS_MOVEMENT|CW_PTP_CHANGE_SET_POINT_IMM), resVal_1))	return 0;

	// 初始化设置、启动完成，等待接收数据点 set-points 的值

	/* 无错误 返回 1 */
	return 1;
}


/* PP set-points 目标位置 + 轮廓速度 和上个函数配合用
 * [#] 检查状态字的bit12 set-point ACK, =0表示set-point缓存可用 ???
 * [#] 发送 目标位置 @CAN_SDO_PP_Target_Position
 * [#] 发送 轮廓速度 @CAN_SDO_PP_Profile_Velocity
 * [#] 发送 控制字bit4=1 使位置和速度被驱动器接收， 同时驱动器设置状态字bit12=1
 * [#] 发送 控制字bit4=0 为下一次发送数据做准备；让驱动器能够设置状态字的bit12=0
 * */
int PP_SetPoints_Value(canDriver *driver, uint16_t NodeID, int32_t tarPos, uint32_t profVel)
{
	uint32_t resVal[1] = {0};
	uint16_t resVal_1[1] = {0};

	/* 等待状态字中的 New Set-point acknowledge=0 */

	// 目标位置
	if (1 != CAN_SDO_PP_Target_Position(driver, NodeID, tarPos, resVal))		return 0;
	// 轮廓速度
	if (1 != CAN_SDO_PP_Profile_Velocity(driver, NodeID, profVel, resVal))	return 0;
	// 控制字bit4=1，注：此时控制字对象已经有值了，应该保证之前的值不被覆盖掉      ____↑^^^^↓_____
	if (1 != CAN_SDO_ControlWord(driver, NodeID, static_cast<CWValTypeDef>(CW_EN_OP|CW_PTP_ABS_MOVEMENT|CW_PTP_CHANGE_SET_POINT_IMM|CW_PTP_NEW_SET_POINT), resVal_1))
		return 0;
//	if (1 != CAN_SDO_ControlWord(driver, NodeID, CW_EN_OP|CW_PTP_ABS_MOVEMENT|CW_PTP_CHANGE_SET_POINT_IMM			   		    , resVal_1))
//		return 0;

	/* 无错误 返回 1 */
	return 1;
}

/* PP 点到点运动暂停方法 */
int PP_Point_Stop(canDriver *driver, uint16_t nodeID)
{
	uint16_t statusWord = 0;
	if (1 != CAN_SDO_StatusWord(driver, nodeID, &statusWord, PRINT_INFO_OFF))	return 0;

	if (SW_TARGET_REACHED != (statusWord & SW_TARGET_REACHED))
	{
		if (1 != CAN_SDO_ControlWord(driver, nodeID, static_cast<CWValTypeDef>(CW_PTP_HALT_STOP|CW_EN_OP|CW_PTP_ABS_MOVEMENT|CW_PTP_CHANGE_SET_POINT_IMM|CW_PTP_NEW_SET_POINT), &statusWord))	return 0;
	}

	return 1;
}

/* PP 模式下 回归 0 点，这个 0 点是通过elmo软件配置的，可以通过软件修改为任意位置成 0 位
 *    这是PP_SetPoints_Value() 的一个特例 PP_SetPoints_Value()也可实现归0功能*/
int PP_Home_Zero(canDriver *driver, uint16_t modeID)
{
	// 设置 位置为0，轮廓速度为1000RPM
	if (1 != PP_SetPoints_Value(driver, modeID, 0, 1000)) return 0;

	return 1;
}

/* *********************************************************************************************************************************************** */

/* PP 用相对运动实现定速转动 jogging。
 * [#] 设置运动模式 @CAN_SDO_Motion_Mode -》MotionModeTypeDef
 * [#] 设置定位选项代码 @ POCTypeDef
 * [#] 设置加减速度 @CAN_SDO_PP_Profile_xxxxxx  @CAN_SDO_PP_Quick_Stop_Deceleration
 * [#] 设置控制字的值使驱动器处于 immediately=1，set-point=0， status：OP_EN
 * 		@CAN_SDO_ControlWord
 * 	*/
int PP_Jogging_Init(canDriver *driver, uint16_t NodeID)
{
	uint32_t resVal[1] = {0};
	uint16_t resVal_1[1] = {0};
	/* 设置运动模式 选用Profile Position 模式 */
	if ( 1!= CAN_SDO_Motion_Mode(driver, NodeID, MOTION_MODE_PP, resVal))	return 0;
	/* 设置定位选项代码 rado rro cio :转动的方式-最短路径
	 * 		           immediatelyBit=1-直接覆盖立即执行@POC_CIO_IMMEDIATELY
	 * 		                            等待上个点完成再执行下一个点其实用到了缓存4个点@POC_CIO_CONTINUE_BLEND
	 * 		           驱动器内部强制自动将控制字的set-point bit 清 0 @POC_RRO_RELEASE_BUFFER */
	if ( 1!= CAN_SDO_Position_Option_Code(driver, NodeID, static_cast<POCTypeDef>(POC_CIO_IMMEDIATELY|POC_RRO_RELEASE_BUFFER|POC_RADO_LINEAR_AXIS), resVal))	return 0;
	/* 设置轮廓 加速度 的值 这里固定为 1000000 */
	if ( 1!= CAN_SDO_PP_Profile_Acceleration(driver, NodeID, 1000000, resVal))	return 0;
	/* 设置轮廓 减速度 的值 这里固定为 1000000 */
	if ( 1!= CAN_SDO_PP_Profile_Deceleration(driver, NodeID, 1000000, resVal))	return 0;
	/* 设置快速停止 减速度 的值 这里固定为 10000 */
	if ( 1!= CAN_SDO_PP_Quick_Stop_Deceleration(driver, NodeID, 9000, resVal))	return 0;

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
	//6. immediately=1，relative，non-blended
	if ( 1!= CAN_SDO_ControlWord(driver, NodeID, static_cast<CWValTypeDef>(CW_EN_OP|CW_PTP_REL_MOVEMENT|CW_PTP_CHANGE_SET_POINT_IMM), resVal_1))	return 0;

	// 初始化设置、启动完成，等待接收数据点 set-points 的值

	/* 无错误 返回 1 */
	return 1;
}

/* PP 模式 jogging相对运动 设置速度值
 *   速度设置正值 -- 电机正转
 *   速度设置负值 -- 电机反转 */
int PP_Set_Jogging(canDriver *driver, uint16_t NodeID, int32_t jogVel)
{
	int32_t flag = jogVel>=0? 1: -1;
	int32_t relPos = flag * 10*100;	//设置相对转动的角度 10度，根据驱动器配置软件使用的参数得到
	uint32_t velocity = (uint32_t)(flag * jogVel);

	uint32_t resVal[1] = {0};
	uint16_t resVal_1[1] = {0};

	/* 等待状态字中的 New Set-point acknowledge=0 */

	// 目标位置
	if (1 != CAN_SDO_PP_Target_Position(driver, NodeID, relPos, resVal))		return 0;
	// 轮廓速度
	if (1 != CAN_SDO_PP_Profile_Velocity(driver, NodeID, velocity, resVal))	return 0;
	// 控制字bit4=1，注：此时控制字对象已经有值了，应该保证之前的值不被覆盖掉      ____↑^^^^↓_____
	if (1 != CAN_SDO_ControlWord(driver, NodeID, static_cast<CWValTypeDef>(CW_EN_OP|CW_PTP_REL_MOVEMENT|CW_PTP_CHANGE_SET_POINT_IMM|CW_PTP_NEW_SET_POINT), resVal_1))
		return 0;
	//HAL_Delay(2);
//	if (1 != CAN_SDO_ControlWord(driver, NodeID, CW_EN_OP|CW_PTP_REL_MOVEMENT|CW_PTP_CHANGE_SET_POINT_IMM			   		    , resVal_1))
//		return 0;

	/* 无错误 返回 1 */
	return 1;

}

/* PP 模式下 jogging暂停运动 用控制字的 Halt 位 */
int PP_Jogging_Stop(canDriver *driver, uint16_t nodeID)
{
	uint16_t statusWord = 0;
	if (1 != CAN_SDO_StatusWord(driver, nodeID, &statusWord, PRINT_INFO_OFF))	return 0;

	if (SW_TARGET_REACHED != (statusWord & SW_TARGET_REACHED))
	{
		if (1 != CAN_SDO_ControlWord(driver, nodeID, static_cast<CWValTypeDef>(CW_PTP_HALT_STOP|CW_EN_OP|CW_PTP_ABS_MOVEMENT), &statusWord))	return 0;

		//uint16_t resVal_1[1] = {0};
		//if (1 != CAN_SDO_ControlWord(driver, nodeID, CW_EN_OP,     resVal_1))	return 0;
	}

	return 1;
}

/* *********************************************************************************************************************************************** */

/* PP 模式退出操作 也就是驱动器退出 operation enable 状态 */
int PP_Driver_Disable(canDriver *driver, uint16_t NodeID)
{
	uint16_t resVal_1[1] = {0};
	/* 执行cw_en_op，清除控制字中与PP模式相关的位 */
	if (1 != CAN_SDO_ControlWord(driver, NodeID, CW_EN_OP,     resVal_1))	return 0;
	/* 切换到switch on状态 */
	if (1 != CAN_SDO_ControlWord(driver, NodeID, CW_DIS_OP, resVal_1))	return 0;
	/* 检查状态字决定是否执行错误复位 */
	CAN_SDO_StatusWord(driver, NodeID, resVal_1, PRINT_INFO_OFF);
	if ( resVal_1[0] & SW_FAULT)
		if (1 != CAN_SDO_ControlWord(driver, NodeID, CW_FAULT_RESELT, resVal_1)) return 0;
	/* 切换到ready to switch on状态 */
	if (1 != CAN_SDO_ControlWord(driver, NodeID, CW_SHUT_DOWN, resVal_1))	return 0;

	/* 过程无误 返回 1 */
	return 1;
}



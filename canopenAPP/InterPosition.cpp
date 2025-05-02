/*
 * ipMode.c
 *
 *	实现与Interpolation Position 相关的功能
 *
 *	只要退出了 operation enable 状态，就需要再次重新进入到该状态，完后再重新设置IP的状态
 *
 *  Created on: 2021年10月27日
 *      Author: maxiufeng
 */

#include "InterPosition.h"
#include "math.h"


#include "stdlib.h"
#include "stdio.h"

#include "GlobalVariables.h"


/* IP 模式内部状态（1） 进入到 inactive 状态
 * 设置 运动模式为 IP 模式 6060 = 7 */
static int IP_Mode_select(canDriver *driver, uint16_t NodeID)
{
	uint32_t resVal[1] = {0};
	if(1 != CAN_SDO_Motion_Mode(driver, NodeID, MOTION_MODE_IP, resVal))	return 0;
	return 1;
}

static int PT_Mode_select(canDriver *driver, uint16_t NodeID)
{
    uint32_t resVal[1] = {0};
    if(1 != CAN_SDO_Motion_Mode(driver, NodeID, MOTION_MODE_TP, resVal))	return 0;
    return 1;
}

/* IP 模式内部状态（2） 从 inactive 状态退出
 * 设置 运动模式 非IP模式，这里设置为 6060=0 */
static int IP_Mode_Deselect(canDriver *driver, uint16_t NodeID)
{
	uint32_t resVal[1] = {0};
	if(1 != CAN_SDO_Motion_Mode(driver, NodeID, MOTION_MODE_CLC, resVal))	return 0;
	return 1;
}

/* IP 模式内部状态（3） 从 inactive -> active  // PDO 方式发送 1个SYNC
 * 设置控制字 CW bit4 = 1 , 此时的 CW 控制驱动器已经处于 OP_EN 状态 */
static int PDO_IP_Active(canDriver *driver, uint16_t NodeID)
{
	uint8_t txData[2] = {(CW_IP_ENABLE|CW_EN_OP)&0xFF, ((CW_IP_ENABLE|CW_EN_OP)>>8)&0xFF};
	if(1 != CAN_PDO_Send_Data(driver, PDO_R1, NodeID, 2, txData))	return 0;
	return 1;
}

/* IP 模式内部状态（4） 从 active -> inactive  //PDO 方式发送 1个SYNC
 * 设置控制字 CW bit4 = 0 , CW 控制驱动器还在 OP_EN 状态 */
static int PDO_IP_Inactive(canDriver *driver, uint16_t NodeID)
{
	uint8_t txData[2] = {(CW_EN_OP)&0xFF, (CW_EN_OP>>8)&0xFF};
	if(1 != CAN_PDO_Send_Data(driver, PDO_R1, NodeID, 2, txData))	return 0;
	return 1;
}

/* 清空/复位插值数据记录缓存 60C4.6 = 0 */
static int IP_Clear_Disable_Buffer_Data_Record(canDriver *driver, uint16_t NodeID)
{
	uint32_t resVal[1] = {0};
	if(1 != CAN_SDO_IP_Data_Config_Buf_Control(driver, NodeID, IP_BUF_CTRL_CLEAR_DISABLE, resVal))	return 0;
	return 1;
}

/* 使能插值数据记录缓存/允许访问 60C4.6 = 1 */
static int IP_Access_Enable_Buffer_Data_Record(canDriver *driver, uint16_t NodeID)
{
	uint32_t resVal[1] = {0};
	if(1 != CAN_SDO_IP_Data_Config_Buf_Control(driver, NodeID, IP_BUF_CTRL_ENABLE, resVal))	return 0;
	return 1;
}

/* 设置 IP 模式的子模式，子模式决定插值数据记录的类型 仅位置/位置+速度 */
static int IP_Set_Sub_Mode(canDriver *driver, uint16_t NodeID, IPSubModeTypeDef subMode)
{
	uint32_t resVal[1] = {0};
	if(1 != CAN_SDO_IP_Sub_Mode_Select(driver, NodeID, subMode, resVal))	return 0;
	return 1;
}

/* 设置 IP 模式的插值数据缓冲区大小（不应超出最大缓冲区数量值）默认=1，submode0-》1，submode-1-》16 */
static int IP_Set_Buf_Actual_Size(canDriver *driver, uint16_t NodeID, uint32_t ActualBufSize)
{
	uint32_t resVal[1] = {0};
	if(1 != CAN_SDO_IP_Data_Config_Actual_Buf_Size(driver, NodeID, ActualBufSize, resVal))	return 0;
	return 1;
}

/* 设置 IP 模式缓存的类型 FIFO/Ring
 * 注：当sub_mode=-1,buffer_size=1时，设置为Ring类型无意义 */
static int IP_Set_Buf_FIFO_Ring(canDriver *driver, uint16_t NodeID, IPBufOrgTypeDef bufOrg)
{
	uint32_t resVal[1] = {0};
	if(1 != CAN_SDO_IP_Data_Config_Buf_Org(driver, NodeID, bufOrg, resVal))	return 0;
	return 1;
}

/* 设置 IP 模式时间周期 包括周期的值1~10 和 周期值的单位ms 这两个部分 */
static int IP_Set_Period(canDriver *driver, uint16_t NodeID, uint8_t Period, IPTimeIndexTypeDef timeIndex)
{
	uint32_t resVal[1] = {0};
	// 设置IP时间周期值 1-10ms
	if(1 != CAN_SDO_IP_Time_Record_Period(driver, NodeID, Period, resVal))	return 0;
	// 设置IP时间周期的单位 10^-3s = 1ms
	if(1 != CAN_SDO_IP_Time_Record_Period_Index(driver, NodeID, timeIndex, resVal))	return 0;
	return 1;
}

/* 设置 IP 插值最后一个点完成 电机停止运动前 外推再计算几个周期 */
static int IP_Set_Extrapolation_Cycle_Timeout(canDriver *driver, uint16_t NodeID, int16_t cycleNum)
{
	uint32_t resVal[1] = {0};
	if(1 != CAN_SDO_IP_Extrapolation_Cycles_Timeout(driver, NodeID, cycleNum, resVal))	return 0;
	return 1;
}

/* 通过 TPDO 值，获取位置和速度信息 */
static void IP_TPDO_Get_PosVel_Value(canDriver *driver)
{
	for (uint8_t i = 0; i < ManipulatorState.NumOfJoint; ++i)
	{
		
		ManipulatorState.ActualPosition[i] = 0;
		for (uint8_t n = 0; n < 4; n++)
			ManipulatorState.ActualPosition[i] |= ((tpdo2DataStruct.tpdoData[i].m_CanRxData[n]) << (n*8));
		ManipulatorState.ActualVelocity[i] = 0;
		for (uint8_t m = 4; m < 8; m++)
			ManipulatorState.ActualVelocity[i] |= ((tpdo2DataStruct.tpdoData[i].m_CanRxData[m]) << ((m-4)*8));
			
	}
}

/* 通过 TPDO 值，获取 SW 的值 */
static void IP_TPDO_Get_StatusCurrent_Value(canDriver *driver)
{
	for (uint8_t i = 0; i < ManipulatorState.NumOfJoint ; ++i)
	{

		ManipulatorState.ActualStatus[i] = 0;
		for (uint8_t n = 0; n < 2; n++)
			ManipulatorState.ActualStatus[i] |= ((tpdo1DataStruct.tpdoData[i].m_CanRxData[n]) << (n*8));
		ManipulatorState.ActualCurrent[i]=0;
		for (uint8_t n = 2; n < 4; n++)
			ManipulatorState.ActualCurrent[i] |= ((tpdo1DataStruct.tpdoData[i].m_CanRxData[n]) << ((n-2)*8));


	}
}


/* 设置 CW 让驱动器状态机进入到 EN_OP 状态下 */
static int IP_CW_Enable_Operation(canDriver *driver, uint8_t NodeId)
{
	uint16_t resVal[1] = {0};
	// CW 清空 = 0
	if (1 != CAN_SDO_ControlWord(driver, NodeId, CW_CLEAR, resVal))	return 0;
	// CW 复位错误
	if (1 != CAN_SDO_ControlWord(driver, NodeId, CW_FAULT_RESELT, resVal))	return 0;
	// 进入Ready to Switch on 状态
	if (1 != CAN_SDO_ControlWord(driver, NodeId, CW_SHUT_DOWN, resVal))	return 0;
	// 进入Switched on 状态
	if (1 != CAN_SDO_ControlWord(driver, NodeId, CW_SWITCH_ON, resVal))	return 0;
	// 进入Operation Enable 状态
	if (1 != CAN_SDO_ControlWord(driver, NodeId, CW_EN_OP, resVal))	return 0;

	return 1;
}

/* 设置 CW 让驱动器进入到switch on disable状态 */
static int IP_CW_Switch_On_Disable(canDriver *driver, uint8_t NodeId)
{
	uint16_t resVal[1] = {0};
	// 进入Switch on Disable 状态
	if (1 != CAN_SDO_ControlWord(driver, NodeId, CW_DIS_VOL, resVal))	return 0;

	// 把错误复位
	if (1 != CAN_SDO_ControlWord(driver, NodeId, CW_FAULT_RESELT, resVal))	return 0;
	if (1 != CAN_SDO_ControlWord(driver, NodeId, CW_CLEAR, resVal))	return 0;

	return 1;
}

/* 用RPDO发送IP的位置数据 */
static int IP_Send_Position_Velocity(canDriver *driver, uint8_t NodeId, int32_t posValue,int32_t velValue)
{
	uint8_t txData[8] = {(posValue>>0)&0xFF, (posValue>>8)&0xFF, (posValue>>16)&0xFF, (posValue>>24)&0xFF,
    (velValue>>0)&0xFF, (velValue>>8)&0xFF, (velValue>>16)&0xFF, (velValue>>24)&0xFF};
	if (1 != CAN_PDO_Send_Data(driver, PDO_R2, NodeId, 8, txData))	return 0;
	return 1;
}

/* 用RPDO发送IP的位置数据 */
static int IP_Send_Position(canDriver *driver, uint8_t NodeId, int32_t posValue)
{
    uint8_t txData[8] = {(posValue>>0)&0xFF, (posValue>>8)&0xFF, (posValue>>16)&0xFF, (posValue>>24)&0xFF};
    if (1 != CAN_PDO_Send_Data(driver, PDO_R2, NodeId, 4, txData))	return 0;
    return 1;
}

/* 用RPDO发送IP的位置数据 */
static int IP_Send_Torque(canDriver *driver, uint8_t NodeId, int16_t torqueValue)
{
    uint8_t txData[8] = {(torqueValue>>0)&0xFF, (torqueValue>>8)&0xFF};
    if (1 != CAN_PDO_Send_Data(driver, PDO_R2, NodeId, 2, txData))	return 0;
    return 1;
}

/* IP 模式初始化函数
 *  映射PDO 设置运动参数 设置IP周期等参数
 * 设置NMT启动节点 PDO同步方式设置控制字进入EN_OP 选中IP模式 设置IP子模式 控制字使能IP模式 */

int IP_Mode_Init(canDriver *driver)
{

// 判断标志位
	if (ipModeFlags&IP_Flag_Init)
		return 0;
	else {
		ipModeFlags = IP_Flag_Clc;
	}

	// 节点复位
    CAN_NMT_Switch_State(driver, 0x00, NMT_CMD_PRE_OP);
    sleep(1);
	

//1. 设置驱动器的IP相关的对象，主要是buffer相关的字典对象
	for (uint8_t i = 0; i < ManipulatorState.NumOfJoint; ++i) {
		//1) 清空所有的数据记录
		if (1 != IP_Clear_Disable_Buffer_Data_Record(driver, ManipulatorState.NodeID[i]))	return 0;
		//2) 使能buffer的访问
		if (1 != IP_Access_Enable_Buffer_Data_Record(driver, ManipulatorState.NodeID[i]))	return 0;
		//3) 设置子模式
		if (1 != IP_Set_Sub_Mode(driver, ManipulatorState.NodeID[i], IP_SUB_MODE_POS_VEL))	return 0;
		//4) 实际buffer的大小
		if (1 != IP_Set_Buf_Actual_Size(driver, ManipulatorState.NodeID[i], 1))	return 0;
		//5) buffer org 形式 FIFO、Ring
		if (1 != IP_Set_Buf_FIFO_Ring(driver, ManipulatorState.NodeID[i], IP_BUF_ORG_FIFO))	return 0;
		//6) 设置周期 1.2.3....10ms
		if (1 != IP_Set_Period(driver, ManipulatorState.NodeID[i], 10, IP_TIME_INDEX_10_2))	return 0;
		//7) 设置最后一个IP记录后 电机停止运动前 外推计算几个周期点
		if (1 != IP_Set_Extrapolation_Cycle_Timeout(driver, ManipulatorState.NodeID[i], 2))	return 0;
	/*----Note: 单个节点首发一次共计 16 条数据帧 */
		usleep(2000);
	}

//2. 设置包括加速度、加速度等在内的对象值，加载所需值
	for (uint8_t i = 0; i < ManipulatorState.NumOfJoint; ++i) {
		uint32_t resVal[1] = {0};
		//1) 快速停止选项代码设置 = 2
		if (1 != CAN_SDO_Quick_Stop_Option_Code(driver, ManipulatorState.NodeID[i], 2, resVal))	return 0;
		//2) 设置轮廓加速度、减速度、停止减速度值
		if (1 != CAN_SDO_IP_Profile_Acceleration(driver, ManipulatorState.NodeID[i], 1000, resVal))	return 0;
		if (1 != CAN_SDO_IP_Profile_Deceleration(driver, ManipulatorState.NodeID[i], 1000, resVal))	return 0;
		if (1 != CAN_SDO_IP_Quick_Stop_Deceleration(driver, ManipulatorState.NodeID[i], 1000, resVal))	return 0;
	/*----Note: 单个节点首发一次共计 8 条数据帧 */
		usleep(2000);
	}

//3. PDO 对象映射 mapping
	for (uint8_t i = 0; i < ManipulatorState.NumOfJoint; ++i) {
		// TDO1 -》 SW
		/*           销毁/禁用 TPDO1	---------------------------------------------------------------*/
		CAN_PDO_Set_ComParam(driver, PDO_T1, ManipulatorState.NodeID[i], PDO_FUNCTION_DISABLE, PDO_ASYNC_2, 0, 0);
		/*           TPDO1 映射个数清 0 并设置新的映射对象	*/
		ODOBJTypeDef TPDO1MappedObj[2] = {OBJ_Status_Word, OBJ_Current_Actual_Value};
		uint8_t mappingNum = 2;
		CAN_PDO_Set_MapParam(driver, PDO_T1, ManipulatorState.NodeID[i], mappingNum, TPDO1MappedObj);
//		/*           设置 TPDO1 传输类型 改变就发送	*/
//		CAN_PDO_Set_ComParam(driver, PDO_T1, ManipulatorState.NodeID[i], PDO_FUNCTION_SET_TRANS_TYPE, PDO_ASYNC_2, 0, 0);

		/*           设置 TPDO1 传输类型 1 个 SYNC	*/
		CAN_PDO_Set_ComParam(driver, PDO_T1, ManipulatorState.NodeID[i], PDO_FUNCTION_SET_TRANS_TYPE, PDO_Tx_1SYNC, 0, 0);
		/*           设置 TPDO1 禁用时间 为 4*0.1ms = 400us */
//		CAN_PDO_Set_ComParam(driver, PDO_T1, ManipulatorState.NodeID[i], PDO_FUNCTION_SET_INHIBIT_TIME, PDO_Tx_1SYNC, 2000, 0);

		/*           使能 TPDO1	*/
		CAN_PDO_Set_ComParam(driver, PDO_T1, ManipulatorState.NodeID[i], PDO_FUNCTION_ENABLE, PDO_Tx_1SYNC, 0, 0);
	/* TPDO1 单个节点映射收发 12 帧 */

		//TPDO2 -》 Position + velocity
		/*           销毁/禁用 TPDO2	---------------------------------------------------------------*/
		CAN_PDO_Set_ComParam(driver, PDO_T2, ManipulatorState.NodeID[i], PDO_FUNCTION_DISABLE, PDO_Tx_1SYNC, 0, 0);
		/*           TPDO2 映射个数清 0 并设置新的映射对象	*/
		ODOBJTypeDef TPDO2MappedObj[2] = {OBJ_Actual_Position_User, OBJ_Actual_Velocity_User};
//		ODOBJTypeDef TPDO2MappedObj[1] = {OBJ_Actual_Position_User};
		CAN_PDO_Set_MapParam(driver, PDO_T2, ManipulatorState.NodeID[i], 2, TPDO2MappedObj);
		/*           设置 TPDO2 传输类型 1 个 SYNC	*/
		CAN_PDO_Set_ComParam(driver, PDO_T2, ManipulatorState.NodeID[i], PDO_FUNCTION_SET_TRANS_TYPE, PDO_Tx_1SYNC, 0, 0);
		/*           设置 TPDO2 禁用时间 为 4*0.1ms = 400us */
		CAN_PDO_Set_ComParam(driver, PDO_T2, ManipulatorState.NodeID[i], PDO_FUNCTION_SET_INHIBIT_TIME, PDO_Tx_1SYNC, 6000, 0);
		/*           使能 TPDO2	*/
		CAN_PDO_Set_ComParam(driver, PDO_T2, ManipulatorState.NodeID[i], PDO_FUNCTION_ENABLE, PDO_Tx_1SYNC, 0, 0);
	/* TPDO2 单个节点映射收发 14 帧 */


		// RPDO1 -》 CW
		/*           销毁/禁用 RPDO1	---------------------------------------------------------------*/
		CAN_PDO_Set_ComParam(driver, PDO_R1, ManipulatorState.NodeID[i], PDO_FUNCTION_DISABLE, PDO_Rx_1_SYNC, 0, 0);
		//*          RPDO1 映射个数清零 并设置新的映射对象   */
		ODOBJTypeDef RPDO1MappedObj[1] = {OBJ_Control_Word};
		CAN_PDO_Set_MapParam(driver, PDO_R1, ManipulatorState.NodeID[i],    1,    RPDO1MappedObj);
		/*           设置 RPDO1 传输类型 1个同步信号做同步	*/
		CAN_PDO_Set_ComParam(driver, PDO_R1, ManipulatorState.NodeID[i], PDO_FUNCTION_SET_TRANS_TYPE, PDO_Rx_1_SYNC, 0, 0);
		/*           使能 RPDO1	*/
		CAN_PDO_Set_ComParam(driver, PDO_R1, ManipulatorState.NodeID[i], PDO_FUNCTION_ENABLE, PDO_Rx_1_SYNC, 0, 0);
	// /* RPDO1 单个节点映射收发 12 帧 */

		//RPDO2 -》 data record of IP pos / pos+vel
		/*           销毁/禁用 RPDO2	---------------------------------------------------------------*/
		CAN_PDO_Set_ComParam(driver, PDO_R2, ManipulatorState.NodeID[i], PDO_FUNCTION_DISABLE, PDO_Rx_1_SYNC, 0, 0);
		//*          RPDO2 映射个数清零 并设置新的映射对象   */
		// ODOBJTypeDef RPDO2MappedObj[1] = {OBJ_IP_Data_Record_Tag_Pos};
		ODOBJTypeDef RPDO2MappedObj[2] = {OBJ_IP_Data_Record_Tag_Pos, OBJ_IP_Data_Record_Tag_Vel};
		CAN_PDO_Set_MapParam(driver, PDO_R2, ManipulatorState.NodeID[i], 2 /* 2 */,     RPDO2MappedObj);
		/*           设置 RPDO2 传输类型 1个同步信号做同步	*/
		CAN_PDO_Set_ComParam(driver, PDO_R2, ManipulatorState.NodeID[i], PDO_FUNCTION_SET_TRANS_TYPE, PDO_Rx_1_SYNC, 0, 0);
		/*           使能 RPDO2	*/
		CAN_PDO_Set_ComParam(driver, PDO_R2, ManipulatorState.NodeID[i], PDO_FUNCTION_ENABLE, PDO_Rx_1_SYNC, 0, 0);
	/* RPDO1 单个节点映射收发 12 帧 */

	}
//#####################################################################################################//
    //zxh设置夹钳PDO Mapping，禁用TPDOx，RPDO1 control word， RPDO2 TargetTroque
	/*           销毁/禁用 TPDO1	---------------------------------------------------------------*/
	CAN_PDO_Set_ComParam(driver, PDO_T1, ManipulatorState.NodeID[6], PDO_FUNCTION_DISABLE, PDO_ASYNC_2, 0, 0);
	/*           销毁/禁用 TPDO1	---------------------------------------------------------------*/
	CAN_PDO_Set_ComParam(driver, PDO_T2, ManipulatorState.NodeID[6], PDO_FUNCTION_DISABLE, PDO_ASYNC_2, 0, 0);


    // RPDO1 -》 CW
    /*           销毁/禁用 RPDO1	---------------------------------------------------------------*/
    CAN_PDO_Set_ComParam(driver, PDO_R1, ManipulatorState.NodeID[6], PDO_FUNCTION_DISABLE, PDO_Rx_1_SYNC, 0, 0);
    //*          RPDO1 映射个数清零 并设置新的映射对象   */
    ODOBJTypeDef RPDO1MappedObj[1] = {OBJ_Control_Word};
    CAN_PDO_Set_MapParam(driver, PDO_R1, ManipulatorState.NodeID[6],    1,    RPDO1MappedObj);
    /*           设置 RPDO1 传输类型 1个同步信号做同步	*/
    CAN_PDO_Set_ComParam(driver, PDO_R1, ManipulatorState.NodeID[6], PDO_FUNCTION_SET_TRANS_TYPE, PDO_Rx_1_SYNC, 0, 0);
    /*           使能 RPDO1	*/
    CAN_PDO_Set_ComParam(driver, PDO_R1, ManipulatorState.NodeID[6], PDO_FUNCTION_ENABLE, PDO_Rx_1_SYNC, 0, 0);
    // /* RPDO1 单个节点映射收发 12 帧 */

    //RPDO2 -》 data record of IP pos / pos+vel
    /*           销毁/禁用 RPDO2	---------------------------------------------------------------*/
    CAN_PDO_Set_ComParam(driver, PDO_R2, ManipulatorState.NodeID[6], PDO_FUNCTION_DISABLE, PDO_Rx_1_SYNC, 0, 0);
    //*          RPDO2 映射个数清零 并设置新的映射对象   */
     ODOBJTypeDef RPDO2MappedObj[1] = {OBJ_PT_Target_Torque};
    CAN_PDO_Set_MapParam(driver, PDO_R2, ManipulatorState.NodeID[6], 1 /* 2 */,     RPDO2MappedObj);
    /*           设置 RPDO2 传输类型 1个同步信号做同步	*/
    CAN_PDO_Set_ComParam(driver, PDO_R2, ManipulatorState.NodeID[6], PDO_FUNCTION_SET_TRANS_TYPE, PDO_Rx_1_SYNC, 0, 0);
    /*           使能 RPDO2	*/
    CAN_PDO_Set_ComParam(driver, PDO_R2, ManipulatorState.NodeID[6], PDO_FUNCTION_ENABLE, PDO_Rx_1_SYNC, 0, 0);
    /* RPDO1 单个节点映射收发 12 帧 */
//#####################################################################################################//






//4. 设置NMT，节点进入NMT 开始状态 , 00 -》所有节点
	CAN_NMT_Switch_State(driver, 0x00, NMT_CMD_START);
    sleep(1);
	// 接收7个驱动器回复的消息
	for(uint8_t i = 0; i < ManipulatorState.NumOfMotor; ++i)
	{
		CAN_Rx_Data_Handle(driver, ManipulatorState.NodeID[i]);
	}

	/* NMT 进入启动状态 1 帧 -- 如果是通电后第一次启动 SW TPDO 数据有从节点个数个 */
	usleep(5000);
	CAN_SYNC_Msg(driver);
	for(uint8_t i = 0; i < ManipulatorState.NumOfJoint; ++i)
	{
		CAN_Rx_Data_Handle(driver, ManipulatorState.NodeID[i]);
		CAN_Rx_Data_Handle(driver, ManipulatorState.NodeID[i]);
	}
	
	IP_TPDO_Get_PosVel_Value(driver);
	IP_TPDO_Get_StatusCurrent_Value(driver);





// 获取驱动器的状态值 并判断是否有错误， 如果其中的一个有错误，设置CW到06 Ready to Switch on 状态
	/*           销毁/禁用 TPDO1	---------------------------------------------------------------*/
//	for (uint8_t i = 0; i < ManipulatorState.NumOfJoint; ++i) {
//		CAN_PDO_Set_ComParam(driver, PDO_T1, ManipulatorState.NodeID[i], PDO_FUNCTION_DISABLE, PDO_ASYNC_2, 0, 0);
//	}

	// sleep(2);
	// CAN_SYNC_Msg(driver);
	// IP_TPDO_Get_PosVel_Value(driver);
	// IP_TPDO_Get_StatusCurrent_Value(driver);

// 设置标志位
	ipModeFlags = IP_Flag_Init;
	// 所有的都设置完，设置成功 返回 1

//#####################################################################################################//
    //此时系统还没有开始循环发布SYNC信息，所以可以用SDO初始化PT模式，此时电机已经使能了，后续不需要另外使能。但设置的目标电流为0.
    PT_Torque_Init(driver, ManipulatorState.NodeID[6]);
//#####################################################################################################//

	return 1;
}


/* IP 模式初始化函数
 *  映射PDO 设置运动参数 设置IP周期等参数
 * 设置NMT启动节点 PDO同步方式设置控制字进入EN_OP 选中IP模式 设置IP子模式 控制字使能IP模式 */

int IP_Mode_Init_Pos(canDriver *driver)
{

    // 判断标志位
    if (ipModeFlags&IP_Flag_Init)
        return 0;
    else {
        ipModeFlags = IP_Flag_Clc;
    }

    // 节点复位
    CAN_NMT_Switch_State(driver, 0x00, NMT_CMD_PRE_OP);
    sleep(1);


    //1. 设置驱动器的IP相关的对象，主要是buffer相关的字典对象
    for (uint8_t i = 0; i < ManipulatorState.NumOfJoint; ++i) {
        //1) 清空所有的数据记录
        if (1 != IP_Clear_Disable_Buffer_Data_Record(driver, ManipulatorState.NodeID[i]))	return 0;
        //2) 使能buffer的访问
        if (1 != IP_Access_Enable_Buffer_Data_Record(driver, ManipulatorState.NodeID[i]))	return 0;
        //3) 设置子模式
        if (1 != IP_Set_Sub_Mode(driver, ManipulatorState.NodeID[i], IP_SUB_MODE_POS))	return 0;
        //4) 实际buffer的大小
        if (1 != IP_Set_Buf_Actual_Size(driver, ManipulatorState.NodeID[i], 1))	return 0;
        //5) buffer org 形式 FIFO、Ring
        if (1 != IP_Set_Buf_FIFO_Ring(driver, ManipulatorState.NodeID[i], IP_BUF_ORG_FIFO))	return 0;
        //6) 设置周期 1.2.3....10ms
        if (1 != IP_Set_Period(driver, ManipulatorState.NodeID[i], 8, IP_TIME_INDEX_10_3))	return 0;
        //7) 设置最后一个IP记录后 电机停止运动前 外推计算几个周期点
        if (1 != IP_Set_Extrapolation_Cycle_Timeout(driver, ManipulatorState.NodeID[i], 2))	return 0;
        /*----Note: 单个节点首发一次共计 16 条数据帧 */
        usleep(2000);
    }

    //2. 设置包括加速度、加速度等在内的对象值，加载所需值
    for (uint8_t i = 0; i < ManipulatorState.NumOfJoint; ++i) {
        uint32_t resVal[1] = {0};
        //1) 快速停止选项代码设置 = 2
        if (1 != CAN_SDO_Quick_Stop_Option_Code(driver, ManipulatorState.NodeID[i], 2, resVal))	return 0;
        //2) 设置轮廓加速度、减速度、停止减速度值
        if (1 != CAN_SDO_IP_Profile_Acceleration(driver, ManipulatorState.NodeID[i], 1000, resVal))	return 0;
        if (1 != CAN_SDO_IP_Profile_Deceleration(driver, ManipulatorState.NodeID[i], 1000, resVal))	return 0;
        if (1 != CAN_SDO_IP_Quick_Stop_Deceleration(driver, ManipulatorState.NodeID[i], 1000, resVal))	return 0;
        /*----Note: 单个节点首发一次共计 8 条数据帧 */
        usleep(2000);
    }

    //3. PDO 对象映射 mapping
    for (uint8_t i = 0; i < ManipulatorState.NumOfJoint; ++i) {
        // TDO1 -》 SW
        /*           销毁/禁用 TPDO1	---------------------------------------------------------------*/
        CAN_PDO_Set_ComParam(driver, PDO_T1, ManipulatorState.NodeID[i], PDO_FUNCTION_DISABLE, PDO_ASYNC_2, 0, 0);
        /*           TPDO1 映射个数清 0 并设置新的映射对象	*/
        ODOBJTypeDef TPDO1MappedObj[2] = {OBJ_Status_Word, OBJ_Current_Actual_Value};
        uint8_t mappingNum = 2;
        CAN_PDO_Set_MapParam(driver, PDO_T1, ManipulatorState.NodeID[i], mappingNum, TPDO1MappedObj);
        //		/*           设置 TPDO1 传输类型 改变就发送	*/
        //		CAN_PDO_Set_ComParam(driver, PDO_T1, ManipulatorState.NodeID[i], PDO_FUNCTION_SET_TRANS_TYPE, PDO_ASYNC_2, 0, 0);

        /*           设置 TPDO1 传输类型 1 个 SYNC	*/
        CAN_PDO_Set_ComParam(driver, PDO_T1, ManipulatorState.NodeID[i], PDO_FUNCTION_SET_TRANS_TYPE, PDO_Tx_1SYNC, 0, 0);
        /*           设置 TPDO1 禁用时间 为 4*0.1ms = 400us */
        //		CAN_PDO_Set_ComParam(driver, PDO_T1, ManipulatorState.NodeID[i], PDO_FUNCTION_SET_INHIBIT_TIME, PDO_Tx_1SYNC, 2000, 0);

        /*           使能 TPDO1	*/
        CAN_PDO_Set_ComParam(driver, PDO_T1, ManipulatorState.NodeID[i], PDO_FUNCTION_ENABLE, PDO_Tx_1SYNC, 0, 0);
        /* TPDO1 单个节点映射收发 12 帧 */

        //TPDO2 -》 Position + velocity
        /*           销毁/禁用 TPDO2	---------------------------------------------------------------*/
        CAN_PDO_Set_ComParam(driver, PDO_T2, ManipulatorState.NodeID[i], PDO_FUNCTION_DISABLE, PDO_Tx_1SYNC, 0, 0);
        /*           TPDO2 映射个数清 0 并设置新的映射对象	*/
        ODOBJTypeDef TPDO2MappedObj[2] = {OBJ_Actual_Position_User, OBJ_Actual_Velocity_User};
        //		ODOBJTypeDef TPDO2MappedObj[1] = {OBJ_Actual_Position_User};
        CAN_PDO_Set_MapParam(driver, PDO_T2, ManipulatorState.NodeID[i], 2, TPDO2MappedObj);
        /*           设置 TPDO2 传输类型 1 个 SYNC	*/
        CAN_PDO_Set_ComParam(driver, PDO_T2, ManipulatorState.NodeID[i], PDO_FUNCTION_SET_TRANS_TYPE, PDO_Tx_1SYNC, 0, 0);
        /*           设置 TPDO2 禁用时间 为 4*0.1ms = 400us */
        CAN_PDO_Set_ComParam(driver, PDO_T2, ManipulatorState.NodeID[i], PDO_FUNCTION_SET_INHIBIT_TIME, PDO_Tx_1SYNC, 6000, 0);
        /*           使能 TPDO2	*/
        CAN_PDO_Set_ComParam(driver, PDO_T2, ManipulatorState.NodeID[i], PDO_FUNCTION_ENABLE, PDO_Tx_1SYNC, 0, 0);
        /* TPDO2 单个节点映射收发 14 帧 */


        // RPDO1 -》 CW
        /*           销毁/禁用 RPDO1	---------------------------------------------------------------*/
        CAN_PDO_Set_ComParam(driver, PDO_R1, ManipulatorState.NodeID[i], PDO_FUNCTION_DISABLE, PDO_Rx_1_SYNC, 0, 0);
        //*          RPDO1 映射个数清零 并设置新的映射对象   */
        ODOBJTypeDef RPDO1MappedObj[1] = {OBJ_Control_Word};
        CAN_PDO_Set_MapParam(driver, PDO_R1, ManipulatorState.NodeID[i],    1,    RPDO1MappedObj);
        /*           设置 RPDO1 传输类型 1个同步信号做同步	*/
        CAN_PDO_Set_ComParam(driver, PDO_R1, ManipulatorState.NodeID[i], PDO_FUNCTION_SET_TRANS_TYPE, PDO_Rx_1_SYNC, 0, 0);
        /*           使能 RPDO1	*/
        CAN_PDO_Set_ComParam(driver, PDO_R1, ManipulatorState.NodeID[i], PDO_FUNCTION_ENABLE, PDO_Rx_1_SYNC, 0, 0);
        // /* RPDO1 单个节点映射收发 12 帧 */

        //RPDO2 -》 data record of IP pos / pos+vel
        /*           销毁/禁用 RPDO2	---------------------------------------------------------------*/
        CAN_PDO_Set_ComParam(driver, PDO_R2, ManipulatorState.NodeID[i], PDO_FUNCTION_DISABLE, PDO_Rx_1_SYNC, 0, 0);
        //*          RPDO2 映射个数清零 并设置新的映射对象   */
        // ODOBJTypeDef RPDO2MappedObj[1] = {OBJ_IP_Data_Record_Tag_Pos};
        ODOBJTypeDef RPDO2MappedObj[1] = {OBJ_IP_Data_Record_Tag_Pos};
        CAN_PDO_Set_MapParam(driver, PDO_R2, ManipulatorState.NodeID[i], 1 /* 2 */,     RPDO2MappedObj);
        /*           设置 RPDO2 传输类型 1个同步信号做同步	*/
        CAN_PDO_Set_ComParam(driver, PDO_R2, ManipulatorState.NodeID[i], PDO_FUNCTION_SET_TRANS_TYPE, PDO_Rx_1_SYNC, 0, 0);
        /*           使能 RPDO2	*/
        CAN_PDO_Set_ComParam(driver, PDO_R2, ManipulatorState.NodeID[i], PDO_FUNCTION_ENABLE, PDO_Rx_1_SYNC, 0, 0);
        /* RPDO1 单个节点映射收发 12 帧 */

    }
    //#####################################################################################################//
    //zxh设置夹钳PDO Mapping，禁用TPDOx，RPDO1 control word， RPDO2 TargetTroque
    /*           销毁/禁用 TPDO1	---------------------------------------------------------------*/
    CAN_PDO_Set_ComParam(driver, PDO_T1, ManipulatorState.NodeID[6], PDO_FUNCTION_DISABLE, PDO_ASYNC_2, 0, 0);
    /*           销毁/禁用 TPDO1	---------------------------------------------------------------*/
    CAN_PDO_Set_ComParam(driver, PDO_T2, ManipulatorState.NodeID[6], PDO_FUNCTION_DISABLE, PDO_ASYNC_2, 0, 0);


    // RPDO1 -》 CW
    /*           销毁/禁用 RPDO1	---------------------------------------------------------------*/
    CAN_PDO_Set_ComParam(driver, PDO_R1, ManipulatorState.NodeID[6], PDO_FUNCTION_DISABLE, PDO_Rx_1_SYNC, 0, 0);
    //*          RPDO1 映射个数清零 并设置新的映射对象   */
    ODOBJTypeDef RPDO1MappedObj[1] = {OBJ_Control_Word};
    CAN_PDO_Set_MapParam(driver, PDO_R1, ManipulatorState.NodeID[6],    1,    RPDO1MappedObj);
    /*           设置 RPDO1 传输类型 1个同步信号做同步	*/
    CAN_PDO_Set_ComParam(driver, PDO_R1, ManipulatorState.NodeID[6], PDO_FUNCTION_SET_TRANS_TYPE, PDO_Rx_1_SYNC, 0, 0);
    /*           使能 RPDO1	*/
    CAN_PDO_Set_ComParam(driver, PDO_R1, ManipulatorState.NodeID[6], PDO_FUNCTION_ENABLE, PDO_Rx_1_SYNC, 0, 0);
    // /* RPDO1 单个节点映射收发 12 帧 */

    //RPDO2 -》 data record of IP pos / pos+vel
    /*           销毁/禁用 RPDO2	---------------------------------------------------------------*/
    CAN_PDO_Set_ComParam(driver, PDO_R2, ManipulatorState.NodeID[6], PDO_FUNCTION_DISABLE, PDO_Rx_1_SYNC, 0, 0);
    //*          RPDO2 映射个数清零 并设置新的映射对象   */
    ODOBJTypeDef RPDO2MappedObj[1] = {OBJ_PT_Target_Torque};
    CAN_PDO_Set_MapParam(driver, PDO_R2, ManipulatorState.NodeID[6], 1 /* 2 */,     RPDO2MappedObj);
    /*           设置 RPDO2 传输类型 1个同步信号做同步	*/
    CAN_PDO_Set_ComParam(driver, PDO_R2, ManipulatorState.NodeID[6], PDO_FUNCTION_SET_TRANS_TYPE, PDO_Rx_1_SYNC, 0, 0);
    /*           使能 RPDO2	*/
    CAN_PDO_Set_ComParam(driver, PDO_R2, ManipulatorState.NodeID[6], PDO_FUNCTION_ENABLE, PDO_Rx_1_SYNC, 0, 0);
    /* RPDO1 单个节点映射收发 12 帧 */
    //#####################################################################################################//






    //4. 设置NMT，节点进入NMT 开始状态 , 00 -》所有节点
    CAN_NMT_Switch_State(driver, 0x00, NMT_CMD_START);
    sleep(1);
    // 接收7个驱动器回复的消息
    for(uint8_t i = 0; i < ManipulatorState.NumOfMotor; ++i)
    {
        CAN_Rx_Data_Handle(driver, ManipulatorState.NodeID[i]);
    }

    /* NMT 进入启动状态 1 帧 -- 如果是通电后第一次启动 SW TPDO 数据有从节点个数个 */
    usleep(5000);
    CAN_SYNC_Msg(driver);
    for(uint8_t i = 0; i < ManipulatorState.NumOfJoint; ++i)
    {
        CAN_Rx_Data_Handle(driver, ManipulatorState.NodeID[i]);
        CAN_Rx_Data_Handle(driver, ManipulatorState.NodeID[i]);
    }

    IP_TPDO_Get_PosVel_Value(driver);
    IP_TPDO_Get_StatusCurrent_Value(driver);





    // 获取驱动器的状态值 并判断是否有错误， 如果其中的一个有错误，设置CW到06 Ready to Switch on 状态
    /*           销毁/禁用 TPDO1	---------------------------------------------------------------*/
    //	for (uint8_t i = 0; i < ManipulatorState.NumOfJoint; ++i) {
    //		CAN_PDO_Set_ComParam(driver, PDO_T1, ManipulatorState.NodeID[i], PDO_FUNCTION_DISABLE, PDO_ASYNC_2, 0, 0);
    //	}

    // sleep(2);
    // CAN_SYNC_Msg(driver);
    // IP_TPDO_Get_PosVel_Value(driver);
    // IP_TPDO_Get_StatusCurrent_Value(driver);

    // 设置标志位
    ipModeFlags = IP_Flag_Init;
    // 所有的都设置完，设置成功 返回 1

    //#####################################################################################################//
    //此时系统还没有开始循环发布SYNC信息，所以可以用SDO初始化PT模式，此时电机已经使能了，后续不需要另外使能。但设置的目标电流为0.
    PT_Torque_Init(driver, ManipulatorState.NodeID[6]);
    //#####################################################################################################//

    return 1;
}

//#####################################################################################################//


//#####################################################################################################//
// 让机械比关节1-6使能，注意，此时夹爪已经使能，不需要额外使能
int IP_Mode_Enable(canDriver *driver)
{

	//5. 设置控制字让电机进入到 EN_OP 状态下
	for (uint8_t i = 0; i < ManipulatorState.NumOfJoint; ++i) {
		if (1 != IP_CW_Enable_Operation(driver, ManipulatorState.NodeID[i]))	return 0;
		usleep(1000);
		
	/* 设置驱动器状态切换 单个节点收发 10 帧 */
	}

//6. 设置运动模式为 IP 6060 = 7;
	for (uint8_t i = 0; i < ManipulatorState.NumOfJoint; ++i) {
		if (1 != IP_Mode_select(driver, ManipulatorState.NodeID[i]))	return 0;
		usleep(1000);
	/* 单个节点首发 2 帧 */
	}

    for (uint8_t i = 0; i < ManipulatorState.NumOfJoint; ++i) {
        if (1 != PDO_IP_Active(driver, ManipulatorState.NodeID[i]))	return 0;
        usleep(1000);
	/* 单个节点首发 2 帧 */
	}



}
/* IP 状态下驱动器去使能
 * 写状态字清错误标志
 * 驱动器进入到Ready to Switch On 状态下 */
// 让所有的驱动器都切换到ready to Switch On
int IP_Mode_Disable(canDriver *driver)
{
	uint16_t resVal_1[1] = {0};
	// CAN_SYNC_Msg(driver);
	// sleep(1);

    // 让所有的驱动器都去使能
    for (uint8_t idx = 0; idx < ManipulatorState.NumOfJoint; ++idx) {
        PDO_IP_Inactive(driver, ManipulatorState.NodeID[idx]); // PDO取消激活
        usleep(2000);
    }

    for (uint8_t idx = 0; idx < ManipulatorState.NumOfMotor; ++idx) {
        IP_CW_Switch_On_Disable(driver, ManipulatorState.NodeID[idx]); // 驱动器转化为switch on disable，所有驱动器都一样
		usleep(2000);
	}
	usleep(2000);
	// 发送一个同步消息
//	CAN_SYNC_Msg(driver);


    // 所有NMT进入 Pre Op 状态下，禁用PDO
    CAN_NMT_Switch_State(driver, 00, NMT_CMD_PRE_OP);
    sleep(1);
    // IP 状态机退出 运动模式清空 进入到EN_OP
    for (uint8_t idx = 0; idx < ManipulatorState.NumOfMotor; ++idx) {
        IP_Mode_Deselect(driver, ManipulatorState.NodeID[idx]);
    }
	// 驱动器状态机进入 Ready to Switch On 下
	for (uint8_t idx = 0; idx < ManipulatorState.NumOfMotor; ++idx) {
		CAN_SDO_ControlWord(driver, ManipulatorState.NodeID[idx], CW_DIS_OP, resVal_1);
		/* 检查状态字决定是否执行错误复位 */
		CAN_SDO_StatusWord(driver, ManipulatorState.NodeID[idx], resVal_1, PRINT_INFO_OFF);
		if ( resVal_1[0] & SW_FAULT)
			if (1 != CAN_SDO_ControlWord(driver, ManipulatorState.NodeID[idx], CW_FAULT_RESELT, resVal_1)) return 0;
		CAN_SDO_ControlWord(driver, ManipulatorState.NodeID[idx], CW_SHUT_DOWN, resVal_1);
	}

// 初始化完成标志位复位
	ipModeFlags = IP_Flag_Clc;

	return 1;
}

void IP_Mode_ReceieveState(canDriver *driver)
{
	CAN_SYNC_Msg(driver);
    IP_TPDO_Get_PosVel_Value(driver);
    IP_TPDO_Get_StatusCurrent_Value(driver);
}

void IP_Mode_ReceieveTPDO(canDriver *driver)
{
    for(uint8_t i=0;i<ManipulatorState.NumOfJoint;i++)
        {
            CAN_Rx_Data_Handle(driver, ManipulatorState.NodeID[i]);
            CAN_Rx_Data_Handle(driver, ManipulatorState.NodeID[i]);
        };
}
void IP_Mode_SendCommandPosVel(canDriver *driver,int32_t *posCMD,int32_t *velCMD)
{

    velCMD[0] = 0; //no sense

    for(uint8_t i=0;i<ManipulatorState.NumOfJoint;i++){
        IP_Send_Position_Velocity(driver, ManipulatorState.NodeID[i], posCMD[i],velCMD[i]);
	}
	
}

void IP_Mode_SendCommandPos(canDriver *driver,int32_t *posCMD)
{
    for(uint8_t i=0;i<ManipulatorState.NumOfJoint;i++){
        IP_Send_Position(driver, ManipulatorState.NodeID[i], posCMD[i]);
    }

}

void PDO_Set_TargetTorque(canDriver *driver, int16_t targetTorque){

    IP_Send_Torque(driver, ManipulatorState.NodeID[6], targetTorque);
}

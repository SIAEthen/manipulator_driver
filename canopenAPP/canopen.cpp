/*
 * canopen.c
 *
 *  Created on: Oct 11, 2021
 *      Author: maxiufeng
 */

#include "canopen.h"
#include "string.h"
#include "math.h"
#include "stdio.h"
#include "GlobalVariables.h"





/** CAN RX FIFO0 接收回调函数 处理CAN接收
  * @brief  Rx FIFO 0 message pending callback.
  * @param  driver pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
int CANOPEN_Receieve_Msg(canDriver *driver)
{	//printf("use interrupt\n");
		// if(!driver->canDriverReadSync(1)) {  

		// } //timeout error //等待下一帧消息，最多等待1ms
		if (canReadWaitFlag){
			if (!driver->canDriverReadWait(&canRxHeader.StdId,canRxData,&canRxHeader.DLC,&canRxHeader.flags,1)) return 0;
		}else{
			if (!driver->canDriverRead(&canRxHeader.StdId,canRxData,&canRxHeader.DLC,&canRxHeader.flags)) return 0;
		}
		
		/* * * * * * */
		uint32_t COBID_fc 				= 	canRxHeader.StdId & FC_MASK;
		canRxDataStruct.m_NodeID 		= (canRxHeader.StdId & ID_MASK);
		canRxDataStruct.m_DLC	  		= canRxHeader.DLC;
		memcpy(canRxDataStruct.m_CanRxData, canRxData, canRxHeader.DLC);


		if (COBID_fc == FC_EMERGENCY)
			canRxDataTypeFlag.m_EMYC_flag = 1;	// 收到紧急消息报文 更改数据就绪标志位
			memcpy(&canEmergencyDataStruct, canRxData,canRxHeader.DLC);

		if (COBID_fc == FC_TSDO)
			canRxDataTypeFlag.m_TSDO_flag = 1;	// 收到从机的SDO响应 更改数据就绪标志位
		if (COBID_fc == FC_TPDO1)
		{
			CAN_Rx_TPDOx_Data_Handle(&tpdo1DataStruct);
			canRxDataTypeFlag.m_TPDO1_flag = 1; // 收到从机的TPDO1数据 更改数据就绪标志位

		}
		if (COBID_fc == FC_TPDO2)
		{
			CAN_Rx_TPDOx_Data_Handle(&tpdo2DataStruct);
			canRxDataTypeFlag.m_TPDO2_flag = 1; // 收到从机的TPDO2数据 更改数据就绪标志位
		}

		if (COBID_fc == FC_ERRCONTROL_NODEGUARD)			// 节点错误和防护响应 更改数据就绪
			canRxDataTypeFlag.m_NODE_GUARD_flag = 1;
		/* * * * * * */
		return 1;
}


/**
  * @brief  handle receive data and category data type
  *
  * @note   NodeID [1-127].
  *
  * @param  NodeID: 	@uint16_t.
  * @retval int
 */
int CAN_Rx_Data_Handle(canDriver *driver, uint16_t NodeID)
{
	 if(!CANOPEN_Receieve_Msg(driver)){
		return 0;
	 }

	canRxWaitTime = 2;	// 超时等待，添加到 stm32f4xxx_it.c 中 递减计时 ms 单位

	while ((!canRxDataTypeFlag.m_TSDO_flag &&
			!canRxDataTypeFlag.m_NODE_GUARD_flag &&
			!canRxDataTypeFlag.m_EMYC_flag &&
			!canRxDataTypeFlag.m_TPDO1_flag &&
			!canRxDataTypeFlag.m_TPDO2_flag &&
			!canRxDataTypeFlag.m_TPDO3_flag &&
			!canRxDataTypeFlag.m_TPDO4_flag   ) && canRxWaitTime);	// 判断接收是什么类型数据，同时添加超时等待
	if (canRxWaitTime > 0 && canRxDataStruct.m_NodeID == NodeID)
	{
		canRxWaitTime = 0;
		RxResult = 0;
		if (1 == canRxDataTypeFlag.m_TSDO_flag)
		{	// 快速SDO 收到从机响应标志
			canRxDataTypeFlag.m_TSDO_flag = 0;
			if ((canRxDataStruct.m_CanRxData[0] & SDO_CS_MASK) == SDO_FAST_DOWN_SCS)
			{	// 快速下载SDO 从机的应答
				int i = 1;
				for (int var = 4; var < canRxDataStruct.m_DLC; ++var) {
					RxResult |= (canRxDataStruct.m_CanRxData[canRxDataStruct.m_DLC-i] << 8*(canRxDataStruct.m_DLC-4-i));
				}
				return 1;
			}
			else if((canRxDataStruct.m_CanRxData[0] & SDO_CS_MASK) == SDO_FAST_UPLOAD_SCS)
			{	// 快速上载SDO 从机的应答
				int i = 1;
				for (int var = 4; var < canRxDataStruct.m_DLC; ++var) {
					RxResult |= (canRxDataStruct.m_CanRxData[canRxDataStruct.m_DLC-i] << 8*(canRxDataStruct.m_DLC-4-i));
					i++;
				}
				return 2;
			}
			else if((canRxDataStruct.m_CanRxData[0] & SDO_CS_MASK) == SDO_SEGMENT_UPLOAD_SCS)
			{

				return 6;
			}
			else if ((canRxDataStruct.m_CanRxData[0] & SDO_CS_MASK) == SDO_ABORT_CS)
			{	// 快速下载或上载SDO 从机发出终止应答 数据段为错误代码
				int i = 1;
				for (int var = 4; var < canRxDataStruct.m_DLC; ++var) {
					RxResult |= (canRxDataStruct.m_CanRxData[canRxDataStruct.m_DLC-i] << 8*(canRxDataStruct.m_DLC-4-i));
				}
				return 3;
			}
		}
		else if (1 == canRxDataTypeFlag.m_EMYC_flag)
		{	// 紧急消息紧急响应 数据8字节全是错误代码等信息 用了前 4 个字节， Byte |0-1 errorCode|2 errorRegister|3 elmoCode|
			canRxDataTypeFlag.m_EMYC_flag = 0;
			for (int var = 0; var < canRxDataStruct.m_DLC-4; ++var) {
				RxResult |= (canRxDataStruct.m_CanRxData[canRxDataStruct.m_DLC-5-var] << 8*(canRxDataStruct.m_DLC-5-var));
			}
			return 4;
		}
//		else if (1 == canRxDataTypeFlag.m_NODE_GUARD_flag && canReceiveDataStruct.m_DLC == 1) {
//			canRxDataTypeFlag.m_NODE_GUARD_flag = 0;
//			RxResult = canReceiveDataStruct.m_CanRxData[0];
//			return 5;
//		}
		else if(1 == canRxDataTypeFlag.m_TPDO1_flag)
		{	// TPDO1 的数据接收 最多8个字节
			canRxDataTypeFlag.m_TPDO1_flag = 0;
			for (int var = 0; var < canRxDataStruct.m_DLC; ++var) {
				RxResult |= canRxDataStruct.m_CanRxData[var];
			}
			return 5;
		}
		else if(1 == canRxDataTypeFlag.m_TPDO2_flag)
		{	// TPDO1 的数据接收 最多8个字节
			canRxDataTypeFlag.m_TPDO2_flag = 0;
			for (int var = 0; var < canRxDataStruct.m_DLC; ++var) {
				RxResult |= canRxDataStruct.m_CanRxData[var];
			}
			return 6;
		}
	}
	else {
		return 0;
	}
	return 0;
}


/* can 接收中断里用这个接收TPDO的数据信息 */
void CAN_Rx_TPDOx_Data_Handle(TPDODatasTypeDef *tpdoxDataStruct)
{

	uint8_t i = 0;
	for (i = 0; i < ManipulatorState.NumOfMotor ; i++)
	{
		if (ManipulatorState.NodeID[i] == canRxDataStruct.m_NodeID )
		{
			// 把pdo的值存进来
			for (uint8_t idx = 0; idx < canRxDataStruct.m_DLC; idx++)
			{
				tpdoxDataStruct->tpdoData[i].m_CanRxData[idx] = canRxDataStruct.m_CanRxData[idx];
			}
			tpdoxDataStruct->tpdoData[i].m_DLC = canRxDataStruct.m_DLC;
			tpdoxDataStruct->tpdoData[i].m_NodeID = canRxDataStruct.m_NodeID;
			break;
		}
	}
}


/* ************************↓↓↓ 一些实际用到的功能 与驱动器对象 [写入] 相关的 SDO服务数据方式 ↓↓↓************************ */
/* SDO方式控制驱动器的数字量输出端口 高、低电平 每次控制一个DO口 */
int CAN_SDO_Digital_Output(canDriver *driver, uint16_t NodeID, DOValueTypeDef DOVal, uint64_t *resVal)
{
	uint8_t sendData[4] = {DOVal, 0, 0, 0};

	// 先读当前的DO对象的值
	uint64_t DOstatus = 0;
	CAN_SDO_Digital_Output_Status(driver, NodeID, &DOstatus);

	if (0 == DOstatus)	sendData[0] = DOVal;	// 当前是全关闭的，直接按照设定DO口操作，有些多余，下边的可以实现这个功能，这行也保留了。
	else {
		switch (DOVal) {
			case DO_1_5V_LEAD_ON:
			case DO_2_5V_LEAD_ON:
			case DO_3_3V3_NO_ON:
			case DO_4_3V3_BRAKE_ON:
				sendData[0] = (uint8_t)(DOstatus)|DOVal;	// 打开指定的DO口
				break;
			case DO_1_5V_LEAD_OFF:
			case DO_2_5V_LEAD_OFF:
			case DO_3_3V3_NO_OFF:
			case DO_4_3V3_BRAKE_OFF:
				sendData[0] = (uint8_t)(DOstatus)&DOVal;	// 关闭指定的DO口
				break;
			default:
				sendData[0] = 0x00;	// 所有的DO都关了
				break;
		}
	} // 经过处理的DO控制字节，确保更改其中某一位不会影响到其他位的状态

	// 设置指定的DOx打开 或 关闭
	if (0 == CAN_SDO_Fast_Write(driver, NodeID, &OBJ_Digital_Output, sendData))
		return 0;
	int res = CAN_Rx_Data_Handle(driver,NodeID);
	if (0 == res)
		return 0;
	else if (4 == res || 3 == res)
	{
		*resVal = RxResult;	// 存放错误代码
		return -1;
	}
	else if (1 == res && RxResult == 0)
	{
		*resVal = RxResult;
		if (4 == CAN_Rx_Data_Handle(driver,NodeID))
		{
			*resVal = RxResult;	// 存放错误代码
			return -1;
		}
		return 1;
	}
	return 0;
}

/* SDO方式写驱动器控制字对象 切换驱动器的状态 返回为是否正常标记、TSDO里的数据字节部分 DS402 */
int CAN_SDO_ControlWord(canDriver *driver, uint16_t NodeID, CWValTypeDef CWVal, uint16_t *resVal)
{
	uint8_t sendData[4] = {CWVal & 0xff, (CWVal >> 8)&0xff, 00, 00};	// SDO待发送数据存放，和发送SDO写参数部分配合

	/* 快速SDO写控制字 更改控制字的值 实现功能的切换 */
	if(0 == CAN_SDO_Fast_Write(driver, NodeID, &OBJ_Control_Word, sendData))
		return 0;
	int res = CAN_Rx_Data_Handle(driver,NodeID);
	if (0 == res)	// 通信上的超时标志
	{
		return 0;
	}
	else if (4 == res || 3 == res)	// SDO 终止传输 或 EMCY消息标志
	{
		*resVal = RxResult;
		return -1;
	}
	else if (1 == res && RxResult == 0)	// 有效的SDO响应 且 无错误
	{
		*resVal = RxResult;
		if (4 == CAN_Rx_Data_Handle(driver,NodeID)) {	// 在有效的SDO响应之后如果有SDO终止响应标志
			*resVal = RxResult;
			return -1;
		}
		return 1;
	}
	else {
		return 0;
	}

	return 0;
}

/* 写驱动器运动模式对象 设定具体的运动模式 */
int CAN_SDO_Motion_Mode(canDriver *driver, uint16_t NodeID, MotionModeTypeDef motionMode, uint32_t *resVal)
{
	uint8_t sendData[4] = {motionMode & 0xff, 00, 00, 00};	// SDO待发送数据存放，和发送SDO写参数部分配合

	/* 快速SDO写控制字 更改控制字的值 实现功能的切换 */
	if(0 == CAN_SDO_Fast_Write(driver, NodeID, &OBJ_Operation_Modes, sendData))
		return 0;
	int res = CAN_Rx_Data_Handle(driver,NodeID);
	if (0 == res)	// 通信上的超时标志
	{
		return 0;
	}
	else if (4 == res || 3 == res)	// SDO 终止传输 或 EMCY消息标志
	{
		*resVal = RxResult;
		return -1;
	}
	else if (1 == res && RxResult == 0)	// 有效的SDO响应 且 无错误
	{
		*resVal = RxResult;
		if (4 == CAN_Rx_Data_Handle(driver,NodeID)) {	// 在有效的SDO响应之后如果有SDO终止响应标志
			*resVal = RxResult;
			return -1;
		}
		return 1;
	}
	else {
		return 0;
	}

	return 0;
}

/* 写驱动器 快速停止选项代码 Quick stop option code 1 2 5 6 */
int CAN_SDO_Quick_Stop_Option_Code(canDriver *driver, uint16_t NodeID, int16_t qsoc, uint32_t *resVal)
{
	uint8_t sendData[4] = {qsoc & 0xFF, (qsoc>>8)&0xFF, 00, 00};	// SDO待发送数据存放，和发送SDO写参数部分配合

	/* 快速SDO写控制字 更改控制字的值 实现功能的切换 */
	if(0 == CAN_SDO_Fast_Write(driver, NodeID, &OBJ_Quick_Stop_Option_Code, sendData))
		return 0;
	int res = CAN_Rx_Data_Handle(driver,NodeID);
	if (0 == res)	// 通信上的超时标志
	{
		return 0;
	}
	else if (4 == res || 3 == res)	// SDO 终止传输 或 EMCY消息标志
	{
		*resVal = RxResult;
		return -1;
	}
	else if (1 == res && RxResult == 0)	// 有效的SDO响应 且 无错误
	{
		*resVal = RxResult;
		if (4 == CAN_Rx_Data_Handle(driver,NodeID)) {	// 在有效的SDO响应之后如果有SDO终止响应标志
			*resVal = RxResult;
			return -1;
		}
		return 1;
	}
	else {
		return 0;
	}

	return 0;
}

/* 写驱动器 位置控制功能里的 Positioning Option Code 用过个时使用或操作进行 */
int CAN_SDO_Position_Option_Code(canDriver *driver, uint16_t NodeID, POCTypeDef poc, uint32_t *resVal)
{
	uint8_t sendData[4] = {poc & 0xFF, (poc>>8)&0xFF, 00, 00};	// SDO待发送数据存放，和发送SDO写参数部分配合

	/* 快速SDO写控制字 更改控制字的值 实现功能的切换 */
	if(0 == CAN_SDO_Fast_Write(driver, NodeID, &OBJ_Positioning_Option_Code, sendData))
		return 0;
	int res = CAN_Rx_Data_Handle(driver,NodeID);
	if (0 == res)	// 通信上的超时标志
	{
		return 0;
	}
	else if (4 == res || 3 == res)	// SDO 终止传输 或 EMCY消息标志
	{
		*resVal = RxResult;
		return -1;
	}
	else if (1 == res && RxResult == 0)	// 有效的SDO响应 且 无错误
	{
		*resVal = RxResult;
		if (4 == CAN_Rx_Data_Handle(driver,NodeID)) {	// 在有效的SDO响应之后如果有SDO终止响应标志
			*resVal = RxResult;
			return -1;
		}
		return 1;
	}
	else {
		return 0;
	}

	return 0;
}


/* ************** 设置Profile Position 的有关参数 ********************
 	[#] 设定点设置 由控制字 set-point位、change set immediately位的时序(顺序) 和 状态字的set-point acknowledge位控制
 	[#] change set immediately = 1: 单个设定点, 驱动器立即处理接收到的新的目标位置，产生新的运动。
 		到达目标位置，速度降到0，状态字target reached=1，驱动器准备好接收新的设定点。
 	[#] change set immediately = 0: 设定点集,  设定点的缓冲可用，驱动器有 4 个设定点缓冲器，两种选择
 		[!] blended=1 驱动器到一个设定点速度通常不会降为 0
 		[!] blended=0 驱动器到一个设定点速度就会降为 0，每个点都如此
 	    以上两种情况，驱动器达到最后一个设定点状态字的 target reached = 1
 	 [#] 设置和激活新设定点的顺序是：
 	 	 |- 1.主机发送轨迹数据
 	 	 |- 2.主机验证数据，通过 控制字的 new set-point = 1 启动运动
 	 	 |- 3.驱动器设置状态字的 set-point scknowledge =1 ，做出响应
 	 	 |- 4.开始运动
 	 	 |- 5.主机设置控制子的 new set-point = 0，如果驱动器能够接收更多的 设定点，驱动器的 set-point acknowledge = 0 (通过60F2可进行配置)
 	 	 |- 6.*/
/* SDO方式写驱动器 PP 模式下，设置目标位置 */
int CAN_SDO_PP_Target_Position(canDriver *driver, uint16_t NodeID, int32_t posVal, uint32_t *resVal)
{
	uint8_t sendData[4] = {posVal&0xFF, (posVal>>8)&0xFF, (posVal>>16)&0xFF, (posVal>>24)&0xFF};
	// 设置指定的DOx打开 或 关闭
	if (0 == CAN_SDO_Fast_Write(driver, NodeID, &OBJ_PP_Target_Pos, sendData))
		return 0;
	int res = CAN_Rx_Data_Handle(driver,NodeID);
	if (0 == res)
		return 0;
	else if (4 == res || 3 == res)
	{
		*resVal = RxResult;	// 存放错误代码
		return -1;
	}
	else if (1 == res && RxResult == 0)
	{
		*resVal = RxResult;
		if (4 == CAN_Rx_Data_Handle(driver,NodeID))
		{
			*resVal = RxResult;	// 存放错误代码
			return -1;
		}
		return 1;
	}
	return 0;
}

/* SDO方式写驱动器 PP 模式下，设置轮廓速度 */
int CAN_SDO_PP_Profile_Velocity(canDriver *driver, uint16_t NodeID, uint32_t velVal, uint32_t *resVal)
{
	uint8_t sendData[4] = {velVal&0xFF, (velVal>>8)&0xFF, (velVal>>16)&0xFF, (velVal>>24)&0xFF};
	// 设置指定的DOx打开 或 关闭
	if (0 == CAN_SDO_Fast_Write(driver, NodeID, &OBJ_PP_Profile_Velocity, sendData))
		return 0;
	int res = CAN_Rx_Data_Handle(driver,NodeID);
	if (0 == res)
		return 0;
	else if (4 == res || 3 == res)
	{
		*resVal = RxResult;	// 存放错误代码
		return -1;
	}
	else if (1 == res && RxResult == 0)
	{
		*resVal = RxResult;
		if (4 == CAN_Rx_Data_Handle(driver,NodeID))
		{
			*resVal = RxResult;	// 存放错误代码
			return -1;
		}
		return 1;
	}
	return 0;
}

/* SDO方式写驱动器 PP 模式下，设置轮廓加速度 */
int CAN_SDO_PP_Profile_Acceleration(canDriver *driver, uint16_t NodeID, uint32_t accVal, uint32_t *resVal)
{
	uint8_t sendData[4] = {accVal&0xFF, (accVal>>8)&0xFF, (accVal>>16)&0xFF, (accVal>>24)&0xFF};
	// 设置指定的DOx打开 或 关闭
	if (0 == CAN_SDO_Fast_Write(driver, NodeID, &OBJ_PP_Profile_Acc, sendData))
		return 0;
	int res = CAN_Rx_Data_Handle(driver,NodeID);
	if (0 == res)
		return 0;
	else if (4 == res || 3 == res)
	{
		*resVal = RxResult;	// 存放错误代码
		return -1;
	}
	else if (1 == res && RxResult == 0)
	{
		*resVal = RxResult;
		if (4 == CAN_Rx_Data_Handle(driver,NodeID))
		{
			*resVal = RxResult;	// 存放错误代码
			return -1;
		}
		return 1;
	}
	return 0;
}

/* SDO方式写驱动器 PP 模式下，设置轮廓减速度 */
int CAN_SDO_PP_Profile_Deceleration(canDriver *driver, uint16_t NodeID, uint32_t decVal, uint32_t *resVal)
{
	uint8_t sendData[4] = {decVal&0xFF, (decVal>>8)&0xFF, (decVal>>16)&0xFF, (decVal>>24)&0xFF};
	// 设置指定的DOx打开 或 关闭
	if (0 == CAN_SDO_Fast_Write(driver, NodeID, &OBJ_PP_Profile_Dec, sendData))
		return 0;
	int res = CAN_Rx_Data_Handle(driver,NodeID);
	if (0 == res)
		return 0;
	else if (4 == res || 3 == res)
	{
		*resVal = RxResult;	// 存放错误代码
		return -1;
	}
	else if (1 == res && RxResult == 0)
	{
		*resVal = RxResult;
		if (4 == CAN_Rx_Data_Handle(driver,NodeID))
		{
			*resVal = RxResult;	// 存放错误代码
			return -1;
		}
		return 1;
	}
	return 0;
}

/* SDO方式写驱动器 PP 模式下，设置快速停止减速度 */
int CAN_SDO_PP_Quick_Stop_Deceleration(canDriver *driver, uint16_t NodeID, uint32_t decVal, uint32_t *resVal)
{
	uint8_t sendData[4] = {decVal&0xFF, (decVal>>8)&0xFF, (decVal>>16)&0xFF, (decVal>>24)&0xFF};
	// 设置指定的DOx打开 或 关闭
	if (0 == CAN_SDO_Fast_Write(driver, NodeID, &OBJ_PP_QUICK_STOP_Dec, sendData))
		return 0;
	int res = CAN_Rx_Data_Handle(driver,NodeID);
	if (0 == res)
		return 0;
	else if (4 == res || 3 == res)
	{
		*resVal = RxResult;	// 存放错误代码
		return -1;
	}
	else if (1 == res && RxResult == 0)
	{
		*resVal = RxResult;
		if (4 == CAN_Rx_Data_Handle(driver,NodeID))
		{
			*resVal = RxResult;	// 存放错误代码
			return -1;
		}
		return 1;
	}
	return 0;
}

/* ************** 设置Profile Velocity 的有关参数 ******************** */
int CAN_SDO_PV_Target_Velocity(canDriver *driver, uint16_t NodeID, int32_t velVal, uint32_t *resVal)
{
	uint8_t sendData[4] = {velVal&0xFF, (velVal>>8)&0xFF, (velVal>>16)&0xFF, (velVal>>24)&0xFF};
	// 设置指定的DOx打开 或 关闭
	if (0 == CAN_SDO_Fast_Write(driver, NodeID, &OBJ_PV_Target_Velocity, sendData))
		return 0;
	int res = CAN_Rx_Data_Handle(driver,NodeID);
	if (0 == res)
		return 0;
	else if (4 == res || 3 == res)
	{
		*resVal = RxResult;	// 存放错误代码
		return -1;
	}
	else if (1 == res && RxResult == 0)
	{
		*resVal = RxResult;
		if (4 == CAN_Rx_Data_Handle(driver,NodeID))
		{
			*resVal = RxResult;	// 存放错误代码
			return -1;
		}
		return 1;
	}
	return 0;
}
/* SDO方式写驱动器 PV 模式下，设置轮廓加速度 */
int CAN_SDO_PV_Profile_Acceleration(canDriver *driver, uint16_t NodeID, uint32_t accVal, uint32_t *resVal)
{
	uint8_t sendData[4] = {accVal&0xFF, (accVal>>8)&0xFF, (accVal>>16)&0xFF, (accVal>>24)&0xFF};
	// 设置指定的DOx打开 或 关闭
	if (0 == CAN_SDO_Fast_Write(driver, NodeID, &OBJ_PV_Profile_Acc, sendData))
		return 0;
	int res = CAN_Rx_Data_Handle(driver,NodeID);
	if (0 == res)
		return 0;
	else if (4 == res || 3 == res)
	{
		*resVal = RxResult;	// 存放错误代码
		return -1;
	}
	else if (1 == res && RxResult == 0)
	{
		*resVal = RxResult;
		if (4 == CAN_Rx_Data_Handle(driver,NodeID))
		{
			*resVal = RxResult;	// 存放错误代码
			return -1;
		}
		return 1;
	}
	return 0;
}
/* SDO方式写驱动器 PV 模式下，设置轮廓减速度 */
int CAN_SDO_PV_Profile_Deceleration(canDriver *driver, uint16_t NodeID, uint32_t decVal, uint32_t *resVal)
{
	uint8_t sendData[4] = {decVal&0xFF, (decVal>>8)&0xFF, (decVal>>16)&0xFF, (decVal>>24)&0xFF};
	// 设置指定的DOx打开 或 关闭
	if (0 == CAN_SDO_Fast_Write(driver, NodeID, &OBJ_PV_Profile_Dec, sendData))
		return 0;
	int res = CAN_Rx_Data_Handle(driver,NodeID);
	if (0 == res)
		return 0;
	else if (4 == res || 3 == res)
	{
		*resVal = RxResult;	// 存放错误代码
		return -1;
	}
	else if (1 == res && RxResult == 0)
	{
		*resVal = RxResult;
		if (4 == CAN_Rx_Data_Handle(driver,NodeID))
		{
			*resVal = RxResult;	// 存放错误代码
			return -1;
		}
		return 1;
	}
	return 0;
}
/* SDO方式写驱动器 PV 模式下，设置快速停止减速度 */
int CAN_SDO_PV_Quick_Stop_Deceleration(canDriver *driver, uint16_t NodeID, uint32_t decVal, uint32_t *resVal)
{
	uint8_t sendData[4] = {decVal&0xFF, (decVal>>8)&0xFF, (decVal>>16)&0xFF, (decVal>>24)&0xFF};
	// 设置指定的DOx打开 或 关闭
	if (0 == CAN_SDO_Fast_Write(driver, NodeID, &OBJ_PV_QUICK_STOP_Dec, sendData))
		return 0;
	int res = CAN_Rx_Data_Handle(driver,NodeID);
	if (0 == res)
		return 0;
	else if (4 == res || 3 == res)
	{
		*resVal = RxResult;	// 存放错误代码
		return -1;
	}
	else if (1 == res && RxResult == 0)
	{
		*resVal = RxResult;
		if (4 == CAN_Rx_Data_Handle(driver,NodeID))
		{
			*resVal = RxResult;	// 存放错误代码
			return -1;
		}
		return 1;
	}
	return 0;
}


/* SDO方式写驱动器 PT 模式下，设置目标扭矩、电流 */
int CAN_SDO_PT_Target_Torque_Current(canDriver *driver, uint16_t NodeID, int16_t torqueVal, uint32_t *resVal)
{
	uint8_t sendData[4] = {torqueVal&0xFF, (torqueVal>>8)&0xFF, 0, 0};
	// 设置目标扭矩 这里不是实际的电流值 需要做完转换的值
	if (0 == CAN_SDO_Fast_Write(driver, NodeID, &OBJ_PT_Target_Torque, sendData))
		return 0;
	int res = CAN_Rx_Data_Handle(driver,NodeID);
	if (0 == res)
		return 0;
	else if (4 == res || 3 == res)
	{
		*resVal = RxResult;	// 存放错误代码
		return -1;
	}
	else if (1 == res && RxResult == 0)
	{
		*resVal = RxResult;
		if (4 == CAN_Rx_Data_Handle(driver,NodeID))
		{
			*resVal = RxResult;	// 存放错误代码
			return -1;
		}
		return 1;
	}
	return 0;
}


/* SDO方式写驱动器 IP 模式下，设置轮廓加速度 */
int CAN_SDO_IP_Profile_Acceleration(canDriver *driver, uint16_t NodeID, uint32_t accVal, uint32_t *resVal)
{
	uint8_t sendData[4] = {accVal&0xFF, (accVal>>8)&0xFF, (accVal>>16)&0xFF, (accVal>>24)&0xFF};
	// 设置指定的DOx打开 或 关闭
	if (0 == CAN_SDO_Fast_Write(driver, NodeID, &OBJ_IP_Profile_Acc, sendData))
		return 0;
	int res = CAN_Rx_Data_Handle(driver,NodeID);
	if (0 == res)
		return 0;
	else if (4 == res || 3 == res)
	{
		*resVal = RxResult;	// 存放错误代码
		return -1;
	}
	else if (1 == res && RxResult == 0)
	{
		*resVal = RxResult;
		if (4 == CAN_Rx_Data_Handle(driver,NodeID))
		{
			*resVal = RxResult;	// 存放错误代码
			return -1;
		}
		return 1;
	}
	return 0;
}
/* SDO方式写驱动器 IP 模式下，设置轮廓减速度 */
int CAN_SDO_IP_Profile_Deceleration(canDriver *driver, uint16_t NodeID, uint32_t decVal, uint32_t *resVal)
{
	uint8_t sendData[4] = {decVal&0xFF, (decVal>>8)&0xFF, (decVal>>16)&0xFF, (decVal>>24)&0xFF};
	// 设置指定的DOx打开 或 关闭
	if (0 == CAN_SDO_Fast_Write(driver, NodeID, &OBJ_IP_Profile_Dec, sendData))
		return 0;
	int res = CAN_Rx_Data_Handle(driver,NodeID);
	if (0 == res)
		return 0;
	else if (4 == res || 3 == res)
	{
		*resVal = RxResult;	// 存放错误代码
		return -1;
	}
	else if (1 == res && RxResult == 0)
	{
		*resVal = RxResult;
		if (4 == CAN_Rx_Data_Handle(driver,NodeID))
		{
			*resVal = RxResult;	// 存放错误代码
			return -1;
		}
		return 1;
	}
	return 0;
}
/* SDO方式写驱动器 IP 模式下，设置快速停止减速度 */
int CAN_SDO_IP_Quick_Stop_Deceleration(canDriver *driver, uint16_t NodeID, uint32_t decVal, uint32_t *resVal)
{
	uint8_t sendData[4] = {decVal&0xFF, (decVal>>8)&0xFF, (decVal>>16)&0xFF, (decVal>>24)&0xFF};
	// 设置指定的DOx打开 或 关闭
	if (0 == CAN_SDO_Fast_Write(driver, NodeID, &OBJ_IP_QUICK_STOP_Dec, sendData))
		return 0;
	int res = CAN_Rx_Data_Handle(driver,NodeID);
	if (0 == res)
		return 0;
	else if (4 == res || 3 == res)
	{
		*resVal = RxResult;	// 存放错误代码
		return -1;
	}
	else if (1 == res && RxResult == 0)
	{
		*resVal = RxResult;
		if (4 == CAN_Rx_Data_Handle(driver,NodeID))
		{
			*resVal = RxResult;	// 存放错误代码
			return -1;
		}
		return 1;
	}
	return 0;
}

/* SDO方式写驱动器 IP 模式下，设置IP子模式0、-1 */
int CAN_SDO_IP_Sub_Mode_Select(canDriver *driver, uint16_t NodeID, IPSubModeTypeDef subMode, uint32_t *resVal)
{
	uint8_t sendData[4] = {subMode&0xFF, (subMode>>8)&0xFF, 0, 0};
	// 设置目标扭矩 这里不是实际的电流值 需要做完转换的值
	if (0 == CAN_SDO_Fast_Write(driver, NodeID, &OBJ_IP_Sub_Mode_Select, sendData))
		return 0;
	int res = CAN_Rx_Data_Handle(driver,NodeID);
	if (0 == res)
		return 0;
	else if (4 == res || 3 == res)
	{
		*resVal = RxResult;	// 存放错误代码
		return -1;
	}
	else if (1 == res && RxResult == 0)
	{
		*resVal = RxResult;
		if (4 == CAN_Rx_Data_Handle(driver,NodeID))
		{
			*resVal = RxResult;	// 存放错误代码
			return -1;
		}
		return 1;
	}
	return 0;
}
/* SDO方式写驱动器 IP 模式下，设置IP的周期值 1-10ms */
int CAN_SDO_IP_Time_Record_Period(canDriver *driver, uint16_t NodeID, uint8_t period, uint32_t *resVal)
{
	uint8_t sendData[4] = {period&0xFF ,0 , 0, 0 };
	// 设置目标扭矩 这里不是实际的电流值 需要做完转换的值
	if (0 == CAN_SDO_Fast_Write(driver, NodeID, &OBJ_IP_Time_Record_Period_Units, sendData))
		return 0;
	int res = CAN_Rx_Data_Handle(driver,NodeID);
	if (0 == res)
		return 0;
	else if (4 == res || 3 == res)
	{
		*resVal = RxResult;	// 存放错误代码
		return -1;
	}
	else if (1 == res && RxResult == 0)
	{
		*resVal = RxResult;
		if (4 == CAN_Rx_Data_Handle(driver,NodeID))
		{
			*resVal = RxResult;	// 存放错误代码
			return -1;
		}
		return 1;
	}
	return 0;
}
/* SDO方式写驱动器 IP 模式下，设置IP的周期值的单位 如：10^-1 -2 -3 -4 s (-2 ~ -6) def = -3 -> 10^-3s=1ms */
int CAN_SDO_IP_Time_Record_Period_Index(canDriver *driver, uint16_t NodeID, IPTimeIndexTypeDef timeIndex, uint32_t *resVal)
{
	uint8_t sendData[4] = {timeIndex&0xFF ,0 , 0, 0 };
	// 设置目标扭矩 这里不是实际的电流值 需要做完转换的值
	if (0 == CAN_SDO_Fast_Write(driver, NodeID, &OBJ_IP_Time_Record_Period_Index, sendData))
		return 0;
	int res = CAN_Rx_Data_Handle(driver,NodeID);
	if (0 == res)
		return 0;
	else if (4 == res || 3 == res)
	{
		*resVal = RxResult;	// 存放错误代码
		return -1;
	}
	else if (1 == res && RxResult == 0)
	{
		*resVal = RxResult;
		if (4 == CAN_Rx_Data_Handle(driver,NodeID))
		{
			*resVal = RxResult;	// 存放错误代码
			return -1;
		}
		return 1;
	}
	return 0;
}
/* SDO方式写驱动器 IP 模式下，设置IP的buffer实际大小 (与子模式的值有关联 sub=0: 1, sub=-1: 1~16) */
int CAN_SDO_IP_Data_Config_Actual_Buf_Size(canDriver *driver, uint16_t NodeID, uint32_t ActualBufSize, uint32_t *resVal)
{
	uint8_t sendData[4] = {ActualBufSize&0xFF ,0 , 0, 0 };
	// 设置目标扭矩 这里不是实际的电流值 需要做完转换的值
	if (0 == CAN_SDO_Fast_Write(driver, NodeID, &OBJ_IP_Data_Config_Actual_Buf_Size, sendData))
		return 0;
	int res = CAN_Rx_Data_Handle(driver,NodeID);
	if (0 == res)
		return 0;
	else if (4 == res || 3 == res)
	{
		*resVal = RxResult;	// 存放错误代码
		return -1;
	}
	else if (1 == res && RxResult == 0)
	{
		*resVal = RxResult;
		if (4 == CAN_Rx_Data_Handle(driver,NodeID))
		{
			*resVal = RxResult;	// 存放错误代码
			return -1;
		}
		return 1;
	}
	return 0;
}
/* SDO方式写驱动器 IP 模式下，设置IP的buffer组织形式 */
int CAN_SDO_IP_Data_Config_Buf_Org(canDriver *driver, uint16_t NodeID, IPBufOrgTypeDef orgMode, uint32_t *resVal)
{
	uint8_t sendData[4] = {orgMode&0xFF ,0 , 0, 0 };
	// 设置目标扭矩 这里不是实际的电流值 需要做完转换的值
	if (0 == CAN_SDO_Fast_Write(driver, NodeID, &OBJ_IP_Data_Config_Buf_Org, sendData))
		return 0;
	int res = CAN_Rx_Data_Handle(driver,NodeID);
	if (0 == res)
		return 0;
	else if (4 == res || 3 == res)
	{
		*resVal = RxResult;	// 存放错误代码
		return -1;
	}
	else if (1 == res && RxResult == 0)
	{
		*resVal = RxResult;
		if (4 == CAN_Rx_Data_Handle(driver,NodeID))
		{
			*resVal = RxResult;	// 存放错误代码
			return -1;
		}
		return 1;
	}
	return 0;
}
/* SDO方式写驱动器 IP 模式下，设置IP的buffer的位置值 */
int CAN_SDO_IP_Data_Config_Buf_Pos(canDriver *driver, uint16_t NodeID, uint16_t bufPos, uint32_t *resVal)
{
	uint8_t sendData[4] = {bufPos&0xFF ,(bufPos>>8)&0xFF , 0, 0 };
	// 设置目标扭矩 这里不是实际的电流值 需要做完转换的值
	if (0 == CAN_SDO_Fast_Write(driver, NodeID, &OBJ_IP_Data_Config_Buf_Pos, sendData))
		return 0;
	int res = CAN_Rx_Data_Handle(driver,NodeID);
	if (0 == res)
		return 0;
	else if (4 == res || 3 == res)
	{
		*resVal = RxResult;	// 存放错误代码
		return -1;
	}
	else if (1 == res && RxResult == 0)
	{
		*resVal = RxResult;
		if (4 == CAN_Rx_Data_Handle(driver,NodeID))
		{
			*resVal = RxResult;	// 存放错误代码
			return -1;
		}
		return 1;
	}
	return 0;
}
/* SDO方式写驱动器 IP 模式下，设置IP的buffer控制对象 实现清空、禁用、允许访问操作 */
int CAN_SDO_IP_Data_Config_Buf_Control(canDriver *driver, uint16_t NodeID, IPBufCtrlTypeDef bufCtrl, uint32_t *resVal)
{
	uint8_t sendData[4] = {bufCtrl&0xFF ,0 , 0, 0 };
	// 设置目标扭矩 这里不是实际的电流值 需要做完转换的值
	if (0 == CAN_SDO_Fast_Write(driver, NodeID, &OBJ_IP_Data_Config_Buf_Clear, sendData))
		return 0;
	int res = CAN_Rx_Data_Handle(driver,NodeID);
	if (0 == res)
		return 0;
	else if (4 == res || 3 == res)
	{
		*resVal = RxResult;	// 存放错误代码
		return -1;
	}
	else if (1 == res && RxResult == 0)
	{
		*resVal = RxResult;
		if (4 == CAN_Rx_Data_Handle(driver,NodeID))
		{
			*resVal = RxResult;	// 存放错误代码
			return -1;
		}
		return 1;
	}
	return 0;
}

/* SDO方式写驱动器 电机停止前外推插值周期个数 1~32768 */
int CAN_SDO_IP_Extrapolation_Cycles_Timeout(canDriver *driver, uint16_t NodeID, int16_t cycleNum, uint32_t *resVal)
{
	uint8_t sendData[4] = {cycleNum&0xFF ,(cycleNum>>8)&0xFF , 0, 0 };
	// 设置目标值
	if (0 == CAN_SDO_Fast_Write(driver, NodeID, &OBJ_Extrapolation_Cycles_Timeout, sendData))
		return 0;
	int res = CAN_Rx_Data_Handle(driver,NodeID);
	if (0 == res)
		return 0;
	else if (4 == res || 3 == res)
	{
		*resVal = RxResult;	// 存放错误代码
		return -1;
	}
	else if (1 == res && RxResult == 0)
	{
		*resVal = RxResult;
		if (4 == CAN_Rx_Data_Handle(driver,NodeID))
		{
			*resVal = RxResult;	// 存放错误代码
			return -1;
		}
		return 1;
	}
	return 0;
}


/* **************************** ↓↓↓↓ [读] 取参数相关的SDO服务数据 函数方法  ↓↓↓↓  *****************************/
/* SDO段上载的实现 实现对 《厂商设备名》《厂商硬件版本》《厂商软件版本》的字符串读取 */
/**
 * @brief  RSDO 段读取功能，和节点反复几次握手，能传输n个字节
 *	[#] SDO fast upload R&T
 *	[#] SDO segment 1 upload R&T
 *	[#] SDO segment 2 upload R&T
 *			.....
 * @note   NodeID [1-127].
  * 		<FC_RSDO|NodeID>  [sdo_byte0 | od_index(2Bytes) | od_subindix | NULL(4Bytes)]
 * @param  driver		@CAN_HandleTypeDef
 * @param  NodeID: 	@uint16_t.
 * @param  pOd: 	@ODTypeDef.
 * @param  resArr:	@uint8_t[]. 读上来的指定对象的“字符串”结果, 给一个255大小的数组--建议这样
 * @retval   int
 */
int CAN_SDO_Segment_Read(canDriver *driver, uint16_t NodeID, const ODOBJTypeDef *pOd, uint8_t resArr[])
{
	uint8_t tempData[256] = {0};	//最多255个
	uint8_t readLen = 0;	// 要接收的字节数，最后和actualLen比较确认正确
	uint8_t actualLen = 0;	// 当前实际收到的数
	uint8_t stepLen = 0; //每次段读的数
	uint8_t stopFlag = 0;

	// 1.初始化SDO 快速上载服务 **********************************************
	if (0 == CAN_SDO_Fast_Read(driver, NodeID, pOd))
		return 0;
	int res = CAN_Rx_Data_Handle(driver,NodeID);
	if (0 == res)	// 通信上的超时标志
	{
		return 0;
	}
	else if (4 == res || 3 == res)	// SDO 终止传输 或 EMCY消息标志
	{
		return -1;
	}
	else if (2 == res)	// 有效的SDO响应 且 无错误
	{
		if (4 == CAN_Rx_Data_Handle(driver,NodeID)) {	// 在有效的SDO响应之后如果有SDO终止响应标志
			return -1;
		}
//		return 1;	// 第一次握手成功
		// 由SDO响应获取到待读取的字节数量
		readLen = (uint8_t)RxResult;

		while(1)
		{
			/* 每一个SDO段最多能读出7个字节的数据 */
			// 2.SDO 段奇数读取 **********************************************
			if (0 == CAN_SDO_Segment_Read_Odd(driver, NodeID))
			return 0;
			res = CAN_Rx_Data_Handle(driver,NodeID);
			if (0 == res)	// 通信上的超时标志
			{
				return 0;
			}
			else if (4 == res || 3 == res)	// SDO 终止传输 或 EMCY消息标志
			{
				return -1;
			}
			else if (6 == res)	// 有效的SDO段读取响应 且 无错误
			{
				stepLen = (canRxDataStruct.m_CanRxData[0]&0x0F)>>1;
				stepLen = (stepLen==0)?7:(7-stepLen);	// 当前这个段字节个数
				// /* debug */ printf("1->plan to read byte len = %d\r\n", stepLen);  //*********************************
				stopFlag  = (canRxDataStruct.m_CanRxData[0]&0x01);	//0:后续还有段传输 1:这就是最后一个
				// 数据接收进来，并更改实际数据计数
				for (int idx = 0; idx < stepLen; ++idx) {
					tempData[actualLen+idx] = canRxDataStruct.m_CanRxData[idx+1];
				}
				actualLen += stepLen;
				// /* debug */ printf("%s\r\n", tempData);						//*********************************
				//这是最后一个段的标志，结束段接收，
				if (1 == stopFlag)
				{
					if (actualLen == readLen)	// 读到的数据是有效的
					{
						strncpy((char *)resArr, (char *)tempData, actualLen);
						return 1;
					}
					else
					{
						return 0;
					}
				}
				// 结束标志不为1
				else {
					// 3.SDO 段偶数读取字节数据 **********************************************
					if (0 == CAN_SDO_Segment_Read_Even(driver, NodeID))
						return 0;
					res = CAN_Rx_Data_Handle(driver,NodeID);
					if (0 == res)	// 通信上的超时标志
					{
						return 0;
					}
					else if (4 == res || 3 == res)	// SDO 终止传输 或 EMCY消息标志
					{
						return -1;
					}
					else if (6 == res)
					{
						stepLen = (canRxDataStruct.m_CanRxData[0]&0x0F)>>1;
						stepLen = (stepLen==0)?7:(7-stepLen);	// 当前这个段字节个数
						// /* debug */ printf("2->plan to read byte len = %d\r\n", stepLen);  //*********************************
						stopFlag  = (canRxDataStruct.m_CanRxData[0]&0x01);	//0:后续还有段传输 1:这就是最后一个
						// 数据接收进来，并更改实际数据计数
						for (int idx = 0; idx < stepLen; ++idx) {
							tempData[actualLen+idx] = canRxDataStruct.m_CanRxData[idx+1];
						}
						actualLen += stepLen;
						// /* debug */ printf("%s\r\n", tempData);						//*********************************
						//这是最后一个段的标志，结束段接收，
						if (1 == stopFlag)
						{
							if (actualLen == readLen)	// 读到的数据是有效的
							{
								strncpy((char *)resArr, (char *)tempData, actualLen);
								return 1;
							}
							else
							{
								return 0;
							}
						}
						// 结束标志不为1
						else {
							continue;
						}
					}
				}
			}
		}	// while end


	}
	else {
		return 0;
	}

	return 0;
}

/* 读Digital Output的当前值 resVal 反馈当前Digital Output的状态值 在digital output里也用到了这个 */
int CAN_SDO_Digital_Output_Status(canDriver *driver, uint16_t NodeID, uint64_t *resVal)
{
	if ( 0 == CAN_SDO_Fast_Read(driver, NodeID, &OBJ_Digital_Output))
		return 0;
	int res = CAN_Rx_Data_Handle(driver,NodeID);
	if (0 == res)	// 通信上的超时标志
	{
		return 0;
	}
	else if (4 == res || 3 == res)	// SDO 终止传输 或 EMCY消息标志
	{
		*resVal = RxResult;
		return -1;
	}
	else if (2 == res)	// 有效的SDO响应 且 无错误
	{
		*resVal = RxResult;	// 接收当前的DO状态值
		if (4 == CAN_Rx_Data_Handle(driver,NodeID)) {	// 在有效的SDO响应之后如果有SDO终止响应标志
			*resVal = RxResult;
			return -1;
		}
		return 1;
	}
	return 0;
}

/* SDO读取驱动器5v电压值 mV单位， resval 为该值 */
int CAN_SDO_5V_DC_Supply(canDriver *driver, uint16_t NodeID, int16_t *resVal)
{
	if ( 0 == CAN_SDO_Fast_Read(driver, NodeID, &OBJ_5V_DC_Supply))
		return 0;
	int res = CAN_Rx_Data_Handle(driver,NodeID);
	if (0 == res)	// 通信上的超时标志
	{
		return 0;
	}
	else if (4 == res || 3 == res)	// SDO 终止传输 或 EMCY消息标志
	{
		*resVal = RxResult;
		return -1;
	}
	else if (2 == res)	// 有效的SDO响应 且 无错误
	{
		*resVal = RxResult;	// 接收当前的电压值
		if (4 == CAN_Rx_Data_Handle(driver,NodeID)) {	// 在有效的SDO响应之后如果有SDO终止响应标志
			*resVal = RxResult;
			return -1;
		}
		return 1;
	}
	return 0;
}

/* SDO读取驱动器48V电压值 mV单位，resval存放结果 */
int CAN_SDO_48V_DC_Supply(canDriver *driver, uint16_t NodeID, uint32_t *resVal)
{
	if ( 0 == CAN_SDO_Fast_Read(driver, NodeID, &OBJ_48V_DC_Voltage))
		return 0;
	int res = CAN_Rx_Data_Handle(driver,NodeID);
	if (0 == res)	// 通信上的超时标志
	{
		return 0;
	}
	else if (4 == res || 3 == res)	// SDO 终止传输 或 EMCY消息标志
	{
		*resVal = RxResult;
		return -1;
	}
	else if (2 == res)	// 有效的SDO响应 且 无错误
	{
		*resVal = RxResult;	// 接收当前的48v电压值
		if (4 == CAN_Rx_Data_Handle(driver,NodeID)) {	// 在有效的SDO响应之后如果有SDO终止响应标志
			*resVal = RxResult;
			return -1;
		}
		return 1;
	}
	return 0;
}

/* SDO读取驱动器48v回路上的电流值，结果放在resVal中 */
int CAN_SDO_Current_Value(canDriver *driver, uint16_t NodeID, int16_t *resVal)
{
	if ( 0 == CAN_SDO_Fast_Read(driver, NodeID, &OBJ_Current_Actual_Value))
		return 0;
	int res = CAN_Rx_Data_Handle(driver,NodeID);
	if (0 == res)	// 通信上的超时标志
	{
		return 0;
	}
	else if (4 == res || 3 == res)	// SDO 终止传输 或 EMCY消息标志
	{
		*resVal = RxResult;
		return -1;
	}
	else if (2 == res)	// 有效的SDO响应 且 无错误
	{
		*resVal = RxResult;	// 接收当前的48v回路电流值
		if (4 == CAN_Rx_Data_Handle(driver,NodeID)) {	// 在有效的SDO响应之后如果有SDO终止响应标志
			*resVal = RxResult;
			return -1;
		}
		return 1;
	}
	return 0;
}

/* SDO读取驱动器摄氏温度值 resVal接收温度值 */
int CAN_SDO_Drive_Temperature(canDriver *driver, uint16_t NodeID, uint16_t *resVal)
{
	if ( 0 == CAN_SDO_Fast_Read(driver, NodeID, &OBJ_Drive_Temperature))
		return 0;
	int res = CAN_Rx_Data_Handle(driver,NodeID);
	if (0 == res)	// 通信上的超时标志
	{
		return 0;
	}
	else if (4 == res || 3 == res)	// SDO 终止传输 或 EMCY消息标志
	{
		*resVal = RxResult;
		return -1;
	}
	else if (2 == res)	// 有效的SDO响应 且 无错误
	{
		*resVal = RxResult;	// 接收当前的驱动器温度值 摄氏度
		if (4 == CAN_Rx_Data_Handle(driver,NodeID)) {	// 在有效的SDO响应之后如果有SDO终止响应标志
			*resVal = RxResult;
			return -1;
		}
		return 1;
	}
	return 0;
}

/* SDO读取ADC通道1的电压值，转换成电机线圈的温度 */
int CAN_SDO_Analog_Mv_Temperature(canDriver *driver, uint16_t NodeID, int16_t *resVal)
{
	const int16_t volDivResistance   = 487;	 // 487欧姆 分压电阻
	const int16_t volRef		     = 3300; // 3v3(3300mV)参考供电电压
	#define READCOUNT  5
	uint8_t readCount	   			 = 0;
	int16_t volResistance[READCOUNT] = {0};

	/* 计算温度用到的方程系数 */
	/* T = P1 * V^2 + P2 * V + P3 */
	#define	P1	(-0.00006251)
	#define	P2	(0.2914)
	#define	P3	(-128.3)

nextWhile:
	if ( 0 == CAN_SDO_Fast_Read(driver, NodeID, &OBJ_Analog_mV))
		return 0;
	int res = CAN_Rx_Data_Handle(driver,NodeID);
	if (0 == res)	// 通信上的超时标志
	{
		return 0;
	}
	else if (4 == res || 3 == res)	// SDO 终止传输 或 EMCY消息标志
	{
		*resVal = RxResult;
		return -1;
	}
	else if (2 == res)	// 有效的SDO响应 且 无错误
	{
		// *resVal = RxResult;	// 接收当前的ADC1的值 Mv
		if (4 == CAN_Rx_Data_Handle(driver,NodeID)) {	// 在有效的SDO响应之后如果有SDO终止响应标志
			*resVal = RxResult;
			return -1;
		}
		/* 处理AI_1的数据计算成温度值 */

		volResistance[readCount++]		  = RxResult;	// AD测到的温度电阻上的电压值
/* debug *///		printf("analog 1 is : %d mV\r\n", volResistance[readCount]);	/*debug ************************************************/
		if (readCount < READCOUNT) {
			goto nextWhile;	// 跳转到给定标签处，控制循环次数
		}

		for (int var = 1; var < READCOUNT; ++var) {
			volResistance[0] += volResistance[var];
		}
		// 计算电阻的电压平均值
		float vol = (float)volResistance[0]/(float)READCOUNT;
		// 温度电阻的电阻值
		float Resistance  = volDivResistance * vol/(volRef - vol);
		// 阻值 - 温度 变换
		int16_t temperature = (P1*powf(Resistance, 2.0) + P2*Resistance + P3);
		*resVal = temperature;
		return 1;
	}
	return 0;
}

/* SDO读取设备类型对象，正常情况下返回的是固定值 @DEVICE_TYPE_VALUE */
int CAN_SDO_Device_Type(canDriver *driver, uint16_t NodeID, uint32_t *resVal)
{
	if ( 0 == CAN_SDO_Fast_Read(driver, NodeID, &OBJ_Device_Type))
		return 0;
	int res = CAN_Rx_Data_Handle(driver,NodeID);
	if (0 == res)	// 通信上的超时标志
	{
		return 0;
	}
	else if (4 == res || 3 == res)	// SDO 终止传输 或 EMCY消息标志
	{
		*resVal = RxResult;
		return -1;
	}
	else if (2 == res)	// 有效的SDO响应 且 无错误
	{
		*resVal = RxResult;	// 接收当前的设备类型值
		if (4 == CAN_Rx_Data_Handle(driver,NodeID)) {	// 在有效的SDO响应之后如果有SDO终止响应标志
			*resVal = RxResult;
			return -1;
		}
		return 1;
	}
	return 0;
}

/* SDO读取驱动器的状态字 并返回原始的 状态值 或 错误代码 结果, 打印控制位决定了是否将状态信息详情打印出来 */
int CAN_SDO_StatusWord(canDriver *driver, uint16_t NodeID, uint16_t *resVal, PrintInfoFlagTypeDef printFlag)
{
	if ( 0 == CAN_SDO_Fast_Read(driver, NodeID, &OBJ_Status_Word))
		return 0;
	int res = CAN_Rx_Data_Handle(driver,NodeID);   //res  can收发标志，程序内置参数
	if (0 == res)	// 通信上的超时标志
	{
		return 0;
	}
	else if (4 == res || 3 == res)	// SDO 终止传输 或 EMCY消息标志
	{
		*resVal = RxResult;
		return -1;
	}
	else if (2 == res)	// 有效的SDO响应 且 无错误
	{
		*resVal = RxResult;	// 接收当前设备状态值
		if (4 == CAN_Rx_Data_Handle(driver,NodeID)) {	// 在有效的SDO响应之后如果有SDO终止响应标志
			*resVal = RxResult;
			return -1;
		}
		/* 由打印信息标志位决定是否输出详细信息到串口 start */
		if (printFlag == PRINT_INFO_OFF)
			return 1;
		// 打印信息以字符串形式
		char infoStr[4][24] = {"null", "null", "null", "null"};	//0.状态机状态 1.48v电压状态 2.警告状态 3.到达目标位置标志
		/* 状态机当前状态 */
		uint16_t temp = RxResult&SW_MASK_1;	// 提取出 bit0~3,5,6位
		memset(infoStr[0],'\0',24);
		switch (temp) {
			case SW_SWITCH_ON_DISABLE:
				strcpy(infoStr[0], "Switch on Disable");
				break;
			case SW_READY_SWITCH_ON:
				strcpy(infoStr[0], "Ready to Switch on");
				break;
			case SW_SWITCH_ON:
				strcpy(infoStr[0], "Switch on");
				break;
			case SW_OPERATION_ENABLE:
				strcpy(infoStr[0], "Operation Enable");
				break;
			case SW_QUICK_STOP_ACTIVE:
				strcpy(infoStr[0], "Quick Stop Active");
				break;
			case SW_FAULT_REACTION_ACTIVE:
				strcpy(infoStr[0], "Fault Reaction Active");
				break;
			case SW_FAULT:
				strcpy(infoStr[0], "Fault");
				break;
			default:
				break;
		}
		/* 48V 电压状态 */
		if (RxResult&SW_VOLTAGE_ENABLE)	{ memset(infoStr[1],'\0',24); strcpy(infoStr[1], "Voltage Enable"); }
		else { memset(infoStr[1],'\0',24); strcpy(infoStr[1], "Voltage Disable"); }
		/* 警告状态 */
		if (RxResult&SW_WARNING) { memset(infoStr[2],'\0',24); strcpy(infoStr[2], "Warning"); }
		else { memset(infoStr[2],'\0',24); strcpy(infoStr[2], "NO Warning"); }
		/* 到达目标位置标志 */
		if (RxResult&SW_TARGET_REACHED) { memset(infoStr[3],'\0',24); strcpy(infoStr[3], "Reached"); }
		else { memset(infoStr[3],'\0',24); strcpy(infoStr[3], "Unreached"); }
		printf("\r\n* * * * [ID-%d]StatusWord Information * * * *\r\n"
			   "   state machine Value : %s\r\n"
			   "        voltage status : %s\r\n"
			   "         waring status : %s\r\n"
			   "        target reached : %s\r\n",
			   NodeID, infoStr[0], infoStr[1], infoStr[2], infoStr[3]);

		/* 由打印信息标志位决定是否输出详细信息到串口 end   */
	}
	return 0;
}

/* SDO读取驱动器控制字的当前值 返回该值 或 错误代码 结果 */
int CAN_SDO_ControlWord_Value(canDriver *driver, uint16_t NodeID, uint16_t *CWValue)
{
	if ( 0 == CAN_SDO_Fast_Read(driver, NodeID, &OBJ_Control_Word))
		return 0;
	int res = CAN_Rx_Data_Handle(driver,NodeID);
	if (0 == res)	// 通信上的超时标志
	{
		return 0;
	}
	else if (4 == res || 3 == res)	// SDO 终止传输 或 EMCY消息标志
	{
		*CWValue = RxResult;
		return -1;
	}
	else if (2 == res)	// 有效的SDO响应 且 无错误
	{
		if (4 == CAN_Rx_Data_Handle(driver,NodeID)) {	// 在有效的SDO响应之后如果有SDO终止响应标志
			*CWValue = RxResult;
			return -1;
		}
		*CWValue = RxResult;	// 接收当前控制字的值
/* debug *///		printf("Control word is : 0x%03x\r\n", RxResult);	// debug
		return 1;
	}
	return 0;
}

/* SDO读取驱动器当前的运动模式对象 返回该值 或 错误代码值 */
int CAN_SDO_Motion_Mode_Display(canDriver *driver, uint16_t NodeID, int32_t *MotionModeValue)
{
	if ( 0 == CAN_SDO_Fast_Read(driver, NodeID, &OBJ_Operation_Modes_Display))
		return 0;
	int res = CAN_Rx_Data_Handle(driver,NodeID);
	if (0 == res)	// 通信上的超时标志
	{
		return 0;
	}
	else if (4 == res || 3 == res)	// SDO 终止传输 或 EMCY消息标志
	{
		*MotionModeValue = RxResult;
		return -1;
	}
	else if (2 == res)	// 有效的SDO响应 且 无错误
	{
		if (4 == CAN_Rx_Data_Handle(driver,NodeID)) {	// 在有效的SDO响应之后如果有SDO终止响应标志
			*MotionModeValue = RxResult;
			return -1;
		}
		*MotionModeValue = RxResult;	// 接收当前运动模式的值
/* debug *///		printf("Motion Mode is : 0x%03x\r\n", MotionModeValue);	// debug
		return 1;
	}
	return 0;
}

/* SDO读驱动器 位置控制功能里的 Positioning Option Code 的值 和 PP 模式有很大关系 */
int CAN_SDO_Position_Option_Code_Value(canDriver *driver, uint16_t NodeID, uint16_t *pocValue)
{
	if ( 0 == CAN_SDO_Fast_Read(driver, NodeID, &OBJ_Operation_Modes_Display))
		return 0;
	int res = CAN_Rx_Data_Handle(driver,NodeID);
	if (0 == res)	// 通信上的超时标志
	{
		return 0;
	}
	else if (4 == res || 3 == res)	// SDO 终止传输 或 EMCY消息标志
	{
		*pocValue = RxResult;
		return -1;
	}
	else if (2 == res)	// 有效的SDO响应 且 无错误
	{
		if (4 == CAN_Rx_Data_Handle(driver,NodeID)) {	// 在有效的SDO响应之后如果有SDO终止响应标志
			*pocValue = RxResult;
			return -1;
		}
		*pocValue = RxResult;	// 接收当前的值
/* debug *///		printf("Motion Mode is : 0x%03x\r\n", MotionModeValue);	// debug
		return 1;
	}
	return 0;
}

/* SDO读驱动器 PP 模式下，设置的目标位置值 */
int CAN_SDO_PP_Target_Position_Value(canDriver *driver, uint16_t NodeID, int32_t *resVal)
{
	if ( 0 == CAN_SDO_Fast_Read(driver, NodeID, &OBJ_PP_Target_Pos))
		return 0;
	int res = CAN_Rx_Data_Handle(driver,NodeID);
	if (0 == res)	// 通信上的超时标志
	{
		return 0;
	}
	else if (4 == res || 3 == res)	// SDO 终止传输 或 EMCY消息标志
	{
		*resVal = RxResult;
		return -1;
	}
	else if (2 == res)	// 有效的SDO响应 且 无错误
	{
		if (4 == CAN_Rx_Data_Handle(driver,NodeID)) {	// 在有效的SDO响应之后如果有SDO终止响应标志
			*resVal = RxResult;
			return -1;
		}
		*resVal = RxResult;	// 接收设置的PP 下 目标位置值
/* debug *///		printf("PP target position is : 0x%d\r\n", *resVal);	// debug
		return 1;
	}
	return 0;
}

/* SDO读驱动器 PP 模式下，设置的轮廓速度值 */
int CAN_SDO_PP_Profile_Velocity_Value(canDriver *driver, uint16_t NodeID, uint32_t *resVal)
{
	if ( 0 == CAN_SDO_Fast_Read(driver, NodeID, &OBJ_PP_Profile_Velocity))
		return 0;
	int res = CAN_Rx_Data_Handle(driver,NodeID);
	if (0 == res)	// 通信上的超时标志
	{
		return 0;
	}
	else if (4 == res || 3 == res)	// SDO 终止传输 或 EMCY消息标志
	{
		*resVal = RxResult;
		return -1;
	}
	else if (2 == res)	// 有效的SDO响应 且 无错误
	{
		if (4 == CAN_Rx_Data_Handle(driver,NodeID)) {	// 在有效的SDO响应之后如果有SDO终止响应标志
			*resVal = RxResult;
			return -1;
		}
		*resVal = RxResult;	// 接收设置的PP 下 目标位置值
/* debug *///		printf("PP target position is : 0x%d\r\n", *resVal);	// debug
		return 1;
	}
	return 0;
}

/* SDO读驱动器 PP 模式下，设置的轮廓加速度值 */
int CAN_SDO_PP_Profile_Acceleration_Value(canDriver *driver, uint16_t NodeID, uint32_t *resVal)
{
	if ( 0 == CAN_SDO_Fast_Read(driver, NodeID, &OBJ_PP_Profile_Acc))
		return 0;
	int res = CAN_Rx_Data_Handle(driver,NodeID);
	if (0 == res)	// 通信上的超时标志
	{
		return 0;
	}
	else if (4 == res || 3 == res)	// SDO 终止传输 或 EMCY消息标志
	{
		*resVal = RxResult;
		return -1;
	}
	else if (2 == res)	// 有效的SDO响应 且 无错误
	{
		if (4 == CAN_Rx_Data_Handle(driver,NodeID)) {	// 在有效的SDO响应之后如果有SDO终止响应标志
			*resVal = RxResult;
			return -1;
		}
		*resVal = RxResult;	// 接收设置的PP 下 目标位置值
/* debug *///		printf("PP target position is : 0x%d\r\n", *resVal);	// debug
		return 1;
	}
	return 0;
}

/* SDO读驱动器 PP 模式下，设置的轮廓减速度值 */
int CAN_SDO_PP_Profile_Deceleration_Value(canDriver *driver, uint16_t NodeID, uint32_t *resVal)
{
	if ( 0 == CAN_SDO_Fast_Read(driver, NodeID, &OBJ_PP_Profile_Dec))
		return 0;
	int res = CAN_Rx_Data_Handle(driver,NodeID);
	if (0 == res)	// 通信上的超时标志
	{
		return 0;
	}
	else if (4 == res || 3 == res)	// SDO 终止传输 或 EMCY消息标志
	{
		*resVal = RxResult;
		return -1;
	}
	else if (2 == res)	// 有效的SDO响应 且 无错误
	{
		if (4 == CAN_Rx_Data_Handle(driver,NodeID)) {	// 在有效的SDO响应之后如果有SDO终止响应标志
			*resVal = RxResult;
			return -1;
		}
		*resVal = RxResult;	// 接收设置的PP 下 目标位置值
/* debug *///		printf("PP target position is : 0x%d\r\n", *resVal);	// debug
		return 1;
	}
	return 0;
}

/* SDO方式写驱动器 PP 模式下，设置快速停止减速度 */
int CAN_SDO_PP_Quick_Stop_Deceleration_Value(canDriver *driver, uint16_t NodeID, uint32_t *resVal)
{
	if ( 0 == CAN_SDO_Fast_Read(driver, NodeID, &OBJ_PP_QUICK_STOP_Dec))
		return 0;
	int res = CAN_Rx_Data_Handle(driver,NodeID);
	if (0 == res)	// 通信上的超时标志
	{
		return 0;
	}
	else if (4 == res || 3 == res)	// SDO 终止传输 或 EMCY消息标志
	{
		*resVal = RxResult;
		return -1;
	}
	else if (2 == res)	// 有效的SDO响应 且 无错误
	{
		if (4 == CAN_Rx_Data_Handle(driver,NodeID)) {	// 在有效的SDO响应之后如果有SDO终止响应标志
			*resVal = RxResult;
			return -1;
		}
		*resVal = RxResult;	// 接收设置的PP 下 目标位置值
/* debug *///		printf("PP target position is : 0x%d\r\n", *resVal);	// debug
		return 1;
	}
	return 0;
}

int CAN_SDO_PV_Target_Velocity_Value(canDriver *driver, uint16_t NodeID, int32_t *velVal)
{
	if ( 0 == CAN_SDO_Fast_Read(driver, NodeID, &OBJ_PV_Target_Velocity))
		return 0;
	int res = CAN_Rx_Data_Handle(driver,NodeID);
	if (0 == res)	// 通信上的超时标志
	{
		return 0;
	}
	else if (4 == res || 3 == res)	// SDO 终止传输 或 EMCY消息标志
	{
		*velVal = RxResult;
		return -1;
	}
	else if (2 == res)	// 有效的SDO响应 且 无错误
	{
		if (4 == CAN_Rx_Data_Handle(driver,NodeID)) {	// 在有效的SDO响应之后如果有SDO终止响应标志
			*velVal = RxResult;
			return -1;
		}
		*velVal = RxResult;	// 接收设置的PP 下 目标位置值
/* debug *///		printf("PV target velocity is : 0x%d\r\n", *velVal);	// debug
		return 1;
	}
	return 0;
}
/* SDO方式写驱动器 PV 模式下，设置轮廓加速度 */
int CAN_SDO_PV_Profile_Acceleration_Value(canDriver *driver, uint16_t NodeID, uint32_t *accVal)
{
	if ( 0 == CAN_SDO_Fast_Read(driver, NodeID, &OBJ_PV_Profile_Acc))
		return 0;
	int res = CAN_Rx_Data_Handle(driver,NodeID);
	if (0 == res)	// 通信上的超时标志
	{
		return 0;
	}
	else if (4 == res || 3 == res)	// SDO 终止传输 或 EMCY消息标志
	{
		*accVal = RxResult;
		return -1;
	}
	else if (2 == res)	// 有效的SDO响应 且 无错误
	{
		if (4 == CAN_Rx_Data_Handle(driver,NodeID)) {	// 在有效的SDO响应之后如果有SDO终止响应标志
			*accVal = RxResult;
			return -1;
		}
		*accVal = RxResult;	// 接收设置的PP 下 目标位置值
/* debug *///		printf("PV profile acceleration is : 0x%d\r\n", *accVal);	// debug
		return 1;
	}
	return 0;
}
/* SDO方式写驱动器 PV 模式下，设置轮廓减速度 */
int CAN_SDO_PV_Profile_Deceleration_Value(canDriver *driver, uint16_t NodeID, uint32_t *decVal)
{
	if ( 0 == CAN_SDO_Fast_Read(driver, NodeID, &OBJ_PV_Profile_Dec))
		return 0;
	int res = CAN_Rx_Data_Handle(driver,NodeID);
	if (0 == res)	// 通信上的超时标志
	{
		return 0;
	}
	else if (4 == res || 3 == res)	// SDO 终止传输 或 EMCY消息标志
	{
		*decVal = RxResult;
		return -1;
	}
	else if (2 == res)	// 有效的SDO响应 且 无错误
	{
		if (4 == CAN_Rx_Data_Handle(driver,NodeID)) {	// 在有效的SDO响应之后如果有SDO终止响应标志
			*decVal = RxResult;
			return -1;
		}
		*decVal = RxResult;	// 接收设置的PP 下 目标位置值
/* debug *///		printf("PV profile deceleration is : 0x%d\r\n", *decVal);	// debug
		return 1;
	}
	return 0;
}
/* SDO方式写驱动器 PV 模式下，设置快速停止减速度 */
int CAN_SDO_PV_Quick_Stop_Deceleration_Value(canDriver *driver, uint16_t NodeID, uint32_t *qdecVal)
{
	if ( 0 == CAN_SDO_Fast_Read(driver, NodeID, &OBJ_PV_QUICK_STOP_Dec))
		return 0;
	int res = CAN_Rx_Data_Handle(driver,NodeID);
	if (0 == res)	// 通信上的超时标志
	{
		return 0;
	}
	else if (4 == res || 3 == res)	// SDO 终止传输 或 EMCY消息标志
	{
		*qdecVal = RxResult;
		return -1;
	}
	else if (2 == res)	// 有效的SDO响应 且 无错误
	{
		if (4 == CAN_Rx_Data_Handle(driver,NodeID)) {	// 在有效的SDO响应之后如果有SDO终止响应标志
			*qdecVal = RxResult;
			return -1;
		}
		*qdecVal = RxResult;	// 接收设置的PP 下 目标位置值
/* debug *///		printf("PV quick stop deceleration is : 0x%d\r\n", *qdecVal);	// debug
		return 1;
	}
	return 0;
}

/* SDO方式读驱动器 PT 模式下，设置的目标扭矩、电流 需要做转换处理 */
int CAN_SDO_PT_Target_Torque_Current_Value(canDriver *driver, uint16_t NodeID, int16_t *torqueVal)
{
	if ( 0 == CAN_SDO_Fast_Read(driver, NodeID, &OBJ_PT_Target_Torque))
		return 0;
	int res = CAN_Rx_Data_Handle(driver,NodeID);
	if (0 == res)	// 通信上的超时标志
	{
		return 0;
	}
	else if (4 == res || 3 == res)	// SDO 终止传输 或 EMCY消息标志
	{
		*torqueVal = RxResult;
		return -1;
	}
	else if (2 == res)	// 有效的SDO响应 且 无错误
	{
		if (4 == CAN_Rx_Data_Handle(driver,NodeID)) {	// 在有效的SDO响应之后如果有SDO终止响应标志
			*torqueVal = RxResult;
			return -1;
		}
		*torqueVal = RxResult;	// 接收设置的PT 下 目标扭矩值
/* debug *///		printf("PT Target torque is : 0x%d\r\n", *torqueVal);	// debug
		return 1;
	}
	return 0;
}

/* SDO方式读驱动器 PT 模式下，电机的连续运行（额定）电流值 mA */
int CAN_SDO_PT_Motor_Rated_Current_Value(canDriver *driver, uint16_t NodeID, uint32_t *motorRatedCurrent)
{
	if ( 0 == CAN_SDO_Fast_Read(driver, NodeID, &OBJ_PT_Motor_Rated_Current))
		return 0;
	int res = CAN_Rx_Data_Handle(driver,NodeID);
	if (0 == res)	// 通信上的超时标志
	{
		return 0;
	}
	else if (4 == res || 3 == res)	// SDO 终止传输 或 EMCY消息标志
	{
		*motorRatedCurrent = RxResult;
		return -1;
	}
	else if (2 == res)	// 有效的SDO响应 且 无错误
	{
		if (4 == CAN_Rx_Data_Handle(driver,NodeID)) {	// 在有效的SDO响应之后如果有SDO终止响应标志
			*motorRatedCurrent = RxResult;
			return -1;
		}
		*motorRatedCurrent = RxResult;	// 接收设置的PT 下 目标扭矩值
/* debug *///		printf("PT Motor Rated Current is : 0x%d\r\n", *motorRatedCurrent);	// debug
		return 1;
	}
	return 0;
}


/* SDO方式写驱动器 IP 模式下，轮廓加速度 */
int CAN_SDO_IP_Profile_Acceleration_Value(canDriver *driver, uint16_t NodeID, uint32_t *accVal)
{
	if ( 0 == CAN_SDO_Fast_Read(driver, NodeID, &OBJ_IP_Profile_Acc))
		return 0;
	int res = CAN_Rx_Data_Handle(driver,NodeID);
	if (0 == res)	// 通信上的超时标志
	{
		return 0;
	}
	else if (4 == res || 3 == res)	// SDO 终止传输 或 EMCY消息标志
	{
		*accVal = RxResult;
		return -1;
	}
	else if (2 == res)	// 有效的SDO响应 且 无错误
	{
		if (4 == CAN_Rx_Data_Handle(driver,NodeID)) {	// 在有效的SDO响应之后如果有SDO终止响应标志
			*accVal = RxResult;
			return -1;
		}
		*accVal = RxResult;	// 接收设置的PP 下 目标位置值
/* debug *///		printf("PV profile acceleration is : 0x%d\r\n", *accVal);	// debug
		return 1;
	}
	return 0;
}
/* SDO方式写驱动器 IP 模式下，轮廓减速度 */
int CAN_SDO_IP_Profile_Deceleration_Value(canDriver *driver, uint16_t NodeID, uint32_t *decVal)
{
	if ( 0 == CAN_SDO_Fast_Read(driver, NodeID, &OBJ_IP_Profile_Dec))
		return 0;
	int res = CAN_Rx_Data_Handle(driver,NodeID);
	if (0 == res)	// 通信上的超时标志
	{
		return 0;
	}
	else if (4 == res || 3 == res)	// SDO 终止传输 或 EMCY消息标志
	{
		*decVal = RxResult;
		return -1;
	}
	else if (2 == res)	// 有效的SDO响应 且 无错误
	{
		if (4 == CAN_Rx_Data_Handle(driver,NodeID)) {	// 在有效的SDO响应之后如果有SDO终止响应标志
			*decVal = RxResult;
			return -1;
		}
		*decVal = RxResult;	// 接收设置的PP 下 目标位置值
/* debug *///		printf("PV profile deceleration is : 0x%d\r\n", *decVal);	// debug
		return 1;
	}
	return 0;
}
/* SDO方式写驱动器 IP 模式下，快速停止减速度 */
int CAN_SDO_IP_Quick_Stop_Deceleration_Value(canDriver *driver, uint16_t NodeID, uint32_t *qdecVal)
{
	if ( 0 == CAN_SDO_Fast_Read(driver, NodeID, &OBJ_IP_QUICK_STOP_Dec))
		return 0;
	int res = CAN_Rx_Data_Handle(driver,NodeID);
	if (0 == res)	// 通信上的超时标志
	{
		return 0;
	}
	else if (4 == res || 3 == res)	// SDO 终止传输 或 EMCY消息标志
	{
		*qdecVal = RxResult;
		return -1;
	}
	else if (2 == res)	// 有效的SDO响应 且 无错误
	{
		if (4 == CAN_Rx_Data_Handle(driver,NodeID)) {	// 在有效的SDO响应之后如果有SDO终止响应标志
			*qdecVal = RxResult;
			return -1;
		}
		*qdecVal = RxResult;	// 接收设置的PP 下 目标位置值
/* debug *///		printf("PV quick stop deceleration is : 0x%d\r\n", *qdecVal);	// debug
		return 1;
	}
	return 0;
}

/* SDO方式读驱动器 IP 模式下，IP子模式0、-1 */
int CAN_SDO_IP_Sub_Mode_Select_Value(canDriver *driver, uint16_t NodeID, IPSubModeTypeDef *subMode)
{
	if ( 0 == CAN_SDO_Fast_Read(driver, NodeID, &OBJ_IP_Sub_Mode_Select))
		return 0;
	int res = CAN_Rx_Data_Handle(driver,NodeID);
	if (0 == res)	// 通信上的超时标志
	{
		return 0;
	}
	else if (4 == res || 3 == res)	// SDO 终止传输 或 EMCY消息标志
	{
		*subMode = (IPSubModeTypeDef)RxResult;
		return -1;
	}
	else if (2 == res)	// 有效的SDO响应 且 无错误
	{
		if (4 == CAN_Rx_Data_Handle(driver,NodeID)) {	// 在有效的SDO响应之后如果有SDO终止响应标志
			*subMode = (IPSubModeTypeDef)RxResult;
			return -1;
		}
		*subMode = (IPSubModeTypeDef)RxResult;	// 接收设置的IP子模式的具体值
/* debug *///		printf("IP Sub Mode is: 0x%d\r\n", *subMode);	// debug
		return 1;
	}
	return 0;
}
/* SDO方式读驱动器 IP 模式下，IP的周期值 1-10ms */
int CAN_SDO_IP_Time_Record_Period_Value(canDriver *driver, uint16_t NodeID, uint8_t *period)
{
	if ( 0 == CAN_SDO_Fast_Read(driver, NodeID, &OBJ_IP_Time_Record_Period_Units))
		return 0;
	int res = CAN_Rx_Data_Handle(driver,NodeID);
	if (0 == res)	// 通信上的超时标志
	{
		return 0;
	}
	else if (4 == res || 3 == res)	// SDO 终止传输 或 EMCY消息标志
	{
		*period = RxResult;
		return -1;
	}
	else if (2 == res)	// 有效的SDO响应 且 无错误
	{
		if (4 == CAN_Rx_Data_Handle(driver,NodeID)) {	// 在有效的SDO响应之后如果有SDO终止响应标志
			*period = RxResult;
			return -1;
		}
		*period = RxResult;	// 接收设置的IP中 插值时间间隔--周期
/* debug *///		printf("IP Time Period is: 0x%d\r\n", *period);	// debug
		return 1;
	}
	return 0;
}
/* SDO方式读驱动器 IP 模式下，IP的周期值的单位 如：10^-1 -2 -3 -4 s (-2 ~ -6) def = -3 -> 10^-3s=1ms */
int CAN_SDO_IP_Time_Record_Period_Index_Value(canDriver *driver, uint16_t NodeID, IPTimeIndexTypeDef *timeIndex)
{
	if ( 0 == CAN_SDO_Fast_Read(driver, NodeID, &OBJ_IP_Time_Record_Period_Index))
		return 0;
	int res = CAN_Rx_Data_Handle(driver,NodeID);
	if (0 == res)	// 通信上的超时标志
	{
		return 0;
	}
	else if (4 == res || 3 == res)	// SDO 终止传输 或 EMCY消息标志
	{
		*timeIndex = (IPTimeIndexTypeDef)RxResult;
		return -1;
	}
	else if (2 == res)	// 有效的SDO响应 且 无错误
	{
		if (4 == CAN_Rx_Data_Handle(driver,NodeID)) {	// 在有效的SDO响应之后如果有SDO终止响应标志
			*timeIndex = (IPTimeIndexTypeDef)RxResult;
			return -1;
		}
		*timeIndex = (IPTimeIndexTypeDef)RxResult;	// 接收设置的IP中 插值周期的时间单位索引值
/* debug *///		printf("IP Time Period Index is: 10^%d s\r\n", *timeIndex);	// debug
		return 1;
	}
	return 0;
}
/* SDO方式读驱动器 IP 模式下，IP的buffer最大值 (虽然可读可写 - 但是只要子模式已经确定这个对象的值也跟着确定了 sub=0: 1, sub=-1: 16) */
int CAN_SDO_IP_Data_Config_Max_Buf_Size_Value(canDriver *driver, uint16_t NodeID, uint32_t *MaxBufSize)
{
	if ( 0 == CAN_SDO_Fast_Read(driver, NodeID, &OBJ_IP_Data_Config_Max_Buf_Size))
		return 0;
	int res = CAN_Rx_Data_Handle(driver,NodeID);
	if (0 == res)	// 通信上的超时标志
	{
		return 0;
	}
	else if (4 == res || 3 == res)	// SDO 终止传输 或 EMCY消息标志
	{
		*MaxBufSize = RxResult;
		return -1;
	}
	else if (2 == res)	// 有效的SDO响应 且 无错误
	{
		if (4 == CAN_Rx_Data_Handle(driver,NodeID)) {	// 在有效的SDO响应之后如果有SDO终止响应标志
			*MaxBufSize = RxResult;
			return -1;
		}
		*MaxBufSize = RxResult;	// 接收设置的IP下 buffer的最大值
/* debug *///		printf("IP Buffer Max Number is: %ld s\r\n", *MaxBufSize);	// debug
		return 1;
	}
	return 0;
}
/* SDO方式读驱动器 IP 模式下，IP的buffer实际大小 (与子模式的值有关联 sub=0: 1, sub=-1: 1~16) */
int CAN_SDO_IP_Data_Config_Actual_Buf_Size_Value(canDriver *driver, uint16_t NodeID, uint32_t *ActualBufSize)
{
	if ( 0 == CAN_SDO_Fast_Read(driver, NodeID, &OBJ_IP_Data_Config_Actual_Buf_Size))
		return 0;
	int res = CAN_Rx_Data_Handle(driver,NodeID);
	if (0 == res)	// 通信上的超时标志
	{
		return 0;
	}
	else if (4 == res || 3 == res)	// SDO 终止传输 或 EMCY消息标志
	{
		*ActualBufSize = RxResult;
		return -1;
	}
	else if (2 == res)	// 有效的SDO响应 且 无错误
	{
		if (4 == CAN_Rx_Data_Handle(driver,NodeID)) {	// 在有效的SDO响应之后如果有SDO终止响应标志
			*ActualBufSize = RxResult;
			return -1;
		}
		*ActualBufSize = RxResult;	// 接收设置的IP下的 buffer实际的大小
/* debug *///		printf("IP Buffer Actual Number is: %ld s\r\n", *ActualBufSize);	// debug
		return 1;
	}
	return 0;
}
/* SDO方式读驱动器 IP 模式下，IP的buffer组织形式 */
int CAN_SDO_IP_Data_Config_Buf_Org_Value(canDriver *driver, uint16_t NodeID, IPBufOrgTypeDef *orgMode)
{
	if ( 0 == CAN_SDO_Fast_Read(driver, NodeID, &OBJ_IP_Data_Config_Buf_Org))
		return 0;
	int res = CAN_Rx_Data_Handle(driver,NodeID);
	if (0 == res)	// 通信上的超时标志
	{
		return 0;
	}
	else if (4 == res || 3 == res)	// SDO 终止传输 或 EMCY消息标志
	{
		*orgMode = (IPBufOrgTypeDef)RxResult;
		return -1;
	}
	else if (2 == res)	// 有效的SDO响应 且 无错误
	{
		if (4 == CAN_Rx_Data_Handle(driver,NodeID)) {	// 在有效的SDO响应之后如果有SDO终止响应标志
			*orgMode = (IPBufOrgTypeDef)RxResult;
			return -1;
		}
		*orgMode = (IPBufOrgTypeDef)RxResult;	// 接收设置的IP buffer 形式 FIFO、Ring
/* debug *///		printf("IP Buffer Organization is: %d\r\n", *orgMode);	// debug
		return 1;
	}
	return 0;
}
/* SDO方式读驱动器 IP 模式下，IP的buffer的位置值 */
int CAN_SDO_IP_Data_Config_Buf_Pos_Value(canDriver *driver, uint16_t NodeID, uint16_t *bufPos)
{
	if ( 0 == CAN_SDO_Fast_Read(driver, NodeID, &OBJ_IP_Data_Config_Buf_Pos))
		return 0;
	int res = CAN_Rx_Data_Handle(driver,NodeID);
	if (0 == res)	// 通信上的超时标志
	{
		return 0;
	}
	else if (4 == res || 3 == res)	// SDO 终止传输 或 EMCY消息标志
	{
		*bufPos = RxResult;
		return -1;
	}
	else if (2 == res)	// 有效的SDO响应 且 无错误
	{
		if (4 == CAN_Rx_Data_Handle(driver,NodeID)) {	// 在有效的SDO响应之后如果有SDO终止响应标志
			*bufPos = RxResult;
			return -1;
		}
		*bufPos = RxResult;	// 接收设置的IP中，此时Buffer的实际位置
/* debug *///		printf("IP Buffer Position is: %ld\r\n", *bufPos);	// debug
		return 1;
	}
	return 0;
}
/* SDO方式读驱动器 IP 模式下，IP的每条数据记录的大小 以字节为单位 sub=0: 4, sub=-1: 8 */
int CAN_SDO_IP_Data_Config_Data_Record_Size_Value(canDriver *driver, uint16_t NodeID, uint8_t *dataRecordSize)
{
	if ( 0 == CAN_SDO_Fast_Read(driver, NodeID, &OBJ_IP_Data_Config_Data_Record_Size))
		return 0;
	int res = CAN_Rx_Data_Handle(driver,NodeID);
	if (0 == res)	// 通信上的超时标志
	{
		return 0;
	}
	else if (4 == res || 3 == res)	// SDO 终止传输 或 EMCY消息标志
	{
		*dataRecordSize = RxResult;
		return -1;
	}
	else if (2 == res)	// 有效的SDO响应 且 无错误
	{
		if (4 == CAN_Rx_Data_Handle(driver,NodeID)) {	// 在有效的SDO响应之后如果有SDO终止响应标志
			*dataRecordSize = RxResult;
			return -1;
		}
		*dataRecordSize = RxResult;	// 接收设置的IP中，每条数据记录的字节个数--大小
/* debug *///		printf("IP Data Record Size is: %d Byte\r\n", *dataRecordSize);	// debug
		return 1;
	}
	return 0;
}


/* SDO方式写驱动器 电机停止前外推插值周期个数 1~32768 */
int CAN_SDO_IP_Extrapolation_Cycles_Timeout_Value(canDriver *driver, uint16_t NodeID, int16_t *cycleNumValue)
{
	if ( 0 == CAN_SDO_Fast_Read(driver, NodeID, &OBJ_Extrapolation_Cycles_Timeout))
		return 0;
	int res = CAN_Rx_Data_Handle(driver,NodeID);
	if (0 == res)	// 通信上的超时标志
	{
		return 0;
	}
	else if (4 == res || 3 == res)	// SDO 终止传输 或 EMCY消息标志
	{
		*cycleNumValue = RxResult;
		return -1;
	}
	else if (2 == res)	// 有效的SDO响应 且 无错误
	{
		if (4 == CAN_Rx_Data_Handle(driver,NodeID)) {	// 在有效的SDO响应之后如果有SDO终止响应标志
			*cycleNumValue = RxResult;
			return -1;
		}
		*cycleNumValue = RxResult;	// 接收设置的IP中，每条数据记录的字节个数--大小
/* debug *///		printf("IP Data Record Size is: %d Byte\r\n", *dataRecordSize);	// debug
		return 1;
	}
	return 0;
}



/* SDO读取驱动器里电机的位置值 (以用户定义的单位给出) int32 *position 返回实际的位置 */
int CAN_SDO_Actual_Position_User(canDriver *driver, uint16_t NodeID, int32_t *position)
{
	if ( 0 == CAN_SDO_Fast_Read(driver, NodeID, &OBJ_Actual_Position_User))
		return 0;
	int res = CAN_Rx_Data_Handle(driver,NodeID);
	if (0 == res)	// 通信上的超时标志
	{
		return 0;
	}
	else if (4 == res || 3 == res)	// SDO 终止传输 或 EMCY消息标志
	{
		*position = RxResult;
		return -1;
	}
	else if (2 == res)	// 有效的SDO响应 且 无错误
	{
		if (4 == CAN_Rx_Data_Handle(driver,NodeID)) {	// 在有效的SDO响应之后如果有SDO终止响应标志
			*position = RxResult;
			return -1;
		}
		*position = RxResult;	// 接收当前位置值
	/* debug *///	printf("Actual Position is : %ld\r\n", *position);	// debug
		return 1;
	}
	return 0;
}

/* SDO读驱动器里电机位置控制命令值 demmand value int32 */
int CAN_SDO_Demand_Position_Value_User(canDriver *driver, uint16_t NodeID, int32_t *demmandPos)
{
	if ( 0 == CAN_SDO_Fast_Read(driver, NodeID, &OBJ_Demand_Position_User))
		return 0;
	int res = CAN_Rx_Data_Handle(driver,NodeID);
	if (0 == res)	// 通信上的超时标志
	{
		return 0;
	}
	else if (4 == res || 3 == res)	// SDO 终止传输 或 EMCY消息标志
	{
		*demmandPos = RxResult;
		return -1;
	}
	else if (2 == res)	// 有效的SDO响应 且 无错误
	{
		if (4 == CAN_Rx_Data_Handle(driver,NodeID)) {	// 在有效的SDO响应之后如果有SDO终止响应标志
			*demmandPos = RxResult;
			return -1;
		}
		*demmandPos = RxResult;	// 接收当前位置命令值
	/* debug *///	printf("Demand Position is : %ld\r\n", *position);	// debug
		return 1;
	}
	return 0;
}

/* SDO读取驱动器里电机的速度值 (以用户定义的单位给出) int32 *velocity 返回实际的速度 */
int CAN_SDO_Actual_Velocity_User(canDriver *driver, uint16_t NodeID, int32_t *velocity)
{
	if ( 0 == CAN_SDO_Fast_Read(driver, NodeID, &OBJ_Actual_Velocity_User))
			return 0;
		int res = CAN_Rx_Data_Handle(driver,NodeID);
		if (0 == res)	// 通信上的超时标志
		{
			return 0;
		}
		else if (4 == res || 3 == res)	// SDO 终止传输 或 EMCY消息标志
		{
			*velocity = RxResult;
			return -1;
		}
		else if (2 == res)	// 有效的SDO响应 且 无错误
		{
			if (4 == CAN_Rx_Data_Handle(driver,NodeID)) {	// 在有效的SDO响应之后如果有SDO终止响应标志
				*velocity = RxResult;
				return -1;
			}
			*velocity = RxResult;	// 接收当前速度值
		/* debug *///	printf("Actual Position is : %ld\r\n", *position);	// debug
			return 1;
		}
		return 0;
}

/* SDO读取驱动器里电机的位置跟踪误差 */
int CAN_SDO_Position_Following_Error_User(canDriver *driver, uint16_t NodeID, int32_t *posError)
{
	if ( 0 == CAN_SDO_Fast_Read(driver, NodeID, &OBJ_Position_Following_Error_User))
		return 0;
	int res = CAN_Rx_Data_Handle(driver,NodeID);
	if (0 == res)	// 通信上的超时标志
	{
		return 0;
	}
	else if (4 == res || 3 == res)	// SDO 终止传输 或 EMCY消息标志
	{
		*posError = RxResult;
		return -1;
	}
	else if (2 == res)	// 有效的SDO响应 且 无错误
	{
		if (4 == CAN_Rx_Data_Handle(driver,NodeID)) {	// 在有效的SDO响应之后如果有SDO终止响应标志
			*posError = RxResult;
			return -1;
		}
		*posError = RxResult;	// 接收当前位置跟踪误差
		/* debug *///	printf("Position Following Error is : %ld\r\n", *posError);	// debug
		return 1;
	}
	return 0;
}

/* SDO读取驱动器的位置环的输出量 control effort */
int CAN_SDO_Position_Control_Effort(canDriver *driver, uint16_t NodeID, int32_t *posControlEffort)
{
	if ( 0 == CAN_SDO_Fast_Read(driver, NodeID, &OBJ_Position_Loop_Control_Effort))
		return 0;
	int res = CAN_Rx_Data_Handle(driver,NodeID);
	if (0 == res)	// 通信上的超时标志
	{
		return 0;
	}
	else if (4 == res || 3 == res)	// SDO 终止传输 或 EMCY消息标志
	{
		*posControlEffort = RxResult;
		return -1;
	}
	else if (2 == res)	// 有效的SDO响应 且 无错误
	{
		if (4 == CAN_Rx_Data_Handle(driver,NodeID)) {	// 在有效的SDO响应之后如果有SDO终止响应标志
			*posControlEffort = RxResult;
			return -1;
		}
		*posControlEffort = RxResult;	// 接收位置环的控制作用输出的量
		/* debug *///	printf("Position Control Effort is : %ld\r\n", *posControlEffort);	// debug
		return 1;
	}
	return 0;
}

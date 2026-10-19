/*
 * pdo.c
 *
 *  Created on: Oct 11, 2021
 *      Author: maxiufeng
 */

#include "pdo.h"
#include "sdo.h"
#include "string.h"

extern int CAN_Rx_Data_Handle(canDriver *driver, uint16_t NodeID); //defined in canopen.h

ODOBJTypeDef	OBJ_PDOx_COMP_COBID_Sub1 = {0};
ODOBJTypeDef	OBJ_PDOx_COMP_TRANS_TYPE_Sub2 = {0};
ODOBJTypeDef	OBJ_PDOx_COMP_INHIBIT_TIME_Sub3 = {0};
ODOBJTypeDef	OBJ_PDOx_COMP_EVENT_TIMER_Sub5 = {0};
ODOBJTypeDef	OBJ_PDOx_MAPPING_NUM_Sub0 = {0};
ODOBJTypeDef	OBJ_PDOx_MAPPING_PDO_Subx[8] = {{0}};

/* 用PDOxType类型 FunctionCoide 值决定PDO的具体对象 */
static void getPDOxObjects(PDOxTypeDef PDOxType)
{
	switch (PDOxType) {
		case PDO_R1:	// RPDO1 相关联对象
		{
			/* RPDO1 通信参数字典对象 */
			OBJ_PDOx_COMP_COBID_Sub1		=	OBJ_RPDO1_COMP_COBID_Sub1;
			OBJ_PDOx_COMP_TRANS_TYPE_Sub2	=	OBJ_RPDO1_COMP_TRANS_TYPE_Sub2;
			/* RPDO1 映射参数字典对象 */
			OBJ_PDOx_MAPPING_NUM_Sub0		=	OBJ_RPDO1_MAPPING_NUM_Sub0;
			OBJ_PDOx_MAPPING_PDO_Subx[0]	=	OBJ_RPDO1_MAPPING_PDO_Sub1;
			OBJ_PDOx_MAPPING_PDO_Subx[1]	=	OBJ_RPDO1_MAPPING_PDO_Sub2;
			OBJ_PDOx_MAPPING_PDO_Subx[2]	=	OBJ_RPDO1_MAPPING_PDO_Sub3;
			OBJ_PDOx_MAPPING_PDO_Subx[3]	=	OBJ_RPDO1_MAPPING_PDO_Sub4;
			OBJ_PDOx_MAPPING_PDO_Subx[4]	=	OBJ_RPDO1_MAPPING_PDO_Sub5;
			OBJ_PDOx_MAPPING_PDO_Subx[5]	=	OBJ_RPDO1_MAPPING_PDO_Sub6;
			OBJ_PDOx_MAPPING_PDO_Subx[6]	=	OBJ_RPDO1_MAPPING_PDO_Sub7;
			OBJ_PDOx_MAPPING_PDO_Subx[7]	=	OBJ_RPDO1_MAPPING_PDO_Sub8;
			break;
		}
		case PDO_R2:	// RPDO2 相关联对象
		{
			/* RPDO2 通信参数字典对象 */
			OBJ_PDOx_COMP_COBID_Sub1		=	OBJ_RPDO2_COMP_COBID_Sub1;
			OBJ_PDOx_COMP_TRANS_TYPE_Sub2	=	OBJ_RPDO2_COMP_TRANS_TYPE_Sub2;
			/* RPDO2 映射参数字典对象 */
			OBJ_PDOx_MAPPING_NUM_Sub0		=	OBJ_RPDO2_MAPPING_NUM_Sub0;
			OBJ_PDOx_MAPPING_PDO_Subx[0]	=	OBJ_RPDO2_MAPPING_PDO_Sub1;
			OBJ_PDOx_MAPPING_PDO_Subx[1]	=	OBJ_RPDO2_MAPPING_PDO_Sub2;
			OBJ_PDOx_MAPPING_PDO_Subx[2]	=	OBJ_RPDO2_MAPPING_PDO_Sub3;
			OBJ_PDOx_MAPPING_PDO_Subx[3]	=	OBJ_RPDO2_MAPPING_PDO_Sub4;
			OBJ_PDOx_MAPPING_PDO_Subx[4]	=	OBJ_RPDO2_MAPPING_PDO_Sub5;
			OBJ_PDOx_MAPPING_PDO_Subx[5]	=	OBJ_RPDO2_MAPPING_PDO_Sub6;
			OBJ_PDOx_MAPPING_PDO_Subx[6]	=	OBJ_RPDO2_MAPPING_PDO_Sub7;
			OBJ_PDOx_MAPPING_PDO_Subx[7]	=	OBJ_RPDO2_MAPPING_PDO_Sub8;
			break;
		}
		case PDO_R3:	// RPDO3 相关联对象
		{
			/* RPDO3 通信参数字典对象 */
			OBJ_PDOx_COMP_COBID_Sub1		=	OBJ_RPDO3_COMP_COBID_Sub1;
			OBJ_PDOx_COMP_TRANS_TYPE_Sub2	=	OBJ_RPDO3_COMP_TRANS_TYPE_Sub2;
			/* RPDO3 映射参数字典对象 */
			OBJ_PDOx_MAPPING_NUM_Sub0		=	OBJ_RPDO3_MAPPING_NUM_Sub0;
			OBJ_PDOx_MAPPING_PDO_Subx[0]	=	OBJ_RPDO3_MAPPING_PDO_Sub1;
			OBJ_PDOx_MAPPING_PDO_Subx[1]	=	OBJ_RPDO3_MAPPING_PDO_Sub2;
			OBJ_PDOx_MAPPING_PDO_Subx[2]	=	OBJ_RPDO3_MAPPING_PDO_Sub3;
			OBJ_PDOx_MAPPING_PDO_Subx[3]	=	OBJ_RPDO3_MAPPING_PDO_Sub4;
			OBJ_PDOx_MAPPING_PDO_Subx[4]	=	OBJ_RPDO3_MAPPING_PDO_Sub5;
			OBJ_PDOx_MAPPING_PDO_Subx[5]	=	OBJ_RPDO3_MAPPING_PDO_Sub6;
			OBJ_PDOx_MAPPING_PDO_Subx[6]	=	OBJ_RPDO3_MAPPING_PDO_Sub7;
			OBJ_PDOx_MAPPING_PDO_Subx[7]	=	OBJ_RPDO3_MAPPING_PDO_Sub8;
			break;
		}
		case PDO_R4:	// RPDO4 相关联对象
		{
			/* RPDO4 通信参数字典对象 */
			OBJ_PDOx_COMP_COBID_Sub1		=	OBJ_RPDO4_COMP_COBID_Sub1;
			OBJ_PDOx_COMP_TRANS_TYPE_Sub2	=	OBJ_RPDO4_COMP_TRANS_TYPE_Sub2;
			/* RPDO4 映射参数字典对象 */
			OBJ_PDOx_MAPPING_NUM_Sub0		=	OBJ_RPDO4_MAPPING_NUM_Sub0;
			OBJ_PDOx_MAPPING_PDO_Subx[0]	=	OBJ_RPDO4_MAPPING_PDO_Sub1;
			OBJ_PDOx_MAPPING_PDO_Subx[1]	=	OBJ_RPDO4_MAPPING_PDO_Sub2;
			OBJ_PDOx_MAPPING_PDO_Subx[2]	=	OBJ_RPDO4_MAPPING_PDO_Sub3;
			OBJ_PDOx_MAPPING_PDO_Subx[3]	=	OBJ_RPDO4_MAPPING_PDO_Sub4;
			OBJ_PDOx_MAPPING_PDO_Subx[4]	=	OBJ_RPDO4_MAPPING_PDO_Sub5;
			OBJ_PDOx_MAPPING_PDO_Subx[5]	=	OBJ_RPDO4_MAPPING_PDO_Sub6;
			OBJ_PDOx_MAPPING_PDO_Subx[6]	=	OBJ_RPDO4_MAPPING_PDO_Sub7;
			OBJ_PDOx_MAPPING_PDO_Subx[7]	=	OBJ_RPDO4_MAPPING_PDO_Sub8;
			break;
		}
		case PDO_T1:	// TPDO1 相关联对象
		{
			/* TPDO1 通信参数字典对象 */
			OBJ_PDOx_COMP_COBID_Sub1		=	OBJ_TPDO1_COMP_COBID_Sub1;
			OBJ_PDOx_COMP_TRANS_TYPE_Sub2	=	OBJ_TPDO1_COMP_TRANS_TYPE_Sub2;
			OBJ_PDOx_COMP_INHIBIT_TIME_Sub3	=	OBJ_TPDO1_COMP_INHIBIT_TIME_Sub3;
			OBJ_PDOx_COMP_EVENT_TIMER_Sub5	=	OBJ_TPDO1_COMP_EVENT_TIMER_Sub5;
			/* TPDO1 映射参数字典对象 */
			OBJ_PDOx_MAPPING_NUM_Sub0		=	OBJ_TPDO1_MAPPING_NUM_Sub0;
			OBJ_PDOx_MAPPING_PDO_Subx[0]	=	OBJ_TPDO1_MAPPING_PDO_Sub1;
			OBJ_PDOx_MAPPING_PDO_Subx[1]	=	OBJ_TPDO1_MAPPING_PDO_Sub2;
			OBJ_PDOx_MAPPING_PDO_Subx[2]	=	OBJ_TPDO1_MAPPING_PDO_Sub3;
			OBJ_PDOx_MAPPING_PDO_Subx[3]	=	OBJ_TPDO1_MAPPING_PDO_Sub4;
			OBJ_PDOx_MAPPING_PDO_Subx[4]	=	OBJ_TPDO1_MAPPING_PDO_Sub5;
			OBJ_PDOx_MAPPING_PDO_Subx[5]	=	OBJ_TPDO1_MAPPING_PDO_Sub6;
			OBJ_PDOx_MAPPING_PDO_Subx[6]	=	OBJ_TPDO1_MAPPING_PDO_Sub7;
			OBJ_PDOx_MAPPING_PDO_Subx[7]	=	OBJ_TPDO1_MAPPING_PDO_Sub8;
			break;
		}
		case PDO_T2:	// TPDO2 相关联对象
		{
			/* TPDO2 通信参数字典对象 */
			OBJ_PDOx_COMP_COBID_Sub1		=	OBJ_TPDO2_COMP_COBID_Sub1;
			OBJ_PDOx_COMP_TRANS_TYPE_Sub2	=	OBJ_TPDO2_COMP_TRANS_TYPE_Sub2;
			OBJ_PDOx_COMP_INHIBIT_TIME_Sub3	=	OBJ_TPDO2_COMP_INHIBIT_TIME_Sub3;
			OBJ_PDOx_COMP_EVENT_TIMER_Sub5	=	OBJ_TPDO2_COMP_EVENT_TIMER_Sub5;
			/* TPDO2 映射参数字典对象 */
			OBJ_PDOx_MAPPING_NUM_Sub0		=	OBJ_TPDO2_MAPPING_NUM_Sub0;
			OBJ_PDOx_MAPPING_PDO_Subx[0]	=	OBJ_TPDO2_MAPPING_PDO_Sub1;
			OBJ_PDOx_MAPPING_PDO_Subx[1]	=	OBJ_TPDO2_MAPPING_PDO_Sub2;
			OBJ_PDOx_MAPPING_PDO_Subx[2]	=	OBJ_TPDO2_MAPPING_PDO_Sub3;
			OBJ_PDOx_MAPPING_PDO_Subx[3]	=	OBJ_TPDO2_MAPPING_PDO_Sub4;
			OBJ_PDOx_MAPPING_PDO_Subx[4]	=	OBJ_TPDO2_MAPPING_PDO_Sub5;
			OBJ_PDOx_MAPPING_PDO_Subx[5]	=	OBJ_TPDO2_MAPPING_PDO_Sub6;
			OBJ_PDOx_MAPPING_PDO_Subx[6]	=	OBJ_TPDO2_MAPPING_PDO_Sub7;
			OBJ_PDOx_MAPPING_PDO_Subx[7]	=	OBJ_TPDO2_MAPPING_PDO_Sub8;
			break;
		}
		case PDO_T3:	// TPDO3 相关联对象
		{
			/* TPDO3 通信参数字典对象 */
			OBJ_PDOx_COMP_COBID_Sub1		=	OBJ_TPDO3_COMP_COBID_Sub1;
			OBJ_PDOx_COMP_TRANS_TYPE_Sub2	=	OBJ_TPDO3_COMP_TRANS_TYPE_Sub2;
			OBJ_PDOx_COMP_INHIBIT_TIME_Sub3	=	OBJ_TPDO3_COMP_INHIBIT_TIME_Sub3;
			OBJ_PDOx_COMP_EVENT_TIMER_Sub5	=	OBJ_TPDO3_COMP_EVENT_TIMER_Sub5;
			/* TPDO3 映射参数字典对象 */
			OBJ_PDOx_MAPPING_NUM_Sub0		=	OBJ_TPDO3_MAPPING_NUM_Sub0;
			OBJ_PDOx_MAPPING_PDO_Subx[0]	=	OBJ_TPDO3_MAPPING_PDO_Sub1;
			OBJ_PDOx_MAPPING_PDO_Subx[1]	=	OBJ_TPDO3_MAPPING_PDO_Sub2;
			OBJ_PDOx_MAPPING_PDO_Subx[2]	=	OBJ_TPDO3_MAPPING_PDO_Sub3;
			OBJ_PDOx_MAPPING_PDO_Subx[3]	=	OBJ_TPDO3_MAPPING_PDO_Sub4;
			OBJ_PDOx_MAPPING_PDO_Subx[4]	=	OBJ_TPDO3_MAPPING_PDO_Sub5;
			OBJ_PDOx_MAPPING_PDO_Subx[5]	=	OBJ_TPDO3_MAPPING_PDO_Sub6;
			OBJ_PDOx_MAPPING_PDO_Subx[6]	=	OBJ_TPDO3_MAPPING_PDO_Sub7;
			OBJ_PDOx_MAPPING_PDO_Subx[7]	=	OBJ_TPDO3_MAPPING_PDO_Sub8;
			break;
		}
		case PDO_T4:	// TPDO4 相关联对象
		{
			/* TPDO4 通信参数字典对象 */
			OBJ_PDOx_COMP_COBID_Sub1		=	OBJ_TPDO4_COMP_COBID_Sub1;
			OBJ_PDOx_COMP_TRANS_TYPE_Sub2	=	OBJ_TPDO4_COMP_TRANS_TYPE_Sub2;
			OBJ_PDOx_COMP_INHIBIT_TIME_Sub3	=	OBJ_TPDO4_COMP_INHIBIT_TIME_Sub3;
			OBJ_PDOx_COMP_EVENT_TIMER_Sub5	=	OBJ_TPDO4_COMP_EVENT_TIMER_Sub5;
			/* TPDO1 映射参数字典对象 */
			OBJ_PDOx_MAPPING_NUM_Sub0		=	OBJ_TPDO4_MAPPING_NUM_Sub0;
			OBJ_PDOx_MAPPING_PDO_Subx[0]	=	OBJ_TPDO4_MAPPING_PDO_Sub1;
			OBJ_PDOx_MAPPING_PDO_Subx[1]	=	OBJ_TPDO4_MAPPING_PDO_Sub2;
			OBJ_PDOx_MAPPING_PDO_Subx[2]	=	OBJ_TPDO4_MAPPING_PDO_Sub3;
			OBJ_PDOx_MAPPING_PDO_Subx[3]	=	OBJ_TPDO4_MAPPING_PDO_Sub4;
			OBJ_PDOx_MAPPING_PDO_Subx[4]	=	OBJ_TPDO4_MAPPING_PDO_Sub5;
			OBJ_PDOx_MAPPING_PDO_Subx[5]	=	OBJ_TPDO4_MAPPING_PDO_Sub6;
			OBJ_PDOx_MAPPING_PDO_Subx[6]	=	OBJ_TPDO4_MAPPING_PDO_Sub7;
			OBJ_PDOx_MAPPING_PDO_Subx[7]	=	OBJ_TPDO4_MAPPING_PDO_Sub8;
			break;
		}
		default:
			break;
	}
}

/**
  ******************************************************************************
  * @name    CAN_PDO_Set_ComParam and CAN_PDO_Set_MapParam()
  * @author  Ma xiufeng
  * @brief   实现 PDO 参数重新映射的方法.
  *          这两个函数分别是：
  *           * PDO的通信参数配置 包括：PDO-COBID和Transmission Type的设置
  *          一个PDO能传送数据量是 1~8 字节，实际长度由具体被映射的对象数据长度所决定
  @verbatim
  ==============================================================================
                        ##### 设置RPDO/TPDO映射流程 #####
  ==============================================================================
  	  [#] 禁用销毁RPDO/TPDO，通过设置 通信参数sub1.bit31 = 1
  	  [#] 设置通信参数的sub2.3.5子对象的值
  	  [#] 去使能 通过设置 RPDO/TPDO 映射参数sub0 = 0
  	  [#] 修改映射的值，通过设置 映射参数sub1.2.3...8	n个
  	  [#] 设置实际被映射的对象个数，通过 映射参数sub0 = n
  	  [#] 创建RPDO/TPDO映射，通过设置 通信参数sub1.bit31 = 0
  ******************************************************************************
  */

/* 设置 RPDO 或 TPDO 的通信参数 包括sub1的COBID 和 sub2的传输类型(同步或异步) */
int CAN_PDO_Set_ComParam(canDriver *driver,
		PDOxTypeDef PDOxType, uint8_t NodeID,
		PDOxFunctionTypeDef PDOxFuncType,
		PDOxTransmissionTypeDef SYNCType, uint16_t inhibitTime, uint16_t eventTimer)
{
	uint32_t COBID = PDOxType | NodeID;	// 拼接COBID标识码 PDO FC + NodeID
	unionUint32ArrayTypeDef txData;

	getPDOxObjects(PDOxType);

	/* 按照功能码执行功能 */
	switch (PDOxFuncType) {
		case PDO_FUNCTION_ENABLE:	//1. 使能启用PDO 通信参数 sub1.bit31 设置为 0
		{
			UNUSED(SYNCType);	UNUSED(inhibitTime);	UNUSED(eventTimer);
			txData.value = PDO_ENABLE | COBID;
			int res= CAN_SDO_Fast_Write(driver, NodeID, &OBJ_PDOx_COMP_COBID_Sub1, txData.array);
			if (0 == res)
				return 0;
			res = CAN_Rx_Data_Handle(driver,NodeID);
			if(res != 1)
				return res;
			break;
		}
		case PDO_FUNCTION_DISABLE:	//2. 销毁禁用PDO 通信参数 sub1.bit31 设置为 1
		{
			UNUSED(SYNCType);	UNUSED(inhibitTime);	UNUSED(eventTimer);
			txData.value = PDO_DISABLE | COBID;
			int res = CAN_SDO_Fast_Write(driver, NodeID, &OBJ_PDOx_COMP_COBID_Sub1, txData.array);
			if (0 == res)
				return 0;
			res = CAN_Rx_Data_Handle(driver,NodeID);
			if (res != 1)
				return res;
			break;
		}
		case PDO_FUNCTION_SET_TRANS_TYPE:	//3. 设置PDO传输类型对象 同步、异步
		{
			UNUSED(inhibitTime);	UNUSED(eventTimer);
			txData.value = SYNCType;
			int res = CAN_SDO_Fast_Write(driver, NodeID, &OBJ_PDOx_COMP_TRANS_TYPE_Sub2, txData.array);
			if (0 == res)
				return 0;
			res = CAN_Rx_Data_Handle(driver,NodeID);
			if (res != 1)
				return res;
			break;
		}
		/* 下边两个TPDO的通信参数子对象设置 ↓↓↓↓↓↓↓ */
		case PDO_FUNCTION_SET_INHIBIT_TIME:	//4. 设置TPDO禁止时间
		{
			UNUSED(SYNCType);	UNUSED(eventTimer);
			txData.value = inhibitTime;
			int res = CAN_SDO_Fast_Write(driver, NodeID, &OBJ_PDOx_COMP_INHIBIT_TIME_Sub3, txData.array);
			if (0 == res)
				return 0;
			res = CAN_Rx_Data_Handle(driver,NodeID);
			if (res != 1)
				return res;
			break;
		}
		case PDO_FUNCTION_SET_EVENT_TIMER:	//5. 设置TPDO事件定时器
		{
			UNUSED(SYNCType);	UNUSED(inhibitTime);
			txData.value = eventTimer;
			int res = CAN_SDO_Fast_Write(driver, NodeID, &OBJ_PDOx_COMP_EVENT_TIMER_Sub5, txData.array);
			if (0 == res)
				return 0;
			res = CAN_Rx_Data_Handle(driver,NodeID);
			if (res != 1)
				return res;
			break;
		}
		default:
			break;
	}
	return 1;
}

/* 设置 RPDO 或 TPDO 的映射参数 将指定数量的对象映射到PDO映射参数对象中
 * 返回： 0--通信硬件错误	 1--正常  res--包括通信硬件错误 和 从机发出的错误标识
 * */
int CAN_PDO_Set_MapParam(canDriver *driver, PDOxTypeDef PDOxType, uint8_t NodeID, uint8_t MappingNum, ODOBJTypeDef MappedOBJs[])
{
	unionUint32ArrayTypeDef txData = {0};
	// 获取具体用到的PDOx相关对象
	getPDOxObjects(PDOxType);
	// 1.映射到PDO的sub1对象个数设置为 0
	if ( 0 == CAN_SDO_Fast_Write(driver, NodeID, &OBJ_PDOx_MAPPING_NUM_Sub0, txData.array))
		return 0;
	int res = CAN_Rx_Data_Handle(driver,NodeID);
	if (res != 1)
		return res;

	// 2.向PDOsub1、2、3...8中添加指定数量的被映射的对象
	for (uint8_t var = 0; var < MappingNum; ++var) {
		// 被映射的对象
		ODOBJTypeDef mapped_obj      = MappedOBJs[var];
		// PDO映射对象设置子对象
		ODOBJTypeDef pdo_mapping_obj = OBJ_PDOx_MAPPING_PDO_Subx[var];
		// 合成PDO映射对象的数据部分 4 字节	| byte4 len | byte5 sub-id | byte6 id(LSB) | byte7 id(HSB) |
		txData.array[0] = (4-((mapped_obj.data_len>>2) & 0xFF))*8;
		txData.array[1] =  mapped_obj.sub_index;
		txData.array[2] = (mapped_obj.index>>0)&0xFF;
		txData.array[3] = (mapped_obj.index>>8)&0xFF;
		// 写指定的PDO映射参数
		if (0 == CAN_SDO_Fast_Write(driver, NodeID, &pdo_mapping_obj, txData.array))
			return 0;
		int res = CAN_Rx_Data_Handle(driver,NodeID);
		if (res != 1)
			return res;
	}

	// 3.映射到PDO的sub1对象个数设置为 实际被映射对象数
	txData.value = MappingNum;
	if (0 == CAN_SDO_Fast_Write(driver, NodeID, &OBJ_PDOx_MAPPING_NUM_Sub0, txData.array))
		return 0;
	res = CAN_Rx_Data_Handle(driver,NodeID);
	if (res != 1)
		return res;
	return 1;
}


/* 使用PDO的方式发送数据，既从机接收数据 RPDO
 * 前提是 已经配置好了 PDO 的映射
 * 最多一次能发送8个字节 - 64位 */
int CAN_PDO_Send_Data(	canDriver *driver,
						PDOxTypeDef PDOxType,	//应该为 RPDO 的类型，从机接收
						uint8_t NodeID,
						uint8_t txDataNum,	// 要发送数据的个数
						uint8_t txData[])	// 要发送的数据
{
	if (NodeID < 1 && NodeID > 127)
		return 0;
	if (txDataNum < 1 && txDataNum > 8)
		return 0;

	uint8_t canTxData[8] =  {0};	// 传输的数据
	memcpy(canTxData, txData, txDataNum);

	CAN_TxHeaderTypeDef 			canTxHeader;
	canTxHeader.StdId 				= PDOxType | NodeID;	// 构建标准帧11位ID （COBID） 200+NodeID, 300+NodeID, 400+NodeID, 500+NodeID
	
	canTxHeader.DLC					= txDataNum;				// txdataNum 决定长度 1~ 8
	canTxHeader.time_ms				= 10;

	// Waits until all CAN messages for the specified handle are sent
	while(!driver->canDriverWriteSync(100));  
	if (driver->canDriverWrite(canTxHeader,canTxData))
		return 1;
	return 0;
}




/*
 * pdo.h
 *
 *  Created on: Oct 11, 2021
 *      Author: maxiufeng
 */

#ifndef CANOPEN_INC_PDO_H_
#define CANOPEN_INC_PDO_H_

#define UNUSED(x) (void)x



#include "OD.h"

/* 设置 RPDO 或 TPDO 的通信参数 包括sub1的COBID 和 sub2的传输类型(同步或异步) */
int CAN_PDO_Set_ComParam(canDriver *driver,
		PDOxTypeDef PDOxType, uint8_t NodeId,
		PDOxFunctionTypeDef PDOxFuncType,
		PDOxTransmissionTypeDef SYNCType, uint16_t inhibitTime, uint16_t eventTimer);

/* 设置 RPDO 或 TPDO 的映射参数 将指定数量的对象映射到PDO映射参数对象中 */
int CAN_PDO_Set_MapParam(canDriver *driver, PDOxTypeDef PDOxType, uint8_t NodeId, uint8_t MappingNum, ODOBJTypeDef MappedOBJs[]);


/* 使用PDO的方式发送数据，既从机接收数据 RPDO
 * 前提是 已经配置好了 PDO 的映射
 * 最多一次能发送8个字节 - 64位 */
int CAN_PDO_Send_Data(	canDriver *driver,
						PDOxTypeDef PDOxType,	//应该为 RPDO 的类型，从机接收
						uint8_t NodeID,
						uint8_t txDataNum,	// 要发送数据的个数
						uint8_t txData[]);	// 要发送的数据


#endif /* CANOPEN_INC_PDO_H_ */

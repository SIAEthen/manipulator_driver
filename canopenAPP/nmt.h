/*
 * nmt.h
 *
 *  Created on: Oct 11, 2021
 *      Author: maxiufeng
 */

#ifndef CANOPEN_INC_NMT_H_
#define CANOPEN_INC_NMT_H_
#include "OD.h"
#include "canDriver.h"

typedef enum {
	NMT_CMD_START 		= (uint8_t)(0x01),	//1   使节点进入启动状态  PDO、SDO、NMT可用
	NMT_CMD_STOP  		= (uint8_t)(0x02),	//2   使节点进入停止状态  NMT可用
	NMT_CMD_PRE_OP		= (uint8_t)(0x80),	//128 使节点进入预操作状态 PDO不可用，NMT、SDO等
	NMT_CMD_RESET_NODE	= (uint8_t)(0x81),	//129 看作是软件上的复位 此后进入Pre-Op 状态，并发送Boot-Up消息
	NMT_CMD_RESET_COM	= (uint8_t)(0x82),	//130 包括硬件参数在内的复位 比上边的复位更底层，一般不用这个
}NMTCMDTypeDef;


/**
  * @brief  NMT status switch/setting
  *
  * @note   NodeID [0-127], NodeID = 0 (针对所有节点)
  *			-->> <COBID_NMT> [NodeID  NmtCmd]
  * @param  NodeID: 	@uint16_t.
  * @param  NmtCmd: 	@NMTCMDTypeDef.
  * @retval  @int	1:发送成功	0:发送失败
 */
int CAN_NMT_Switch_State(canDriver *driver, uint16_t NodeID, NMTCMDTypeDef NmtCmd);


#endif /* CANOPEN_INC_NMT_H_ */

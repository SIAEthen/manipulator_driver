/*
 * nmt.c
 *
 *  Created on: Oct 11, 2021
 *      Author: maxiufeng
 */

#include "nmt.h"
#include "canDriver.h"

#define	COBID_NMT	FC_NMT	// NMT 网络管理 COBID = 000(00h)


/**
  * @brief  NMT status switch/setting
  *
  * @note   NodeID [0-127], NodeID = 0 (针对所有节点)
  *			-->> <COBID_NMT> [NodeID  NmtCmd]
  * @param  NodeID: 	@uint16_t.
  * @param  NmtCmd: 	@NMTCMDTypeDef.
  * @retval  @int	1:发送成功	0:发送失败
 */
int CAN_NMT_Switch_State(canDriver *driver, uint16_t NodeID, NMTCMDTypeDef NmtCmd)
{
	if (NodeID > 127)
		return 0;

	uint8_t canTxData[2] = {NmtCmd, (uint8_t)NodeID};

	CAN_TxHeaderTypeDef 			canTxHeader;
	canTxHeader.StdId 				= COBID_NMT;
	canTxHeader.DLC					= 2;
	canTxHeader.time_ms				= 10;

	// Waits until all CAN messages for the specified handle are sent
	while(!driver->canDriverWriteSync(100));  
	if (driver->canDriverWrite(canTxHeader,canTxData))
		return 1;
	return 0;
}


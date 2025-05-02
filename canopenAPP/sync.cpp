/*
 * sync.c
 *
 *  Created on: 2021年10月13日
 *      Author: Maxiufeng
 */

#include "canDriver.h"
#include "OD.h"

#define	COBID_SYNC			FC_SYNC			// 同步用 COBID = 128(80h)

/* sync 同步消息发送函数 1:表示发送成功；0:表示发送失败 */
int CAN_SYNC_Msg(canDriver *driver)
{
	CAN_TxHeaderTypeDef 			canTxHeader;
	canTxHeader.StdId 				= COBID_SYNC;
	canTxHeader.DLC					= 0;


	// Waits until all CAN messages for the specified handle are sent
	uint8_t canTxData[8] = {0};
    // while(!driver->canDriverWriteSync(1));
	if (driver->canDriverWrite(canTxHeader,canTxData))
		return 1;
	return 0;
}

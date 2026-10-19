/*
 * sync.h
 *
 *  Created on: 2021年10月13日
 *      Author: Maxiufeng
 */

#ifndef SYNC_INC_SYNC_H_
#define SYNC_INC_SYNC_H_

#include "canDriver.h"
#include "OD.h"

/* sync 同步消息发送函数 1:表示发送成功；0:表示发送失败 */
int CAN_SYNC_Msg(canDriver *driver);

#endif /* SYNC_INC_SYNC_H_ */

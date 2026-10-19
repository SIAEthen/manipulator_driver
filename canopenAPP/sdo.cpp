/*
 * sdo.c
 *
 *  Created on: Oct 11, 2021
 *      Author: maxiufeng
 */

#include "sdo.h"

/**
 * @brief  RSDO 快速下载功能，和节点一次握手，8个字节为一个数据帧
 *
 * @note   NodeID [1-127].
 * 		--> <FC_RSDO|NodeID>  [   2x   | od_index(2Bytes) | od_subindex | data(4Bytes)]
 * 		<-- <FC_TSDO|NodeID>  [   60   | od_index(2Bytes) | od_subindex |  00 (4Bytes)]
 * @param  hcan		@CAN_HandleTypeDef
 * @param  NodeID: 	@uint16_t.
 * @param  pOd: 	@ODTypeDef.
 * @param  pData[]  @uint8_t 4字节的待下载数据
 * @retval   int
 */
int CAN_SDO_Fast_Write(canDriver *driver, uint16_t NodeID, const ODOBJTypeDef *pOd, uint8_t pData[])
{
	if (NodeID < 1 && NodeID > 127)
		return 0;
	if (pOd->is_R_W== OBJ_READ_ONLY)
		return 0;

	uint8_t canTxData[8] = {	(SDO_FAST_DOWN_CSS | pOd->data_len | SDO_TRANSFER_FAST_TYPE | SDO_DATA_SIZE_INDICATE), // 0010xx11
								pOd->index & 0xFF,			// 对象索引低 8 位
								(pOd->index >> 8) & 0xFF,	// 对象索引高 8 位
								pOd->sub_index, 			// 对象子索引
								pData[0], pData[1], pData[2], pData[3]};	// 传输的数据
	
	CAN_TxHeaderTypeDef canTxHeader;
	canTxHeader.StdId 				= FC_RSDO | NodeID;	// 构建标准帧11位ID （COBID）
	canTxHeader.DLC					= 8;				// RSDO 快速下载模式 8 字节定长
	canTxHeader.time_ms				= 10; 				// wait 10 ms
	// Waits until all CAN messages for the specified handle are sent
	while(!driver->canDriverWriteSync(100));  
	if (driver->canDriverWrite(canTxHeader,canTxData)) return 1;
	return 0;
}

/**
 * @brief  RSDO 快速读取功能，和节点一次握手，8个字节为一个数据帧
 *
 * @note   NodeID [1-127].
  * 		--> <FC_RSDO|NodeID>  [    40    | od_index(2Bytes) | od_subindex | NULL(4Bytes)]
  * 		<-- <FC_TSDO|NodeID>  [    4x    | od_index(2Bytes) | od_subindex | data(4Bytes)]
 * @param  hcan		@CAN_HandleTypeDef
 * @param  NodeID: 	@uint16_t.
 * @param  pOd: 	@ODTypeDef.
 * @retval   int
 */
int CAN_SDO_Fast_Read(canDriver *driver, uint16_t NodeID, const ODOBJTypeDef *pOd)
{
	if (NodeID < 1 && NodeID > 127)
		return 0;
	if (pOd->is_R_W== OBJ_WRITE_ONLY)
		return 0;

	uint8_t canTxData[8] = {	SDO_FAST_UPLOAD_CSS,
								pOd->index & 0xFF,
								(pOd->index >> 8) & 0xFF,
								pOd->sub_index,
								0x00, 0x00, 0x00, 0x00};
	CAN_TxHeaderTypeDef 			canTxHeader;
	canTxHeader.StdId 				= FC_RSDO | NodeID;
	canTxHeader.DLC					= 8;
	canTxHeader.time_ms				= 10; 				// wait 10 ms

	// while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0) { };//check if mail is free(free by hardware)
	// we dont wait 
	
	// Waits until all CAN messages for the specified handle are sent
	while(!driver->canDriverWriteSync(100));  
	if (driver->canDriverWrite(canTxHeader,canTxData))
		return 1;
	return 0;
}


/**
 * @brief  RSDO 段读取功能 segment_odd(必须在快速SDO上载协议之后),3,5,7,...，和节点多次握手，共n个字节
 *
 * @note   NodeID [1-127].
  * 		--> <FC_RSDO|NodeID>  [    60    | 00  (7Bytes)]
  * 		<-- <FC_TSDO|NodeID>  [    00    | data(7Bytes)]
 * @param  hcan		@CAN_HandleTypeDef
 * @param  NodeID: 	@uint16_t.
 * @retval   int
 */
int CAN_SDO_Segment_Read_Odd(canDriver *driver, uint16_t NodeID)
{
	uint8_t canTxData[8] = {SDO_SEGMENT_UPLOAD_CSS|SDO_SEGMENT_TOGGLE_ODD, 0, 0, 0, 0, 0, 0, 0};
	CAN_TxHeaderTypeDef 			canTxHeader;
	canTxHeader.StdId 				= FC_RSDO | NodeID;
	canTxHeader.DLC					= 8;
	canTxHeader.time_ms			 	= 10;
	// while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0) { };
	// Waits until all CAN messages for the specified handle are sent
	while(!driver->canDriverWriteSync(100));  
	if (driver->canDriverWrite(canTxHeader,canTxData))
		return 1;
	return 0;
}

/**
 * @brief  RSDO 段读取功能 segment_even(必须在快速SDO奇数段之后),4,6,8,...，和节点多次握手，共n个字节
 *
 * @note   NodeID [1-127].
  * 		--> <FC_RSDO|NodeID>  [    70    | 00  (7Bytes)]
  * 		<-- <FC_TSDO|NodeID>  [    10    | data(7Bytes)]
 * @param  hcan		@CAN_HandleTypeDef
 * @param  NodeID: 	@uint16_t.
 * @retval   int
 */
int CAN_SDO_Segment_Read_Even(canDriver *driver, uint16_t NodeID)
{
	uint8_t canTxData[8] = {SDO_SEGMENT_UPLOAD_CSS|SDO_SEGMENT_TOGGLE_EVEN, 0, 0, 0, 0, 0, 0, 0};
	CAN_TxHeaderTypeDef 			canTxHeader;
	canTxHeader.StdId 				= FC_RSDO | NodeID;
	canTxHeader.DLC					= 8;
	canTxHeader.time_ms			 	= 10;
	// while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0) { };
	// Waits until all CAN messages for the specified handle are sent
	while(!driver->canDriverWriteSync(100));  
	if (driver->canDriverWrite(canTxHeader,canTxData))
		return 1;
	return 0;
}




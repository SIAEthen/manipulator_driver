/*
 * sdo.h
 *
 *  Created on: Oct 11, 2021
 *      Author: maxiufeng
 */

#ifndef CANOPEN_INC_SDO_H_
#define CANOPEN_INC_SDO_H_


#include "OD.h"
#include "canDriver.h"

/* SDO 通信协议中 Byte0 的 bit[7,6,5] CS命令标识符 */
typedef enum {
	SDO_FAST_DOWN_CSS 		= (uint8_t)(1<<5),	// Client -> Serve		SDO Fast Download(8Byte)	2x(h)
	SDO_FAST_DOWN_SCS 		= (uint8_t)(3<<5),	// Serve  -> Client		SDO Fast Download(8Byte)	60(h)
	SDO_FAST_UPLOAD_CSS		= (uint8_t)(2<<5),	// Client -> Serve		SDO Fast Upload		40(h)
	SDO_FAST_UPLOAD_SCS		= (uint8_t)(2<<5),	// Serve  -> Client		SDO Fast Upload		4x(h)
	SDO_ABORT_CS			= (uint8_t)(4<<5),	// usually Server -> Client,(8Byte), data[4,7] indicate errorCode

	SDO_SEGMENT_DOWN_CSS	= (uint8_t)(0<<5),	// Client -> Serve		SDO Segment Download
	SDO_SEGMENT_DOWN_SCS	= (uint8_t)(1<<5),	// Serve  -> Client 	SDO Segment Download
	SDO_SEGMENT_UPLOAD_CSS	= (uint8_t)(3<<5),	// Client -> Serve		SDO Segment Upload
	SDO_SEGMENT_UPLOAD_SCS	= (uint8_t)(0<<5),	// Serve  -> Client		SDO Segment Upload

	SDO_CS_MASK				= (uint8_t)(7<<5),	// bit[7,6,5] mask
}SDOCSTypeDef;

/* SDO 协议中 Byte0 的 bit[3,2] n 数据d Byte[4,5,6,7]中不包含数据字节数 4-n 个 */
/* 其中e=s=1 SDODataNumTypeDef 有效 */
typedef enum {
	SDO_DATA_NUM_1 = (uint8_t)(3<<2),	// SDO中数据d部分有效数据1字节 Byte[4]
	SDO_DATA_NUM_2 = (uint8_t)(2<<2),	// SDO中数据d部分有效数据2字节 Byte[4,5]
	SDO_DATA_NUM_3 = (uint8_t)(1<<2),	// SDO中数据d部分有效数据3字节 Byte[4,5,6]
	SDO_DATA_NUM_4 = (uint8_t)(0<<2),	// SDO中数据d部分有效数据4字节 Byte[4,5,6,7]
}SDODataNumTypeDef;

/* SDO 协议中 Byte0 的 bit1 e 传送类型控制位 */
typedef enum {
	SDO_TRANSFER_SEGMENT_TYPE 	= (uint8_t)(0<<1),	// SDO的传送类型 Segment方式
	SDO_TRANSFER_FAST_TYPE		= (uint8_t)(1<<1),	// SDO的传送类型 Fast   方式
}SDOTransferTypeDef;

/* SDO 协议中 Byte0 的 bit0 s 控制SDO数据帧是否指明了数据大小 */
typedef enum {
	SDO_DATA_SIZE_NO_INDICATE	= (uint8_t)(0<<0),	// 不指定数据大小
	SDO_DATA_SIZE_INDICATE	= (uint8_t)(1<<0),		// 指定数据大小
}SDODataSizeIndicateTypeDef;

/* SDO Segment 协议中多段读写控制标志位 bit4 */
typedef enum {
	SDO_SEGMENT_TOGGLE_ODD	=	(uint8_t)(0<<4),	// 偶数段2,4,6,...,n
	SDO_SEGMENT_TOGGLE_EVEN	=	(uint8_t)(1<<4),	// 奇数段1,3,5,...,n-1
}SDOSegmentToggle;

/* RSDO 快速下载服务 发送 */
int CAN_SDO_Fast_Write(canDriver *driver, uint16_t NodeID, const ODOBJTypeDef *pOd, uint8_t pData[]);
/* RSDO 快速上载服务 发送 */
int CAN_SDO_Fast_Read(canDriver *driver, uint16_t NodeID, const ODOBJTypeDef *pOd);
/* RSDO 段上载服务 --- 奇数段1,3,5...发送 */
int CAN_SDO_Segment_Read_Odd(canDriver *driver, uint16_t NodeID);
/* RSDO 段上载服务 --- 偶数段2,4,6...发送 */
int CAN_SDO_Segment_Read_Even(canDriver *driver, uint16_t NodeID);

#endif /* CANOPEN_INC_SDO_H_ */

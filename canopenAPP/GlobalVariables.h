#ifndef __GLOBAL_VARIABLES_H__
#define __GLOBAL_VARIABLES_H__

#include "canopen.h"
#include "InterPosition.h"

extern canDriver kavaser;
/* 外部定义 */
extern CAN_RxHeaderTypeDef	canRxHeader;
extern uint8_t				canRxData[8];
extern CANRxDataTypeDef canRxDataStruct;
extern CANRxDataFlagTypeDef canRxDataTypeFlag;
extern uint16_t canRxWaitTime;	// 超时等待，添加到 stm32f4xxx_it.c 中 递减计时 ms 单位
extern uint64_t RxResult;


extern TPDODatasTypeDef tpdo1DataStruct;
extern TPDODatasTypeDef tpdo2DataStruct;
extern canEmergencyTypeDef canEmergencyDataStruct;


extern ManipulatorStateTypeDef ManipulatorState;
extern IPFlagValueTypeDef 		ipModeFlags;
extern bool canReadWaitFlag;

#endif

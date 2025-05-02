#include "GlobalVariables.h"
canDriver kavaser(0,500);

CAN_RxHeaderTypeDef	canRxHeader;
uint8_t				canRxData[8];

CANRxDataTypeDef canRxDataStruct = {0,0,{0}};
CANRxDataFlagTypeDef canRxDataTypeFlag = {0};

uint16_t canRxWaitTime = 10;	
uint64_t RxResult;

TPDODatasTypeDef tpdo1DataStruct = {0};
TPDODatasTypeDef tpdo2DataStruct = {0};

canEmergencyTypeDef canEmergencyDataStruct = {0};

ManipulatorStateTypeDef ManipulatorState={{0},{0},{0},{0},{1,2,3,4,5,6,7},6,7};

// IP模式相关的标志位声明与定义
IPFlagValueTypeDef 		ipModeFlags = IP_Flag_Clc;
bool canReadWaitFlag = true;


//第一套机械臂MDH参数
double alpha1= -0.0005,  a1= -0.0403,       theta1= -0.0078,      d1=0.0701;
double alpha2= -1.5717,  a2= 0.1151,        theta2= -1.3797,      d2=0;
double alpha3= 3.1448,   a3= 0.3702,        theta3= -1.3942,      d3=0;
double alpha4= 1.5708,   a4= 0.0498,        theta4=  3.1808,      d4=0.2294;
double alpha5= -1.5699,  a5= 0,             theta5= -0.0106,      d5=0;
double alpha6= 1.5696,   a6= 0,             theta6=       0,      d6=0.3037;






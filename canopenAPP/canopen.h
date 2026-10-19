/*
 * canopen.h
 *
 *  Created on: Oct 11, 2021
 *      Author: maxiufeng
 */

#ifndef CANOPEN_INC_CANOPEN_H_
#define CANOPEN_INC_CANOPEN_H_


#include "canDriver.h"
#include <unistd.h> 
#include "OD.h"

#include "sdo.h"
#include "pdo.h"
#include "sync.h"
#include "nmt.h"

typedef struct 
{
    /* data */
    int32_t    ActualPosition[6];
    int32_t     ActualVelocity[6];
    int16_t     ActualCurrent[6];
    uint16_t    ActualStatus[6];
    uint8_t     NodeID[7];
    uint8_t     NumOfJoint;
    uint8_t     NumOfMotor;

}ManipulatorStateTypeDef;

typedef struct
{
    double m_targetposition[6];
    double m_targetCartPositionRPY[6];
    uint16_t m_waypointnumber;
    uint16_t m_waypointcount;
}ManipulatorTargetStateTypeDef;



/* CAN receive data structure */
typedef struct {
	uint32_t	m_NodeID;
	uint32_t	m_DLC;
	uint8_t		m_CanRxData[8];
}CANRxDataTypeDef;

/* Received the flag of the data type */
typedef struct {
	uint8_t	m_NMT_flag;
	uint8_t	m_EMYC_flag;
	uint8_t	m_TPDO1_flag;
	uint8_t	m_TPDO2_flag;
	uint8_t	m_TPDO3_flag;
	uint8_t	m_TPDO4_flag;
	uint8_t	m_TSDO_flag;
	uint8_t	m_NODE_GUARD_flag;
}CANRxDataFlagTypeDef;

/* TPDO 的数据，从机发上来的数据记录结构 */
typedef struct {
	CANRxDataTypeDef tpdoData[7];
	uint8_t actualNum;
}TPDODatasTypeDef;

/* 存储can节点的 CanID值，和个数 */
typedef struct {
	uint8_t canIDs[7];
	uint8_t canIDNum;
}canIDsTypeDef;

typedef struct {
	uint8_t nodeID[7];
	int32_t position[7];
	int32_t velocity[7];
	uint8_t nodeNum;
}NodePosVelValueTypeDef;

typedef struct {
	uint8_t nodeID[7];
	uint16_t status[7];
	uint8_t nodeNum;
}NodeStatusTypeDef;

/* 存储can节点的 Emergency Code */
typedef struct {
	uint16_t errorCode;
	uint8_t	errorRegister;
	uint8_t elmoErrorCode;
	uint16_t errorCodeDataField1;
	uint16_t errorCodeDataField2;
}canEmergencyTypeDef;

/* Control Word set value */
typedef enum {
	/* 和驱动器状态机相关的控制位取值 bit3~0,7 其他位不影响状态机的状态 */
	CW_CLEAR						=	(uint16_t)(0x00),
	CW_SHUT_DOWN					=	(uint16_t)(0x06),
	CW_SWITCH_ON					=	(uint16_t)(0x07),
	CW_EN_OP						=	(uint16_t)(0x0F),
	CW_DIS_VOL						=	(uint16_t)(0x00),
	CW_QUICK_STOP					=	(uint16_t)(0x02),
	CW_DIS_OP						=	(uint16_t)(0x07),
	CW_FAULT_RESELT					=	(uint16_t)(0x80),
	CW_MASK_BIT73210				=	(uint16_t)((1<<7)|0x0F),
/* Point to Point mode Control Word setting value */
	CW_PTP_NEW_SET_POINT			=	(uint16_t)(0x01<<4),	// 设置新点的标志信号 0→1，上升沿有效
	CW_PTP_CHANGE_SET_POINT_IMM		=	(uint16_t)(0x01<<5),	// 设置新点是否立即执行的标志位
	CW_PTP_ABS_MOVEMENT				=	(uint16_t)(0x00<<6),	// default 绝对运动
	CW_PTP_REL_MOVEMENT				=	(uint16_t)(0x01<<6),	// 相对运动
	CW_PTP_HALT_STOP				=	(uint16_t)(0x01<<8),	// 电机停止 电机状态不变，处于操作模式下 605D设置为2(def)
	CW_PTP_BLENDED					=	(uint16_t)(0x01<<9),	// 混合控制位
/* profile velocity mode Control Word setting value */
	CW_PV_HALT						=	(uint16_t)(0x01<<8),	//bit4 = 1, ip-enable
/* profile torque mode control word setting value */
	CW_PT_HALT						=   (uint16_t)(0x01<<8),
/* IP mode Control word setting value */
	CW_IP_ENABLE					=	(uint16_t)(0x01<<4),	// 使能插值
	CW_IP_HALT						=	(uint16_t)(0x01<<8),	// 进入此状态 IP 不在活跃
}CWValTypeDef;


/* General purpose Digital Output 数字量输出控制对象取值 Gold驱动器有4个DO端口 */
	//	bit | 7 | 6 | 5 | 4 |     3    |  2  | 1  |  0 |
	//	    ---------------------------------------
	//   DO | x | x | x | x |    3v3   | 3v3 | 5v | 5v |
	//	说明					  引出控制24v  未引出  引出  引出
typedef enum {
	DO_1_5V_LEAD_ON		=  (uint8_t)(1<<0),
	DO_1_5V_LEAD_OFF	= (~(uint8_t)(1<<0))&0x0F,
	DO_2_5V_LEAD_ON		=  (uint8_t)(1<<1),
	DO_2_5V_LEAD_OFF	= (~(uint8_t)(1<<1))&0x0F,
	DO_3_3V3_NO_ON		=  (uint8_t)(1<<2),
	DO_3_3V3_NO_OFF		= (~(uint8_t)(1<<2))&0x0F,
	DO_4_3V3_BRAKE_ON	=  (uint8_t)(1<<3),
	DO_4_3V3_BRAKE_OFF	= (~(uint8_t)(1<<3))&0x0F,

	DO_ALL_OFF			=  (uint8_t)(0x00),
}DOValueTypeDef;

/* 控制函数中是否执行打印输出这一部分 */
typedef enum {
	PRINT_INFO_ON	=	(uint8_t)(0x01),
	PRINT_INFO_OFF	=	(uint8_t)(0x00),
}PrintInfoFlagTypeDef;

/* 驱动器状态字值含义的定义 读取结果和这个值做比对 以确定状态 */
typedef enum {
	/* bit0~3,5,6 [xxxx xxxx x@@x @@@@] */
	SW_NOT_READY			=	(uint16_t)((0<<6)|0x00),
	SW_SWITCH_ON_DISABLE	=	(uint16_t)((1<<6)|0x00),
	SW_READY_SWITCH_ON		=	(uint16_t)((1<<5)|0x01),
	SW_SWITCH_ON			=	(uint16_t)((1<<5)|0x03),
	SW_OPERATION_ENABLE		=	(uint16_t)((1<<5)|0x07),
	SW_QUICK_STOP_ACTIVE	=	(uint16_t)((0<<5)|0x07),
	SW_FAULT_REACTION_ACTIVE=	(uint16_t)((0<<6)|0x0F),
	SW_FAULT				=	(uint16_t)((0<<6)|0x08),
	SW_MASK_1				=	(uint16_t)((3<<5)|0x0F),
	/* bit4 Voltage enable */
	SW_VOLTAGE_ENABLE		=	(uint16_t)(1<<4),
	/* bit7 Warning */
	SW_WARNING				=	(uint16_t)(1<<7),
	/* bit10 Target Reached flag */
	SW_TARGET_REACHED		=	(uint16_t)(1<<10),	//0:No reached 1: Reached
/* PP 模式下 相关的状态位 bit12 13 */
	SW_PP_NEW_POINT_ACK		=	(uint16_t)(1<<12),	//0:设定点buffer可用，之前的点已经处理了 1:设定点buffer不可用，之前的点还在处理中
	SW_PP_FOLLOW_ERROR		=	(uint16_t)(1<<13),  //0:没有跟踪误差 1:有跟踪误差
/* PV 模式下 相关状态 */
	SW_PV_SPEDD_ZERO		=	(uint16_t)(1<<12),	//0:速度不为0 1:速度=0
/* IP 模式下 相关状态 */
	SW_IP_INTERPOLATION_ACTIVE = (uint16_t)(1<<12),	//0:IP not active 1:IP Active
}SWValTypeDef;

/* 运动模式对象值 定义 pp,pv,tp,ip... */
typedef enum {
	MOTION_MODE_CLC	=	(int8_t)(0),
	MOTION_MODE_PP	=	(int8_t)(1),
	MOTION_MODE_PV	=	(int8_t)(3),
	MOTION_MODE_TP	=	(int8_t)(4),
	MOTION_MODE_HM	=	(int8_t)(6),
	MOTION_MODE_IP	=	(int8_t)(7),
	MOTION_MODE_CSP	=	(int8_t)(8),
	MOTION_MODE_CSV	=	(int8_t)(9),
	MOTION_MODE_CST	=	(int8_t)(10),
}MotionModeTypeDef;

/* Positioning Option Code(POC) 包括 relative Option(0,1)、cio(2,3)、rro(4,5)、rado(6,7)
 * 定义 定位行为的配置选项代码*/
typedef enum {
	// CW.bit6=1,相对运动方式下RO有效
	POC_RO_PRE_TARGET_POSITION	= (uint16_t)(0<<0),		// 相对前一个目标位置(607A)的相对运动 default
	POC_RO_ACTUAL_POSITION		= (uint16_t)(2<<0),		// 相对当前电机的实际位置(6064)相对运动
	// CW.bit5 change set point immediately
	POC_CIO_IMMEDIATELY			= (uint16_t)(0<<2),		// 收到新的点立即执行 default
	POC_CIO_CONTINUE_BLEND		= (uint16_t)(1<<2),		// 到达目标位置，这个点将继续并和下个新点混合执行
	// request-response option 这个选项配置驱动器如何去释放 CW.bit4(New set-point)
	POC_RRO_NONE_DEF			= (uint16_t)(0<<4),		// 完全按照主机发送的控制字做出响应 default
	POC_RRO_RELEASE_REACHED		= (uint16_t)(1<<4),		// 自动释放bit4，一旦达到目标位置
	POC_RRO_RELEASE_BUFFER		= (uint16_t)(2<<4),		// 自动释放bit4，如果有可用的buffer
	// 电机轴的转动方式 四 种
	POC_RADO_LINEAR_AXIS		= (uint16_t)(0<<6),		// 类似线性的一种运动
	POC_RADO_NEGATIVE_DIRECTION = (uint16_t)(1<<6),		// 反转
	POC_RADO_POSITIVE_DIRECTION = (uint16_t)(2<<6),		// 正转
	POC_RADO_SHORTEST_WAY		= (uint16_t)(3<<6),		// 最短路径转动
}POCTypeDef;

typedef enum {
	IP_SUB_MODE_POS 	= (int16_t) (0),
	IP_SUB_MODE_POS_VEL = (int16_t)(-1),
}IPSubModeTypeDef;	// IP子模式设定参数值

typedef enum {
	IP_TIME_INDEX_10_2	=	(int8_t)(-2),	// 10^-2 s
	IP_TIME_INDEX_10_3	=	(int8_t)(-3),	// 10^-3 s (default ms) 单位
	IP_TIME_INDEX_10_4	=	(int8_t)(-4),	// 10^-4 s
	IP_TIME_INDEX_10_5	=	(int8_t)(-5),	// 10^-5 s
	IP_TIME_INDEX_10_6	=	(int8_t)(-6),	// 10^-6 s
}IPTimeIndexTypeDef;	// 插值周期的单位设置参数值

typedef enum {
	IP_BUF_ORG_FIFO		=	(uint8_t)(0),
	IP_BUF_ORG_RING		=	(uint8_t)(1),
}IPBufOrgTypeDef;	// buffer 的实现方法的参数值

typedef enum {
	IP_BUF_CTRL_CLEAR_DISABLE	=	(uint8_t)(0),	// 清空、   禁用buffer
	IP_BUF_CTRL_ENABLE			=	(uint8_t)(1),	// 使能、允许访问buffer
}IPBufCtrlTypeDef;	// buffer 控制参数值

extern 	CANRxDataTypeDef canRxDataStruct;
extern  CANRxDataFlagTypeDef canRxDataTypeFlag;
extern  uint64_t RxResult;

extern  TPDODatasTypeDef tpdo1DataStruct;
extern  TPDODatasTypeDef tpdo2DataStruct;



/* 根据接收消息标志位对消息响应数据作处理 */
int CAN_Rx_Data_Handle(canDriver *driver, uint16_t NodeID);

/* 从机pdo发上来的消息 */
void CAN_Rx_TPDOx_Data_Handle(TPDODatasTypeDef *tpdoxDataStruct);

/* ****************************** SDO 方式数据交互相关函数方法实现(写入-设置相关) ****************************** */
/* SDO方式控制指定的数字量输出端口开关 */
int CAN_SDO_Digital_Output(canDriver *driver, uint16_t NodeID, DOValueTypeDef DOVal, uint64_t *resVal);

/* 写驱动器控制字 切换状态 */
int CAN_SDO_ControlWord(canDriver *driver, uint16_t NodeID, CWValTypeDef CWVal, uint16_t *resVal);

/* 写驱动器运动模式对象 设定具体的运动模式 PP,PV,TP,IP,CSP,... */
int CAN_SDO_Motion_Mode(canDriver *driver, uint16_t NodeID, MotionModeTypeDef motionMode, uint32_t *resVal);

/* 写驱动器 快速停止选项代码 Quick stop option code 1 2 5 6 */
int CAN_SDO_Quick_Stop_Option_Code(canDriver *driver, uint16_t NodeID, int16_t qsoc, uint32_t *resVal);

/* 写驱动器 位置控制功能里的 Positioning Option Code 用过个时使用或操作进行 */
int CAN_SDO_Position_Option_Code(canDriver *driver, uint16_t NodeID, POCTypeDef poc, uint32_t *resVal);

/* SDO方式写驱动器 PP 模式下，设置目标位置 */
int CAN_SDO_PP_Target_Position(canDriver *driver, uint16_t NodeID, int32_t posVal, uint32_t *resVal);

/* SDO方式写驱动器 PP 模式下，设置轮廓速度 */
int CAN_SDO_PP_Profile_Velocity(canDriver *driver, uint16_t NodeID, uint32_t velVal, uint32_t *resVal);

/* SDO方式写驱动器 PP 模式下，设置轮廓加速度 */
int CAN_SDO_PP_Profile_Acceleration(canDriver *driver, uint16_t NodeID, uint32_t accVal, uint32_t *resVal);

/* SDO方式写驱动器 PP 模式下，设置轮廓减速度 */
int CAN_SDO_PP_Profile_Deceleration(canDriver *driver, uint16_t NodeID, uint32_t decVal, uint32_t *resVal);

/* SDO方式写驱动器 PP 模式下，设置快速停止减速度 */
int CAN_SDO_PP_Quick_Stop_Deceleration(canDriver *driver, uint16_t NodeID, uint32_t decVal, uint32_t *resVal);

/* ************** 设置Profile Velocity 的有关参数 ******************** */
int CAN_SDO_PV_Target_Velocity(canDriver *driver, uint16_t NodeID, int32_t velVal, uint32_t *resVal);
/* SDO方式写驱动器 PV 模式下，设置轮廓加速度 */
int CAN_SDO_PV_Profile_Acceleration(canDriver *driver, uint16_t NodeID, uint32_t accVal, uint32_t *resVal);
/* SDO方式写驱动器 PV 模式下，设置轮廓减速度 */
int CAN_SDO_PV_Profile_Deceleration(canDriver *driver, uint16_t NodeID, uint32_t decVal, uint32_t *resVal);
/* SDO方式写驱动器 PV 模式下，设置快速停止减速度 */
int CAN_SDO_PV_Quick_Stop_Deceleration(canDriver *driver, uint16_t NodeID, uint32_t decVal, uint32_t *resVal);

/* SDO方式写驱动器 PT 模式下，设置目标扭矩、电流 */
int CAN_SDO_PT_Target_Torque_Current(canDriver *driver, uint16_t NodeID, int16_t torqueVal, uint32_t *resVal);

/* SDO方式写驱动器 IP 模式下，设置轮廓加速度 */
int CAN_SDO_IP_Profile_Acceleration(canDriver *driver, uint16_t NodeID, uint32_t accVal, uint32_t *resVal);
/* SDO方式写驱动器 IP 模式下，设置轮廓减速度 */
int CAN_SDO_IP_Profile_Deceleration(canDriver *driver, uint16_t NodeID, uint32_t decVal, uint32_t *resVal);
/* SDO方式写驱动器 IP 模式下，设置快速停止减速度 */
int CAN_SDO_IP_Quick_Stop_Deceleration(canDriver *driver, uint16_t NodeID, uint32_t decVal, uint32_t *resVal);
/* SDO方式写驱动器 IP 模式下，设置IP子模式0、-1 */
int CAN_SDO_IP_Sub_Mode_Select(canDriver *driver, uint16_t NodeID, IPSubModeTypeDef subMode, uint32_t *resVal);
/* SDO方式写驱动器 IP 模式下，设置IP的周期值 1-10ms */
int CAN_SDO_IP_Time_Record_Period(canDriver *driver, uint16_t NodeID, uint8_t period, uint32_t *resVal);
/* SDO方式写驱动器 IP 模式下，设置IP的周期值的单位 如：10^-1 -2 -3 -4 s (-2 ~ -6) def = -3 -> 10^-3s=1ms */
int CAN_SDO_IP_Time_Record_Period_Index(canDriver *driver, uint16_t NodeID, IPTimeIndexTypeDef timeIndex, uint32_t *resVal);
/* SDO方式写驱动器 IP 模式下，设置IP的buffer实际大小 (与子模式的值有关联 sub=0: 1, sub=-1: 1~16) */
int CAN_SDO_IP_Data_Config_Actual_Buf_Size(canDriver *driver, uint16_t NodeID, uint32_t ActualBufSize, uint32_t *resVal);
/* SDO方式写驱动器 IP 模式下，设置IP的buffer组织形式 */
int CAN_SDO_IP_Data_Config_Buf_Org(canDriver *driver, uint16_t NodeID, IPBufOrgTypeDef orgMode, uint32_t *resVal);
/* SDO方式写驱动器 IP 模式下，设置IP的buffer的位置值 */
int CAN_SDO_IP_Data_Config_Buf_Pos(canDriver *driver, uint16_t NodeID, uint16_t bufPos, uint32_t *resVal);
/* SDO方式写驱动器 IP 模式下，设置IP的buffer控制对象 实现清空、禁用、允许访问操作 */
int CAN_SDO_IP_Data_Config_Buf_Control(canDriver *driver, uint16_t NodeID, IPBufCtrlTypeDef bufCtrl, uint32_t *resVal);

/* SDO方式写驱动器 电机停止前外推插值周期个数 1~32768 */
int CAN_SDO_IP_Extrapolation_Cycles_Timeout(canDriver *driver, uint16_t NodeID, int16_t cycleNum, uint32_t *resVal);

/* ****************************** SDO 方式数据交互相关函数方法实现(读取-上传相关) ****************************** */
/* SDO 段上载服务 读多个字节数据/字符串数据 对《厂商设备名》《厂商硬件版本》《厂商软件版本》的字符串读取 */
int CAN_SDO_Segment_Read(canDriver *driver, uint16_t NodeID, const ODOBJTypeDef *pOd, uint8_t resArr[]);

/* 读Digital Output的当前值 resVal 反馈当前Digital Output的状态值 */
int CAN_SDO_Digital_Output_Status(canDriver *driver, uint16_t NodeID, uint64_t *resVal);

/* SDO读取驱动器5v电压值 mV单位， resval 为该值 */
int CAN_SDO_5V_DC_Supply(canDriver *driver, uint16_t NodeID, int16_t *resVal);

/* SDO读取驱动器48V电压值 mV单位，resval存放结果 */
int CAN_SDO_48V_DC_Supply(canDriver *driver, uint16_t NodeID, uint32_t *resVal);

/* SDO读取驱动器48v回路上的电流值，结果放在resVal中 */
int CAN_SDO_Current_Value(canDriver *driver, uint16_t NodeID, int16_t *resVal);

/* SDO读取驱动器摄氏温度值 resVal接收温度值 */
int CAN_SDO_Drive_Temperature(canDriver *driver, uint16_t NodeID, uint16_t *resVal);

/* SDO读取ADC通道1的电压值，转换成电机线圈的温度 */
int CAN_SDO_Analog_Mv_Temperature(canDriver *driver, uint16_t NodeID, int16_t *resVal);

/* SDO读取设备类型对象，正常情况下返回的是固定值 @DEVICE_TYPE_VALUE */
int CAN_SDO_Device_Type(canDriver *driver, uint16_t NodeID, uint32_t *resVal);

/* SDO读取驱动器的状态字 并返回原始的 状态值 或 错误代码 结果 */
int CAN_SDO_StatusWord(canDriver *driver, uint16_t NodeID, uint16_t *resVal, PrintInfoFlagTypeDef printFlag);

/* SDO读取驱动器控制字的当前值 返回该值 或 错误代码 结果 */
int CAN_SDO_ControlWord_Value(canDriver *driver, uint16_t NodeID, uint16_t *CWValue);

/* SDO读取驱动器当前的运动模式对象 返回该值 或 错误代码值 */
int CAN_SDO_Motion_Mode_Display(canDriver *driver, uint16_t NodeID, int32_t *MotionModeValue);

/* SDO读驱动器 位置控制功能里的 Positioning Option Code 的值 */
int CAN_SDO_Position_Option_Code_Value(canDriver *driver, uint16_t NodeID, uint16_t *poc);

/* SDO读驱动器 PP 模式下，设置的目标位置值 */
int CAN_SDO_PP_Target_Position_Value(canDriver *driver, uint16_t NodeID, int32_t *resVal);

/* SDO读驱动器 PP 模式下，设置的轮廓速度值 */
int CAN_SDO_PP_Profile_Velocity_Value(canDriver *driver, uint16_t NodeID, uint32_t *resVal);

/* SDO读驱动器 PP 模式下，设置的轮廓加速度值 */
int CAN_SDO_PP_Profile_Acceleration_Value(canDriver *driver, uint16_t NodeID, uint32_t *resVal);

/* SDO读驱动器 PP 模式下，设置的轮廓减速度值 */
int CAN_SDO_PP_Profile_Deceleration_Value(canDriver *driver, uint16_t NodeID, uint32_t *resVal);

/* SDO方式读驱动器 PP 模式下，设置快速停止减速度 */
int CAN_SDO_PP_Quick_Stop_Deceleration_Value(canDriver *driver, uint16_t NodeID, uint32_t *resVal);

/* SDO方式读驱动器 PV 模式下，目标速度值 */
int CAN_SDO_PV_Target_Velocity_Value(canDriver *driver, uint16_t NodeID, int32_t *velVal);
/* SDO方式读驱动器 PV 模式下，轮廓加速度 */
int CAN_SDO_PV_Profile_Acceleration_Value(canDriver *driver, uint16_t NodeID, uint32_t *accVal);
/* SDO方式读驱动器 PV 模式下，轮廓减速度 */
int CAN_SDO_PV_Profile_Deceleration_Value(canDriver *driver, uint16_t NodeID, uint32_t *decVal);
/* SDO方式读驱动器 PV 模式下，快速停止减速度 */
int CAN_SDO_PV_Quick_Stop_Deceleration_Value(canDriver *driver, uint16_t NodeID, uint32_t *qdecVal);

/* SDO方式读驱动器 PT 模式下，目标扭矩、电流 */
int CAN_SDO_PT_Target_Torque_Current_Value(canDriver *driver, uint16_t NodeID, int16_t *torqueVal);
/* SDO方式读驱动器 PT 模式下，电机的连续运行（额定）电流值 mA */
int CAN_SDO_PT_Motor_Rated_Current_Value(canDriver *driver, uint16_t NodeID, uint32_t *motorRatedCurrent);


/* SDO方式读驱动器 IP 模式下，轮廓加速度 */
int CAN_SDO_IP_Profile_Acceleration_Value(canDriver *driver, uint16_t NodeID, uint32_t *accVal);
/* SDO方式读驱动器 IP 模式下，轮廓减速度 */
int CAN_SDO_IP_Profile_Deceleration_Value(canDriver *driver, uint16_t NodeID, uint32_t *decVal);
/* SDO方式读驱动器 IP 模式下，快速停止减速度 */
int CAN_SDO_IP_Quick_Stop_Deceleration_Value(canDriver *driver, uint16_t NodeID, uint32_t *qdecVal);
/* SDO方式读驱动器 IP 模式下，IP子模式0、-1 */
int CAN_SDO_IP_Sub_Mode_Select_Value(canDriver *driver, uint16_t NodeID, IPSubModeTypeDef *subMode);
/* SDO方式读驱动器 IP 模式下，IP的周期值 1-10ms */
int CAN_SDO_IP_Time_Record_Period_Value(canDriver *driver, uint16_t NodeID, uint8_t *period);
/* SDO方式读驱动器 IP 模式下，IP的周期值的单位 如：10^-1 -2 -3 -4 s (-2 ~ -6) def = -3 -> 10^-3s=1ms */
int CAN_SDO_IP_Time_Record_Period_Index_Value(canDriver *driver, uint16_t NodeID, IPTimeIndexTypeDef *timeIndex);
/* SDO方式读驱动器 IP 模式下，IP的buffer最大值 (虽然可读可写 - 但是只要子模式已经确定这个对象的值也跟着确定了 sub=0: 1, sub=-1: 16) */
int CAN_SDO_IP_Data_Config_Max_Buf_Size_Value(canDriver *driver, uint16_t NodeID, uint32_t *MaxBufSize);
/* SDO方式读驱动器 IP 模式下，IP的buffer实际大小 (与子模式的值有关联 sub=0: 1, sub=-1: 1~16) */
int CAN_SDO_IP_Data_Config_Actual_Buf_Size_Value(canDriver *driver, uint16_t NodeID, uint32_t *ActualBufSize);
/* SDO方式读驱动器 IP 模式下，IP的buffer组织形式 */
int CAN_SDO_IP_Data_Config_Buf_Org_Value(canDriver *driver, uint16_t NodeID, IPBufOrgTypeDef *orgMode);
/* SDO方式读驱动器 IP 模式下，IP的buffer的位置值 */
int CAN_SDO_IP_Data_Config_Buf_Pos_Value(canDriver *driver, uint16_t NodeID, uint16_t *bufPos);
/* SDO方式读驱动器 IP 模式下，IP的每条数据记录的大小 以字节为单位 sub=0: 4, sub=-1: 8 */
int CAN_SDO_IP_Data_Config_Data_Record_Size_Value(canDriver *driver, uint16_t NodeID, uint8_t *dataRecordSize);

/* SDO方式写驱动器 电机停止前外推插值周期个数 1~32768 */
int CAN_SDO_IP_Extrapolation_Cycles_Timeout_Value(canDriver *driver, uint16_t NodeID, int16_t *cycleNumValue);



/* SDO读取驱动器里电机的位置值 (以用户定义的单位给出) int32 *position 返回实际的位置 */
int CAN_SDO_Actual_Position_User(canDriver *driver, uint16_t NodeID, int32_t *position);

/* SDO读驱动器里电机位置控制命令值 demmand value int32 */
int CAN_SDO_Demand_Position_Value_User(canDriver *driver, uint16_t NodeID, int32_t *demmandPos);

/* SDO读取驱动器里电机的速度值 (以用户定义的单位给出) int32 *velocity 返回实际的速度 */
int CAN_SDO_Actual_Velocity_User(canDriver *driver, uint16_t NodeID, int32_t *velocity);

/* SDO读取驱动器里电机的位置跟踪误差 */
int CAN_SDO_Position_Following_Error_User(canDriver *driver, uint16_t NodeID, int32_t *posError);

/* SDO读取驱动器的位置环的输出量 control effort */
int CAN_SDO_Position_Control_Effort(canDriver *driver, uint16_t NodeID, int32_t *posControlEffort);

#endif /* CANOPEN_INC_CANOPEN_H_ */

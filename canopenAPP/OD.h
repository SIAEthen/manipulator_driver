/*
 * OD.h
 *	CANopen 对象“字典”定义
 *  Created on: Oct 11, 2021
 *      Author: maxiufeng
 */

#ifndef CANOPEN_INC_OD_H_
#define CANOPEN_INC_OD_H_

#include <stdint.h>
#include "canDriver.h"

/* Function Code  COBID 的 bit[10,9,8,7] 		    COBID 的范围*/
#define	FC_NMT			(uint16_t)((0x00)<<7)	//	0
#define	FC_SYNC			(uint16_t)((0x01)<<7)	//	128(80h)
#define	FC_TIME_STAMP	(uint16_t)((0x02)<<7)	//	256(100h)
#define	FC_EMERGENCY	(uint16_t)((0x01)<<7)	//	129...255(81h...ffh)
#define	FC_TPDO1		(uint16_t)((0x03)<<7)	//	385...511(181h...1ffh
#define	FC_RPDO1		(uint16_t)((0x04)<<7)	//	513...639(201h...27fh)
#define	FC_TPDO2		(uint16_t)((0x05)<<7)	//	641...767(281h...2ffh)
#define	FC_RPDO2		(uint16_t)((0x06)<<7)	//	769...895(301h...37fh)
#define	FC_TPDO3		(uint16_t)((0x07)<<7)	//	897...1023(381h...3ffh)
#define	FC_RPDO3		(uint16_t)((0x08)<<7)	//	1025...1151(401h...47fh)
#define	FC_TPDO4		(uint16_t)((0x09)<<7)	//	1153...1279(481h...4ffh)
#define	FC_RPDO4		(uint16_t)((0x0A)<<7)	//	1281...1407(501h...57fh)
#define	FC_TSDO			(uint16_t)((0x0B)<<7)	//	1409...1535(581h...5ffh)
#define	FC_RSDO			(uint16_t)((0x0C)<<7)	//	1537...1663(601h...67fh)
#define	FC_ERRCONTROL_NODEGUARD		(uint16_t)((0x0E)<<7)	//	1793...1919(701h...77fh) Bootup、HeartBeat、从节点发出的NMT消息

#define	FC_MASK			(uint16_t)((0x0F)<<7)	//	COBID FunctionCode 掩码 高4位
#define	ID_MASK			(uint16_t)((0x7F)<<0)	//	COBID NodeID 掩码 低7位

/* COBID 包括 Function Code 和 NodeID 两部分 此处是固定的COBID 优先级最高 */
#define	COBID_TIME_STAMP	FC_TIME_STAMP	// 时间戳 COBID = 256(00h)

/* 定义字典中的对象读写类型 */
typedef enum {
	OBJ_READ_ONLY 	= 	(uint8_t)(0x01),						// 对象只读
	OBJ_WRITE_ONLY	=	(uint8_t)(0x02),						// 对象只写
	OBJ_READ_WRITE	=	(uint8_t)(OBJ_READ_ONLY|OBJ_WRITE_ONLY),	// 可读可写
}OBJReadWriteTypeDef;


/**
 * Define Object Dictionary member structure
 */
typedef struct {
	uint16_t			index;		// 对象索引  16bit
	uint8_t				sub_index;	// 对象子索引  8bit
	OBJReadWriteTypeDef	is_R_W;		// 对象读写属性
	uint8_t				data_len;	// 对象对应的数据长度
}ODOBJTypeDef;





// 自定义联合类型 uint32 ←→ uint[4]
typedef union {
	uint8_t array[4];
	uint32_t value;
}unionUint32ArrayTypeDef;

/* 字典对象的数据长度 和SDO命令中有效字节数一致 应用中(4-OBJDataNumTypeDef>>2)为实际的有效字节个数 */
typedef enum {
	OBJ_DATA_NUM_1 = (uint8_t)(3<<2),
	OBJ_DATA_NUM_2 = (uint8_t)(2<<2),
	OBJ_DATA_NUM_3 = (uint8_t)(1<<2),
	OBJ_DATA_NUM_4 = (uint8_t)(0<<2),
}OBJDataNumTypeDef;

typedef enum {
	PDO_ENABLE 	= (uint32_t)(0<<31),
	PDO_DISABLE = (uint32_t)(1<<31),
}PDOEnableBitTypeDef;	// PDO 通信参数 sub1.bit31, 控制PDO是否可用

typedef enum {
	PDO_T1 = FC_TPDO1,
	PDO_T2 = FC_TPDO2,
	PDO_T3 = FC_TPDO3,
	PDO_T4 = FC_TPDO4,
	PDO_R1 = FC_RPDO1,
	PDO_R2 = FC_RPDO2,
	PDO_R3 = FC_RPDO3,
	PDO_R4 = FC_RPDO4,
}PDOxTypeDef;	// 所用到 PDO COBID的Function-Code部分 sub1.bit[10-0]


typedef enum {
	PDO_FUNCTION_ENABLE 			= (uint8_t)(1),
	PDO_FUNCTION_DISABLE			= (uint8_t)(2),
	PDO_FUNCTION_SET_TRANS_TYPE		= (uint8_t)(3),
	PDO_FUNCTION_SET_INHIBIT_TIME	= (uint8_t)(4),	// TPDO 通信参数才有
	PDO_FUNCTION_SET_EVENT_TIMER	= (uint8_t)(5),// TPDO 通信参数才有
}PDOxFunctionTypeDef;	// 和PDO通信参数相关 是PDO开启、关闭、设置传输类型的标志信息


typedef enum {
	PDO_Rx_1_SYNC  = (uint8_t)(1),	// 同步，1个同步信号触发	*RPDO同步传输仅有这一种

	PDO_Tx_1SYNC   = (uint8_t)(1),	// 同步，  1个同步信号触发
	PDO_Tx_2SYNC   = (uint8_t)(2),	// 同步，  2个同步信号触发
	PDO_Tx_3SYNC   = (uint8_t)(3),	// 同步，  3个同步信号触发
	PDO_Tx_4SYNC   = (uint8_t)(4),	// 同步，  4个同步信号触发
	PDO_Tx_5SYNC   = (uint8_t)(5),	// 同步，  5个同步信号触发
	/* 1~240个SYNC信号触发 */
	PDO_Tx_240SYNC   = (uint8_t)(240),// 同步，240个同步信号触发

	PDO_ASYNC_1 = (uint8_t)(254),// 异步，254 事件驱动-厂家自己定义的映射的对象改变或内部定时发送触发
	PDO_ASYNC_2 = (uint8_t)(255),// 异步，255 事件驱动-设备协议决定

}PDOxTransmissionTypeDef;	// PDO 的 communicationParameter.sub2




/* 宏定义对象索引 及存放的固定值 */
/* 设备类型对象 *********************************************************************************************/
#define	OD_DEVICE_TYPE					(uint16_t)(0x1000)		// uint32 RO
#define	   DEVICE_TYPE_VALUE			(uint32_t)(0x020192)	// 设备类型对象的结果是一个固定值

/* 制造生设备名字信息 *****************************************************************************************/
#define OD_MANUFACTURER_DEVICE_NAME		(uint16_t)(0x1008)		// String	"Twitter"
/* 制造商硬件版本信息 *****************************************************************************************/
#define OD_MANUFACTURER_HARD_VERSION	(uint16_t)(0x1009)		// String	"0x"
/* 制造商软件版本信息 *****************************************************************************************/
#define OD_MANUFACTURER_SOFT_VERSION	(uint16_t)(0x100A)		// String	"Twitter 01.01.16.00 08Mar2020B01G"
/* 制造商程序版本信息 *****************************************************************************************/
#define OD_MANUFACTURER_DSP_VERSION		(uint16_t)(0x100B)		// String	"DSP Boot 1.0.1.6 12Feb2014G"

/* 生产者心跳事件产生时间设置对象 *********************************************************************************/
#define	OD_PRODUCER_HEARTBEAT_TIME		(uint16_t)(0x1017)		// uint16 RW
/* 驱动器模拟量输入端口 检测模拟量 ********************************************************************************/
#define OD_ANALOG_INPUT					(uint16_t)(0x2205)	//  int16 RO
#define    ANALOG_INPUT_NUM_SUB0		(uint8_t)(0x00)		// uint8 RO num = 2, 固定值
#define    ANALOG_INPUT_MV_SUB1			(uint8_t)(0x01)		// int16 RO ADC1 [-10, 10]*1000mv 获取电机温度分压电阻上的模拟电压值 转换成电机线圈的温度值
#define    ANALOG_INPUT_A2D_SUB2		(uint8_t)(0x02)		// int16 RO 12bit[0, 4095]
/* 驱动器 5V DC 电压值 mV ***********************************************************************************/
#define	OD_5V_DC_SUPPLY					(uint16_t)(0x2206)		// int16 RO mV
/* 驱动器直流回路电流 48VDC回路电流 ******************************************************************************/
#define	OD_CURRENT_ACTUAL_VALUE			(uint16_t)(0x6078)	//  int16 RO mA
/* 驱动器直流回路电压 48VDC实际值 *******************************************************************************/
#define OD_48V_DC_VOLTAGE				(uint16_t)(0x6079)		// uint32 RO mV
/* 驱动器温度 摄氏度单位 ***************************************************************************************/
#define OD_DRIVE_TEMPERATURE			(uint16_t)(0x22A2)		// uint16 RO

/* General purpose Digital Output 数字量输出控制对象 Gold驱动器有4个DO端口 *****************************************/
	//	bit | 7 | 6 | 5 | 4 |     3    |  2  | 1  |  0 |
	//	    ---------------------------------------
	//   DO | x | x | x | x |    3v3   | 3v3 | 5v | 5v |
	//	说明					  引出控制24v  未引出  引出  引出
#define OD_DIGITAL_OUTPUT				(uint16_t)(0x22A0)		// uint8  RW

/* 控制字 DS402 *******************************************************************************************/
#define OD_CONTROL_WORD					(uint16_t)(0x6040)		// uint16 RW RxMap

/* 状态字 DS402 *******************************************************************************************/
#define OD_STATUS_WORD					(uint16_t)(0x6041)		// uint16 RO TxMap

/* Quick stop option code */
#define OD_QUICK_STOP_OPTION_CODE		(uint16_t)(0x605A)		// int16 RW 1 2 5 6

/* Modes of operation exp:PP、PV、TQ、IP 等 ****************************************************************/
#define OD_OPERATION_MODES				(uint16_t)(0x6060)		// int8   RW	设置模式
#define OD_OPERATION_MODES_DISPLAY		(uint16_t)(0x6061)		// int8   RO 	获取当前模式
#define	OD_SUPPORTED_DRIVE_MODE			(uint16_t)(0x6502)		// uint32 RO 	获取驱动器支持的所有模式
	//	bit	|31-10| 9 | 8 | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
	//		-----------------------------------------------
	// mode	| --- |cst|csv|csp| ip| hm|---| tq| pv| x | pp|
#define	   SUPPORTED_DRIVE_MODE_VALUE	(uint32_t)(0x03ED)		// Gold Twitter 驱动器支持的模式  固定值


/* PC位置控制功能 position control function *******************************************************************/
#define	OD_PC_POSITION_DEMAND_VALUE			(uint16_t)(0x6062)	//	 int32 RO def=0
#define	OD_PC_POSITION_ACTUAL_INTER_VALUE	(uint16_t)(0x6063)	//	 int32 RO def=0
/* 实际的位置值 以用户定义的单位形式 ****/
#define OD_PC_POSITION_ACTUAL_VALUE			(uint16_t)(0x6064)	//	 int32 RO def=0	*******
#define OD_PC_FOLLOWING_ERROR_WINDOW		(uint16_t)(0x6065)	//	uint32 RW
#define	OD_PC_FOLLOWING_ERROR_TIME_OUT		(uint16_t)(0x6066)	//	uint16 RW def=20
#define OD_PC_POSITION_WINDOW				(uint16_t)(0x6067)  //  uint32 RW def=100
#define OD_PC_POSITION_WINDOW_TIME			(uint16_t)(0x6068)	//  uint16 RW def=20
#define OD_PC_FOLLOWING_ERROR_ACTUAL_VALUE	(uint16_t)(0x60F4)	//   int32 RO def=0
#define OD_PC_CONTROL_EFFORT				(uint16_t)(0x60FA)	//   int32 RO def=0
#define OD_PC_POSITION_DEMAND_INTER_VALUE	(uint16_t)(0x60FC)	//   int32 RO def=0
/* Positioning option code 其中决定rado方式的是(bit6-7)*/
	//	bit	| 15| 14-12 |  11- 8  | 7-6           | 5-4        | 3-2        |      1 - 0                       |
	//		-----------------------------------------------------------
	// mode	| ms| ----- |ip option| rado - 旋转轴方向| rro - 请求响应| cio - 立即更改| relative option - 相对运动的相对参考对象|
#define	OD_PC_POSITIONING_OPTION_CODE		(uint16_t)(0x60F2)		// uint16 RW def=0

/* Position range limit **********************************************************************************/
#define OD_POSITION_RANGE_LIMIT				(uint16_t)(0x607B)
#define	   POSITION_RANGE_LIMIT_MEM_SUB0	(uint8_t)(0x00)		// 成员为 2 RO	固定值
#define	   POSITION_RANGE_LIMIT_MIN_SUB1	(uint8_t)(0x01)		// int32 RW		// elmo 软件里设置成了 0
#define	   POSITION_RANGE_LIMIT_MAX_SUB2	(uint8_t)(0x02)		// int32 RW		// elmo 软件里设置成了 360

/* PP - Profile Position模式相关的设置对象 (user unit) ****************************************************************************/
#define	OD_PP_TARGET_POSITION			(uint16_t)(0x607A)	//  int32 RW RxMap
#define OD_PP_MAX_PROFILE_VELOCITY		(uint16_t)(0x607F)	// uint32 RW def=2e9
#define	OD_PP_PROFILED_VELOCITY			(uint16_t)(0x6081)	// uint32 RW RxMap
#define	OD_PP_END_VELOCITY				(uint16_t)(0x6082)	// uint32 RW RxMap
#define OD_PP_PROFILED_ACC				(uint16_t)(0x6083)	// uint32 RW RxMap
#define	OD_PP_PROFILED_DEC				(uint16_t)(0x6084)	// uint32 RW RxMap
#define OD_PP_QUICK_STOP_DEC			(uint16_t)(0x6085)	// uint32 RW RxMap
#define OD_PP_MOTION_PROFILE_TYPE		(uint16_t)(0x6086)	//  int16 RW def=0
#define OD_PP_MAX_ACC					(uint16_t)(0x60C5)	//  int32 RW
#define	OD_PP_MAX_DEC					(uint16_t)(0x60C6)	//  int32 RW

/* PV - Profile Velocity 模式相关的设置对象 (user unit) ****************************************************************************/
#define OD_PV_PROFILED_ACC				(uint16_t)(0x6083)	// uint32 RW RxMap
#define	OD_PV_PROFILED_DEC				(uint16_t)(0x6084)	// uint32 RW RxMap
#define OD_PV_QUICK_STOP_DEC			(uint16_t)(0x6085)	// uint32 RW RxMap
#define OD_PV_SENSOR_SELECTION_CODE		(uint16_t)(0x606A)	// int16 RW def=1 606C速度值[0:来自位置编码器， 或 1:来自速度编码器]
#define OD_PV_VELOCITY_DEMAND_VALUE		(uint16_t)(0x606B)	//  int32 RO RxMap def=0
/* 实际的速度值 以用户定义的单位形式 * */
#define OD_PV_VELOCITY_ACTUAL_VALUE		(uint16_t)(0x606C)	//  int32 RO RxMap def no
#define OD_PV_TARGET_VELOCITY			(uint16_t)(0x60FF)	//  int32 RW RxMap

/* PT Profile-Torque 轮廓扭矩模式相关的设置对象 ******************************* */
#define OD_PT_TARGET_TORQUE				(uint16_t)(0x6071)	// int16  RW -32768~32768 0  val = tarTorque[电流] * 1000/ 6075
#define OD_PT_MAX_TORQUE				(uint16_t)(0x6072)	// uint16 RW 1 ~ 50000 def-50000-驱动器所能提供的最大电流50A
#define	OD_PT_MAX_CURRENT				(uint16_t)(0x6073)	// uint16 RW 1 ~ 50000 def-50000-驱动器所能提供的最大电流50A
#define	OD_PT_TORQUE_DEMAND_VALUE		(uint16_t)(0x6074)	// int16  RO -32768~32768 def=0
#define	OD_PT_MOTOR_RATED_CURRENT		(uint16_t)(0x6075)	// uint32 RW 1 ~ M*1000 mA  M是电机的连续运行的最大电流 def = 1
#define	OD_PT_MOTOR_RATED_TORQUE		(uint16_t)(0x6076)	// uint32 RW 1 ~ M*1000 mA  M是电机的连续运行的最大电流 def = 1
#define OD_PT_TORQUE_ACTUAL_VALUE		(uint16_t)(0x6077)	// int16  RO -32768 ~ 32768 def = 0
#define OD_PT_CURRENT_ACTUAL_VALUE		(uint16_t)(0x6078)	// int16  RO -32768 ~ 32768 def = 0
#define OD_PT_TORQUE_SLOPE				(uint16_t)(0x6087)	// uint32 RW 0 ~ 2^32-1 def = 10000000

//************************* ELMO IP模式 支持的都是线性插值 ****************************************************************/
//	[#] 使用同步或异步 PDO，在每个插值周期向驱动器发送目标位置（也可包括目标速度值-两种数据类型）
//	[#] 也允许主机向驱动器发送带有明确时间参考的插值数据流
//	[#] Elmo驱动器支持输入缓冲区，允许以突发方式发送数据
//	[#] 主机可以用插值数据配置对象设置输入缓冲区的实际可用大小和最大数量，这个大小以插值数据条目为单位不是字节单位的大小
//	[#] sub mode select对象定义了插值的模式。默认是 线性插值（仅支持这种）
//	[#] IP 模式基于同步机制 允许主机同步多个轴到相同的设定点时序
//	[#]	数据记录输入缓冲有两种 FIFO 和 Ring
//		[##] FIFO: 新的数据条目放在缓冲区的尾部
//		[##] Ring: 主机可以更改缓冲区指针在任何位置设置点。驱动器从开始地方作为内部读指针
//	[#] IP模式内部状态 <插值非活跃> <插值活跃> 这两个
//	    [**] 驱动器应该处于Operation enabled状态，ControlWord发送命令
//		[##] 0x6060=7 (IP) 选定  --》进入插值非活跃状态
//		[##] 0x6060=0 (clc)     --》退出插值非活跃状态
//		[##] CW.bit4=1			--》进入插值活跃状态 离开插值非活跃状态
//		[##] CW.bit4=0			--》退出插值活跃状态 进入插值非活跃状态

//	  	[**] Interpolation inactive 状态
//			驱动器处于OP_EN状态同时插值位置模式被选定，6061显示当前选定的状态值
//			驱动器接受输入数据并且缓存数据用于插值计算，但是轴不会转动。
//		[**] Interpolation active 状态
//			OP_EN, IP mode, CW.bit4=1 满足所有 插值开始
//			注意，在第一个设定点到达之前，驱动器会插入实际位置。
//			意味着在第一个点到达之前不会有下溢指示。
//			建议，在CW.bit4=1之前，主机将发送许多SYNC消息和数据记录(60C1)，允许驱动器同步和安排设置点缓冲区。
//		[**] Buffer Reset
//			插值数据缓存一下几个请款复位
//			[1]shutdown 电机 [2]操作/运动模式改变 [3]60C4改变 [4]60C0改变 [5]underflow EMCY进入插值非活跃状态
//	[#] 控制字
//		bit4 0->1 插值活跃， bit4 1->0 插值被打断 按照 Halt功能码(605D)停止
//		bit8 0->1 驱动器停止插值，停止电机 按照 Halt功能码(605D)停止
//		fault 或者 CW命令电机停止，即使bit4=1，也会禁用插值。要通过 CW使驱动器进入OP_EN再设置bit4=1，启用插值
//	[#] 状态字
//		bit12 指示着当前 Interpolation 的状态 0：not active 1:active
//	[#] 60C0 子模式选择
//		这个值的改变仅在插值模式非活跃状态下进行，在 Active 状态下企图改变该值驱动器产生SDO终止消息，elmo error = 185
//		改完模式后，必须对60C1做新的映射，否则会导致未知的错误。
//	[#] 超时 和 下溢紧急消息
//		sub mode = 0/-1 actual buffer size = 1， 主机要保证提供数据和时间周期相‘一致’，保证buffer不为空。
//	[#] 上溢紧急消息
//		主机保证新的数据不覆盖buffer里的原有且未使用的数据。ring buffer不执行这个内部监视操作
//	[#] 运动同步
//		IP模式能够同步多个轴运动，这些轴都应该运行在IP模式下，并且同时调用插值轮廓模式运动，使用SYNC时间机制持续运行
//		启动多个轴: CW映射到 同步 RPDO使用映射后的CW使能各个轴的 插值--即PDO方式操作CW
//				 各个轴立即启动了IP，在下一个同步消息之前不会有任何动作
//	[#]	功能描述
//		[*]驱动器执行一个时间同步的路径。在初始时间用户指定一个参考信号值，在之后的固定的时间间隔指定值
//		[*]数据记录时间间隔T用对象 60C2 设置，ms单位，数据记录由对象 60C1 提供与时间相关的运动路径数据
//		[*]主机/用户必须足够快的提供数据记录，平均每个T至少一个数据记录。
//			[1] submode=-1, 驱动器最多存储 16 条记录
//			[2] submode= 0, 驱动器存储     1 条记录
//		   在子模式 -1 中，路径数据记录能以突发方式写入 从而宽松实时性的要求
//		[*]进入IP模式，必须用到 6060对象，设置为 7。要监视运动模式 应该用 6061 对象
//		[*]6040 使能电机、移动电机。 6041 监视状态信息
//		[*]60C0 插值子模式 设定，决定了使用什么样的数据记录，仅有线性插值方式
//	  [#] Sub Mode = 0: 线性插值
//		[*] 只有位置数据记录，计算轮廓速度，用当前的n点位置和前一个n-1点位置只差与插值时间周期之比
//		[*] 驱动器执行两点之间插值是 250us 一次，例：插值时间周期=2ms，这个周期内就有8个插值分段
//			各个阶段的轮廓位置输出-驱动器插补出的位置值：PPO(k)=P(n)+V*250us*k (k=0...7)
//		[*] 最大buffer 和 实际buffer大小都是 1，主机发送SYNC的时间周期与驱动器的 插值时间周期 相等。
//	  [#] Sub Mode = -1: 线性插值
//		[*] 有位置和速度数据记录。在每个插值周期轮廓速度的值由主机给定。
//		[*] 将实际buffer大小设置为1：主机发送SYNC的时间周期与驱动器的 插值时间周期 相等。
//		[*] 将实际buffer大小设置>1：主机不应该每个插值周期发送同步数据，但可以用突发方式发数据，要确保buffer不为空
/* 轮廓加速度 轮廓减速度 快速停止减速度字典对象定义 */
#define	OD_IP_MAX_MOTOR_SPEED			(uint16_t)(0x6080)	// uint32 RW
#define OD_IP_PROFILED_ACC				(uint16_t)(0x6083)	// uint32 RW RxMap
#define	OD_IP_PROFILED_DEC				(uint16_t)(0x6084)	// uint32 RW RxMap
#define OD_IP_QUICK_STOP_DEC			(uint16_t)(0x6085)	// uint32 RW RxMap
/* Interpolation Mode 子模式选择 def=0 *************************************************/
#define OD_IP_SUB_MODE_SELECT			(uint16_t)(0x60C0)	//  int16 RW 0:数据记录仅包括位置数据60C1.1;-1:数据记录包括位置60C1.1和速度数据60C1.2
/* Interpolation Data Record 用位置 或 位置+速度数据作为运动参考 内部线性插值 数据的解释随着60C0设置的不同而变化 */
#define OD_IP_DATA_RECORD				(uint16_t)(0x60C1)	// 有三个子对象
#define    IP_DATA_RECORD_NUM_SUB0 		(uint8_t)(0x00)		// uint8 RO num = 2, 固定值
#define    IP_DATA_RECORD_TAG_POS_SUB1 	(uint8_t)(0x01)	// int32 RW 数据记录里的位置值 四个字节
#define    IP_DATA_RECORD_TAG_VEL_SUB2 	(uint8_t)(0x02)	// int32 RW 数据记录里的速度值 四个字节
/* Interpolation Time Period  两个给定点之间的相对时间 单位:10^sub2s ********************************************/
#define OD_IP_TIME_PERIOD				(uint16_t)(0x60C2)	// 只能在IP非活跃下能改变这个值，有三个子对象
#define	   IP_TIME_PERIOD_NUM_SUB0		(uint8_t)(0x00)	// uint8 RO num = 2, 固定值
#define    IP_TIME_PERIOD_UNITS_SUB1	(uint8_t)(0x01)	// uint8 RW 1~255 , def = 1
#define	   IP_TIME_PERIOD_INDEX_SUB2	(uint8_t)(0x02)	//  int8 RW -6~-2 , def = -3  10^-3s = 1ms--以ms为单位
/* Interpolation sync definition */
#define	OD_IP_SYNC_DEFINITION			(uint16_t)(0x60C3)	//
/* Interpolation Data Config 包括获取buffer大小、设置配置参数和buffer策略 *******************************************************************/
/** 主机使用该对象确定实际缓冲区大小
 * @ sub3可定义两种类型 First In First Out 或 Ring buffer
 * 	[#] FIFO
 * 	[#] Ring buffer
 * @ 建议主控先发送一些SYNC和Points消息(60C1) 之后在设置控制字的bit12=1,使能插值模式,避免underflow error
 * 		如果没有收到新的数据点，按照605D定义停止代码停止，CW.bit12=0, Operating mode 保持7不变
 * @ IP模式启用了多个轴的同步运动。多个丛轴在ip模式下运行，同时调用插值轮廓运动，则他们的运动将同步。同步使用同步机制持续运行。
 * 	    驱动器执行时间同步的运动路径，驱动器内部插值是按照250us来做计算
 * 	[#] 映射CW到同步RPDO
 * 	[#] 使用映射的CW使能所有轴的插值
 * 	[#] 60C2设置同步时间， 如果60C4.sub2=1则主机发送SYNC时间要与这个对象设置的时间‘一致’，>1
 * 	[#] 建议在插值之前发送64个DYNC消息用来同步内部时间
 */
#define	OD_IP_DATA_CONFIG						(uint16_t)(0x60C4)	// 有七个子对象
#define	   IP_DATA_CONFIG_NUM_SUB0				(uint8_t)(0x00)		// uint8 RO num=6, 固定值
					/* 最大容许的条目数量 */
#define    IP_DATA_CONFIG_MAX_BUF_SIZE_SUB1		(uint8_t)(0x01)		// uint32 RO IPmode0:1	IOmode-1:16 def=1 插值数据记录的数量，buffer的最大值
					/* 实际 get/set 条目数量 */
#define	   IP_DATA_CONFIG_ACTUAL_BUF_SIZE_SUB2	(uint8_t)(0x02)		// uint32 RW 1~16 def=1, 插值数据记录实际数量
#define	   IP_DATA_CONFIG_BUF_ORG_SUB3			(uint8_t)(0x03)		// uint8  RW 0:fifo 或 1:ring类型buffer, 0~1 def=0 和60C0模式有关
#define	   IP_DATA_CONFIG_BUF_POS_SUB4			(uint8_t)(0x04)		// uint16 RW 0~15 def=0
					/* 每一个条目所占用的字节数 */
#define	   IP_DATA_CONFIG_DATA_RECORD_SIZE_SUB5	(uint8_t)(0x05)		// uint8  RO 60C0=0:4Bytes 60C0=-1:8Bytes def=4Bytes
#define	   IP_DATA_CONFIG_BUF_CLEAR_SUB6		(uint8_t)(0x06)		// uint8  WO 0~1 def=null 0:清除输入缓冲区；访问禁用；清所有IP数据记录  1:启用对驱动功能输入缓冲区的访问

#define	OD_IP_EXTRAPOLATION_CYCLES_TIMEOUT		(uint16_t)(0x2F75)	// 驱动器停止之前，执行外推运算的数量 def = 1 1_32767


/* ***************************** PDO 相关的 **************************** */
/* RPDOx 通信参数部分 ****************************************************************************************/
#define	OD_RPDO1_COMP					(uint16_t)(0x1400)
#define	OD_RPDO2_COMP					(uint16_t)(0x1401)
#define	OD_RPDO3_COMP					(uint16_t)(0x1402)
#define	OD_RPDO4_COMP					(uint16_t)(0x1403)
#define	   RPDO_COMP_NUM_SUB0 			(uint8_t)(0x00)		// uint8  RO num = 2,固定值
#define	   RPDO_COMP_COBID_PDO_SUB1 	(uint8_t)(0x01)		// uint32 RW def:0x27F,0x37F,0x47F,0x57F
#define	   RPDO_COMP_TRANSMIT_TYPE_SUB2	(uint8_t)(0x02)		// uint8  RW def:255 [1(1个SYNC),254(事件驱动),255(事件驱动)]

/* RPDOx 映射参数部分 ****************************************************************************************/
	//	映射数据格式说明 LSB| ObjectDataLength 8bit | ObjectSubInddex 8bit | ObjectIndex 16bit |MSB
//1.最多可以将 8 个对象映射到单个 RPDO
//2.Elmo驱动器支持Dummy条目
//3.对象可以在 PRE Operational 和 Operational state期间被映射
//4.SDO 客户端负责数据一致性
#define OD_RPDO1_MAPPING 				(uint16_t)(0x1600)
#define OD_RPDO2_MAPPING 				(uint16_t)(0x1601)
#define OD_RPDO3_MAPPING 				(uint16_t)(0x1602)
#define OD_RPDO4_MAPPING 				(uint16_t)(0x1603)
#define    RPDO_MAPPING_NUM_SUB0		(uint8_t)(0x00)		// uint8  RW num = 0~8
#define    RPDO_MAPPING_PDO_SUB1		(uint8_t)(0x01)		// uint32 RW def:60400010,控制字映射; def:20120040,二进制命令映射
#define    RPDO_MAPPING_PDO_SUB2		(uint8_t)(0x02)		// uint32 RW
#define    RPDO_MAPPING_PDO_SUB3		(uint8_t)(0x03)		// uint32 RW
#define    RPDO_MAPPING_PDO_SUB4		(uint8_t)(0x04)		// uint32 RW
#define    RPDO_MAPPING_PDO_SUB5		(uint8_t)(0x05)		// uint32 RW
#define    RPDO_MAPPING_PDO_SUB6		(uint8_t)(0x06)		// uint32 RW
#define    RPDO_MAPPING_PDO_SUB7		(uint8_t)(0x07)		// uint32 RW
#define    RPDO_MAPPING_PDO_SUB8		(uint8_t)(0x08)		// uint32 RW

/* TPDOx 通信参数部分 ****************************************************************************************/
#define OD_TPDO1_COMP					(uint16_t)(0x1800)
#define OD_TPDO2_COMP					(uint16_t)(0x1801)
#define OD_TPDO3_COMP					(uint16_t)(0x1802)
#define OD_TPDO4_COMP					(uint16_t)(0x1803)
#define    TPDO_COMP_NUM_SUB0			(uint8_t)(0x00)		// uint8  RO num = 5,固定值
#define    TPDO_COMP_COBID_PDO_SUB1		(uint8_t)(0x01)		// uint32 RW def:0x400001FF,0x400002FF,0x400003FF,0x400004FF
#define	   TPDO_COMP_TRANSMIT_TYPE_SUB2	(uint8_t)(0x02)		// uint8  RW def:255,254,0,0 [0-240(n个SYNC),254(事件驱动),255(事件驱动)]
#define	   TPDO_COMP_INHIBIT_TIME_SUB3	(uint8_t)(0x03)		// uint16 RW def:0(no indibit time between message) [0,65535]
#define	   TPDO_COMP_RESERVE_SUB4		(uint8_t)(0x04)		// uint8 reserve
#define	   TPDO_COMP_EVENT_TIMER_SUB5	(uint8_t)(0x05)		// uint16 RW def:0 [0,65535]

/* TPDOx 映射参数部分 ****************************************************************************************/
//1.最多可以将 8 个对象映射到单个 TPDO
//2.Elmo驱动器支持Dummy条目
//3.对象可以在 PRE Operational 和 Operational state期间被映射
//4.SDO 客户端负责数据一致性
#define OD_TPDO1_MAPPING 				(uint16_t)(0x1A00)
#define OD_TPDO2_MAPPING 				(uint16_t)(0x1A01)
#define OD_TPDO3_MAPPING 				(uint16_t)(0x1A02)
#define OD_TPDO4_MAPPING 				(uint16_t)(0x1A03)
#define    TPDO_MAPPING_NUM_SUB0		(uint8_t)(0x00)		// uint8  RW num = 0~8
#define    TPDO_MAPPING_PDO_SUB1		(uint8_t)(0x01)		// uint32 RW def:60410010, 状态字映射; def:20140040, 二进制命令
#define    TPDO_MAPPING_PDO_SUB2		(uint8_t)(0x02)		// uint32 RW
#define    TPDO_MAPPING_PDO_SUB3		(uint8_t)(0x03)		// uint32 RW
#define    TPDO_MAPPING_PDO_SUB4		(uint8_t)(0x04)		// uint32 RW
#define    TPDO_MAPPING_PDO_SUB5		(uint8_t)(0x05)		// uint32 RW
#define    TPDO_MAPPING_PDO_SUB6		(uint8_t)(0x06)		// uint32 RW
#define    TPDO_MAPPING_PDO_SUB7		(uint8_t)(0x07)		// uint32 RW
#define    TPDO_MAPPING_PDO_SUB8		(uint8_t)(0x08)		// uint32 RW


/* extern 声明部分 ****************************************************************************************/
/* 驱动器 设备类型字典对象定义 ***************************** */
extern const ODOBJTypeDef OBJ_Device_Type;

/* 制造生设备名字字典对象定义 String *********************** */
extern const ODOBJTypeDef OBJ_Manufacturer_Device_Name;

/* 制造商硬件版本字典对象定义 String ********************** */
extern const ODOBJTypeDef OBJ_Manufacturer_Hard_Version;

/* 制造商软件版本字典对象定义 String ********************** */
extern const ODOBJTypeDef OBJ_Manufacturer_Soft_Version;

/* 制造商DSP版本字典对象定义 String ********************** */
extern const ODOBJTypeDef OBJ_Manufacturer_DSP_Version;

/* 驱动器模拟量输入端口 检测模拟量 使用sub1的结果计算电机的线圈温度 * */
extern const ODOBJTypeDef OBJ_Analog_mV;

/* 驱动器 5V DC 电压值 mV **************************** */
extern const ODOBJTypeDef OBJ_5V_DC_Supply;

/* 驱动器里电机位置值 (用户定义的单位) ********************** */
extern const ODOBJTypeDef OBJ_Actual_Position_User;

/* 驱动器里电机位置控制命令值 demmand value */
extern const ODOBJTypeDef OBJ_Demand_Position_User;

/* 驱动器位置跟踪误差 (用户定义的单位) ************************/
extern const ODOBJTypeDef OBJ_Position_Following_Error_User;

/* 驱动器位置环 控制器输出的控制量 (用户单位) *******************/
extern const ODOBJTypeDef OBJ_Position_Loop_Control_Effort;

/* 驱动器里电机速度值 (用户定义的单位) ********************** */
extern const ODOBJTypeDef OBJ_Actual_Velocity_User;

/* 驱动器直流回路电流 48VDC 回路 电流 ********************* */
extern const ODOBJTypeDef OBJ_Current_Actual_Value;

/* 驱动器直流回路电压 48VDC 实际值 *********************** */
extern const ODOBJTypeDef OBJ_48V_DC_Voltage;

/* 驱动器温度 摄氏度 *********************************** */
extern const ODOBJTypeDef OBJ_Drive_Temperature;

/* 数字量输出字典对象定义 ******************************** */
extern const ODOBJTypeDef OBJ_Digital_Output;

/* 控制字字典对象定义 *********************************** */
extern const ODOBJTypeDef OBJ_Control_Word ;

/* 状态字字典对象定义 *********************************** */
extern const ODOBJTypeDef OBJ_Status_Word;

/* Modes of Operation 运动模式设置和获取对象 ************* */
extern const ODOBJTypeDef OBJ_Operation_Modes;
extern const ODOBJTypeDef OBJ_Operation_Modes_Display;

/* Quick stop option code */
extern const ODOBJTypeDef OBJ_Quick_Stop_Option_Code;

/* Positioning Option Code */
extern const ODOBJTypeDef OBJ_Positioning_Option_Code;

/* Profile Position 运动模式相关字典对象定 ******************/
extern const ODOBJTypeDef OBJ_PP_Target_Pos		    ;
extern const ODOBJTypeDef OBJ_PP_Profile_Velocity	;
extern const ODOBJTypeDef OBJ_PP_End_Velocity		;
extern const ODOBJTypeDef OBJ_PP_Profile_Acc		;
extern const ODOBJTypeDef OBJ_PP_Profile_Dec		;
extern const ODOBJTypeDef OBJ_PP_QUICK_STOP_Dec	    ;

/* Profile Velocity 运动模式相关字典对象定义 */
extern const ODOBJTypeDef OBJ_PV_Target_Velocity	;
extern const ODOBJTypeDef OBJ_PV_Profile_Acc		;
extern const ODOBJTypeDef OBJ_PV_Profile_Dec		;
extern const ODOBJTypeDef OBJ_PV_QUICK_STOP_Dec	;

/* Profile Torque 运动模式相关字典对象定义 */
extern const ODOBJTypeDef OBJ_PT_Target_Torque				;
extern const ODOBJTypeDef OBJ_PT_Motor_Rated_Current		;

/* Interpolation 运动模式相关的字典对象定义***************** */
//IP的轮廓加速度 轮廓减速度 快速停止减速度
extern const ODOBJTypeDef OBJ_IP_Max_Motor_Speed			 ;
extern const ODOBJTypeDef OBJ_IP_Profile_Acc				 ;
extern const ODOBJTypeDef OBJ_IP_Profile_Dec				 ;
extern const ODOBJTypeDef OBJ_IP_QUICK_STOP_Dec				 ;
extern const ODOBJTypeDef OBJ_IP_Sub_Mode_Select	 		 ;
extern const ODOBJTypeDef OBJ_IP_Data_Record_Tag_Pos 		 ;
extern const ODOBJTypeDef OBJ_IP_Data_Record_Tag_Vel 		 ;
extern const ODOBJTypeDef OBJ_IP_Time_Record_Period_Units 	 ;
extern const ODOBJTypeDef OBJ_IP_Time_Record_Period_Index 	 ;
extern const ODOBJTypeDef OBJ_IP_Data_Config_Max_Buf_Size 	 ;
extern const ODOBJTypeDef OBJ_IP_Data_Config_Actual_Buf_Size ;
extern const ODOBJTypeDef OBJ_IP_Data_Config_Buf_Org 		 ;
extern const ODOBJTypeDef OBJ_IP_Data_Config_Buf_Pos		 ;
extern const ODOBJTypeDef OBJ_IP_Data_Config_Data_Record_Size;
extern const ODOBJTypeDef OBJ_IP_Data_Config_Buf_Clear		 ;

// 与IP相关的对象 -- 设置运动停止前要执行几个外推的周期个数，完后产生EMCY消息
extern const ODOBJTypeDef OBJ_Extrapolation_Cycles_Timeout	 ;


/* rpdo 通信参数字典对象定义 **************************** */
					/* 1400~1403 SUB0 */
extern const ODOBJTypeDef  OBJ_RPDO1_COMP_NUM_SUB0			;
extern const ODOBJTypeDef  OBJ_RPDO2_COMP_NUM_SUB0			;
extern const ODOBJTypeDef  OBJ_RPDO3_COMP_NUM_SUB0			;
extern const ODOBJTypeDef  OBJ_RPDO4_COMP_NUM_SUB0			;
					/* 1400~1403 SUB */
extern const ODOBJTypeDef	OBJ_RPDO1_COMP_COBID_Sub1 		;
extern const ODOBJTypeDef	OBJ_RPDO2_COMP_COBID_Sub1 		;
extern const ODOBJTypeDef	OBJ_RPDO3_COMP_COBID_Sub1 		;
extern const ODOBJTypeDef	OBJ_RPDO4_COMP_COBID_Sub1 		;
					/* 1400~1403 SUB */
extern const ODOBJTypeDef	OBJ_RPDO1_COMP_TRANS_TYPE_Sub2 	;
extern const ODOBJTypeDef	OBJ_RPDO2_COMP_TRANS_TYPE_Sub2 	;
extern const ODOBJTypeDef	OBJ_RPDO3_COMP_TRANS_TYPE_Sub2 	;
extern const ODOBJTypeDef	OBJ_RPDO4_COMP_TRANS_TYPE_Sub2 	;
                                                    ;
/* tpdo 通信参数字典对象定义 **************************** */
                	/* 1800~1803 SUB0 */
extern const ODOBJTypeDef	obj_TPDO1_COMP_NUM_SUB0			;
extern const ODOBJTypeDef	obj_TPDO2_COMP_NUM_SUB0			;
extern const ODOBJTypeDef	obj_TPDO3_COMP_NUM_SUB0			;
extern const ODOBJTypeDef	obj_TPDO4_COMP_NUM_SUB0			;
					/* 1800~1803 SUB */
extern const ODOBJTypeDef	OBJ_TPDO1_COMP_COBID_Sub1		;
extern const ODOBJTypeDef	OBJ_TPDO2_COMP_COBID_Sub1		;
extern const ODOBJTypeDef	OBJ_TPDO3_COMP_COBID_Sub1		;
extern const ODOBJTypeDef	OBJ_TPDO4_COMP_COBID_Sub1		;
					/* 1800~1803 SUB */
extern const ODOBJTypeDef	OBJ_TPDO1_COMP_TRANS_TYPE_Sub2	;
extern const ODOBJTypeDef	OBJ_TPDO2_COMP_TRANS_TYPE_Sub2	;
extern const ODOBJTypeDef	OBJ_TPDO3_COMP_TRANS_TYPE_Sub2	;
extern const ODOBJTypeDef	OBJ_TPDO4_COMP_TRANS_TYPE_Sub2	;
					/* 1800~1803 SUB */
extern const ODOBJTypeDef	OBJ_TPDO1_COMP_INHIBIT_TIME_Sub3;
extern const ODOBJTypeDef	OBJ_TPDO2_COMP_INHIBIT_TIME_Sub3;
extern const ODOBJTypeDef	OBJ_TPDO3_COMP_INHIBIT_TIME_Sub3;
extern const ODOBJTypeDef	OBJ_TPDO4_COMP_INHIBIT_TIME_Sub3;
					/* 1800~1803 SUB */
extern const ODOBJTypeDef	OBJ_TPDO1_COMP_EVENT_TIMER_Sub5	;
extern const ODOBJTypeDef	OBJ_TPDO2_COMP_EVENT_TIMER_Sub5	;
extern const ODOBJTypeDef	OBJ_TPDO3_COMP_EVENT_TIMER_Sub5	;
extern const ODOBJTypeDef	OBJ_TPDO4_COMP_EVENT_TIMER_Sub5	;
                                                    ;
/* rpdo 映射参数字典对象定义 **************************** */
					/* 1600~1603 SUB */
extern const ODOBJTypeDef	OBJ_RPDO1_MAPPING_NUM_Sub0		;
extern const ODOBJTypeDef	OBJ_RPDO2_MAPPING_NUM_Sub0		;
extern const ODOBJTypeDef	OBJ_RPDO3_MAPPING_NUM_Sub0		;
extern const ODOBJTypeDef	OBJ_RPDO4_MAPPING_NUM_Sub0		;
					/* 1800 SUB1~8  */
extern const ODOBJTypeDef	OBJ_RPDO1_MAPPING_PDO_Sub1		;
extern const ODOBJTypeDef	OBJ_RPDO1_MAPPING_PDO_Sub2		;
extern const ODOBJTypeDef	OBJ_RPDO1_MAPPING_PDO_Sub3		;
extern const ODOBJTypeDef	OBJ_RPDO1_MAPPING_PDO_Sub4		;
extern const ODOBJTypeDef	OBJ_RPDO1_MAPPING_PDO_Sub5		;
extern const ODOBJTypeDef	OBJ_RPDO1_MAPPING_PDO_Sub6		;
extern const ODOBJTypeDef	OBJ_RPDO1_MAPPING_PDO_Sub7		;
extern const ODOBJTypeDef	OBJ_RPDO1_MAPPING_PDO_Sub8		;
					/* 1801 SUB1~8  */
extern const ODOBJTypeDef	OBJ_RPDO2_MAPPING_PDO_Sub1		;
extern const ODOBJTypeDef	OBJ_RPDO2_MAPPING_PDO_Sub2		;
extern const ODOBJTypeDef	OBJ_RPDO2_MAPPING_PDO_Sub3		;
extern const ODOBJTypeDef	OBJ_RPDO2_MAPPING_PDO_Sub4		;
extern const ODOBJTypeDef	OBJ_RPDO2_MAPPING_PDO_Sub5		;
extern const ODOBJTypeDef	OBJ_RPDO2_MAPPING_PDO_Sub6		;
extern const ODOBJTypeDef	OBJ_RPDO2_MAPPING_PDO_Sub7		;
extern const ODOBJTypeDef	OBJ_RPDO2_MAPPING_PDO_Sub8		;
					/* 1802 SUB1~8  */
extern const ODOBJTypeDef	OBJ_RPDO3_MAPPING_PDO_Sub1		;
extern const ODOBJTypeDef	OBJ_RPDO3_MAPPING_PDO_Sub2		;
extern const ODOBJTypeDef	OBJ_RPDO3_MAPPING_PDO_Sub3		;
extern const ODOBJTypeDef	OBJ_RPDO3_MAPPING_PDO_Sub4		;
extern const ODOBJTypeDef	OBJ_RPDO3_MAPPING_PDO_Sub5		;
extern const ODOBJTypeDef	OBJ_RPDO3_MAPPING_PDO_Sub6		;
extern const ODOBJTypeDef	OBJ_RPDO3_MAPPING_PDO_Sub7		;
extern const ODOBJTypeDef	OBJ_RPDO3_MAPPING_PDO_Sub8		;
					/* 1803 SUB1~8  */
extern const ODOBJTypeDef	OBJ_RPDO4_MAPPING_PDO_Sub1		;
extern const ODOBJTypeDef	OBJ_RPDO4_MAPPING_PDO_Sub2		;
extern const ODOBJTypeDef	OBJ_RPDO4_MAPPING_PDO_Sub3		;
extern const ODOBJTypeDef	OBJ_RPDO4_MAPPING_PDO_Sub4		;
extern const ODOBJTypeDef	OBJ_RPDO4_MAPPING_PDO_Sub5		;
extern const ODOBJTypeDef	OBJ_RPDO4_MAPPING_PDO_Sub6		;
extern const ODOBJTypeDef	OBJ_RPDO4_MAPPING_PDO_Sub7		;
extern const ODOBJTypeDef	OBJ_RPDO4_MAPPING_PDO_Sub8		;
                                                    ;
/* Tpdo 映射参数字典对象定义 **************************** */
					/* 1A00~1A03 SUB */
extern const ODOBJTypeDef	OBJ_TPDO1_MAPPING_NUM_Sub0		;
extern const ODOBJTypeDef	OBJ_TPDO2_MAPPING_NUM_Sub0		;
extern const ODOBJTypeDef	OBJ_TPDO3_MAPPING_NUM_Sub0		;
extern const ODOBJTypeDef	OBJ_TPDO4_MAPPING_NUM_Sub0		;
					/* 1A00 SUB1~8  */
extern const ODOBJTypeDef	OBJ_TPDO1_MAPPING_PDO_Sub1		;
extern const ODOBJTypeDef	OBJ_TPDO1_MAPPING_PDO_Sub2		;
extern const ODOBJTypeDef	OBJ_TPDO1_MAPPING_PDO_Sub3		;
extern const ODOBJTypeDef	OBJ_TPDO1_MAPPING_PDO_Sub4		;
extern const ODOBJTypeDef	OBJ_TPDO1_MAPPING_PDO_Sub5		;
extern const ODOBJTypeDef	OBJ_TPDO1_MAPPING_PDO_Sub6		;
extern const ODOBJTypeDef	OBJ_TPDO1_MAPPING_PDO_Sub7		;
extern const ODOBJTypeDef	OBJ_TPDO1_MAPPING_PDO_Sub8		;
					/* 1A01 SUB1~8  */
extern const ODOBJTypeDef	OBJ_TPDO2_MAPPING_PDO_Sub1		;
extern const ODOBJTypeDef	OBJ_TPDO2_MAPPING_PDO_Sub2		;
extern const ODOBJTypeDef	OBJ_TPDO2_MAPPING_PDO_Sub3		;
extern const ODOBJTypeDef	OBJ_TPDO2_MAPPING_PDO_Sub4		;
extern const ODOBJTypeDef	OBJ_TPDO2_MAPPING_PDO_Sub5		;
extern const ODOBJTypeDef	OBJ_TPDO2_MAPPING_PDO_Sub6		;
extern const ODOBJTypeDef	OBJ_TPDO2_MAPPING_PDO_Sub7		;
extern const ODOBJTypeDef	OBJ_TPDO2_MAPPING_PDO_Sub8		;
					/* 1A02 SUB1~8 */
extern const ODOBJTypeDef	OBJ_TPDO3_MAPPING_PDO_Sub1		;
extern const ODOBJTypeDef	OBJ_TPDO3_MAPPING_PDO_Sub2		;
extern const ODOBJTypeDef	OBJ_TPDO3_MAPPING_PDO_Sub3		;
extern const ODOBJTypeDef	OBJ_TPDO3_MAPPING_PDO_Sub4		;
extern const ODOBJTypeDef	OBJ_TPDO3_MAPPING_PDO_Sub5		;
extern const ODOBJTypeDef	OBJ_TPDO3_MAPPING_PDO_Sub6		;
extern const ODOBJTypeDef	OBJ_TPDO3_MAPPING_PDO_Sub7		;
extern const ODOBJTypeDef	OBJ_TPDO3_MAPPING_PDO_Sub8		;
					/* 1A03 SUB1~8 */
extern const ODOBJTypeDef	OBJ_TPDO4_MAPPING_PDO_Sub1		;
extern const ODOBJTypeDef	OBJ_TPDO4_MAPPING_PDO_Sub2		;
extern const ODOBJTypeDef	OBJ_TPDO4_MAPPING_PDO_Sub3		;
extern const ODOBJTypeDef	OBJ_TPDO4_MAPPING_PDO_Sub4		;
extern const ODOBJTypeDef	OBJ_TPDO4_MAPPING_PDO_Sub5		;
extern const ODOBJTypeDef	OBJ_TPDO4_MAPPING_PDO_Sub6		;
extern const ODOBJTypeDef	OBJ_TPDO4_MAPPING_PDO_Sub7		;
extern const ODOBJTypeDef	OBJ_TPDO4_MAPPING_PDO_Sub8		;





#endif /* CANOPEN_INC_OD_H_ */

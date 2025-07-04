/*
 * OD.c
 *
 *  Created on: 2021年10月14日
 *      Author: Maxiufeng
 */

#include "OD.h"
/* 驱动器 设备类型字典对象定义 **************************************************************************************************************/
const ODOBJTypeDef OBJ_Device_Type	= {OD_DEVICE_TYPE, 0x00, OBJ_READ_ONLY, OBJ_DATA_NUM_4};

/* 制造生设备名字字典对象定义 String ********************************************************************************************************/
const ODOBJTypeDef OBJ_Manufacturer_Device_Name = {OD_MANUFACTURER_DEVICE_NAME, 0x00, OBJ_READ_ONLY, OBJ_DATA_NUM_4};

/* 制造商硬件版本字典对象定义 String ********************************************************************************************************/
const ODOBJTypeDef OBJ_Manufacturer_Hard_Version = {OD_MANUFACTURER_HARD_VERSION, 0x00,OBJ_READ_ONLY,OBJ_DATA_NUM_4};

/* 制造商软件版本字典对象定义 String ********************************************************************************************************/
const ODOBJTypeDef OBJ_Manufacturer_Soft_Version = {OD_MANUFACTURER_SOFT_VERSION,0x00, OBJ_READ_ONLY,OBJ_DATA_NUM_4};

/* 制造商DSP版本字典对象定义 String ********************************************************************************************************/
const ODOBJTypeDef OBJ_Manufacturer_DSP_Version	 = {OD_MANUFACTURER_DSP_VERSION, 0x00, OBJ_READ_ONLY,OBJ_DATA_NUM_4};

/* 驱动器模拟量输入端口 检测模拟量 使用sub1的结果计算电机的线圈温度 ************************************************************************************/
const ODOBJTypeDef OBJ_Analog_mV	= {OD_ANALOG_INPUT, ANALOG_INPUT_MV_SUB1, OBJ_READ_ONLY, OBJ_DATA_NUM_2};

/* 驱动器 5V DC 电压值 mV ***************************************************************************************************************/
const ODOBJTypeDef OBJ_5V_DC_Supply		= {OD_5V_DC_SUPPLY, 0x00, OBJ_READ_ONLY, OBJ_DATA_NUM_2};

/* 驱动器里电机位置控制命令值 demmand value */
const ODOBJTypeDef OBJ_Demand_Position_User = {OD_PC_POSITION_DEMAND_VALUE, 0x00, OBJ_READ_ONLY, OBJ_DATA_NUM_4};

/* 驱动器里电机位置值 (用户定义的单位) *********************************************************************************************************/
const ODOBJTypeDef OBJ_Actual_Position_User	= {OD_PC_POSITION_ACTUAL_VALUE, 0x00, OBJ_READ_ONLY, OBJ_DATA_NUM_4};

/* 驱动器位置跟踪误差 (用户定义的单位) *********************************************************************************************************/
const ODOBJTypeDef OBJ_Position_Following_Error_User = {OD_PC_FOLLOWING_ERROR_ACTUAL_VALUE, 0x00, OBJ_READ_ONLY, OBJ_DATA_NUM_4};

/* 驱动器位置环 控制器输出的控制量 (用户单位) */
const ODOBJTypeDef OBJ_Position_Loop_Control_Effort  = {OD_PC_CONTROL_EFFORT, 0x00, OBJ_READ_ONLY, OBJ_DATA_NUM_4};

/* 驱动器里电机速度值 (用户定义的单位) */
const ODOBJTypeDef OBJ_Actual_Velocity_User	= {OD_PV_VELOCITY_ACTUAL_VALUE, 0x00, OBJ_READ_ONLY, OBJ_DATA_NUM_4};

/* 驱动器直流回路电流 48VDC 回路 电流 *******************************************************************************************************/
const ODOBJTypeDef OBJ_Current_Actual_Value		= {OD_CURRENT_ACTUAL_VALUE, 0x00, OBJ_READ_ONLY, OBJ_DATA_NUM_2};

/* 驱动器直流回路电压 48VDC 实际值 *********************************************************************************************************/
const ODOBJTypeDef OBJ_48V_DC_Voltage			= {OD_48V_DC_VOLTAGE, 0x00, OBJ_READ_ONLY, OBJ_DATA_NUM_4};

/* 驱动器温度 摄氏度单位 ******************************************************************************************************************/
const ODOBJTypeDef OBJ_Drive_Temperature= {OD_DRIVE_TEMPERATURE, 0x00, OBJ_READ_ONLY,OBJ_DATA_NUM_2};

/* 数字量输出字典对象定义 *****************************************************************************************************************/
const ODOBJTypeDef OBJ_Digital_Output	= {OD_DIGITAL_OUTPUT, 0x00, OBJ_READ_WRITE, OBJ_DATA_NUM_1};

/* 控制字字典对象定义 ********************************************************************************************************************/
const ODOBJTypeDef OBJ_Control_Word 	= {OD_CONTROL_WORD, 0x00, OBJ_READ_WRITE, OBJ_DATA_NUM_2};

/* 状态字字典对象定义 ********************************************************************************************************************/
const ODOBJTypeDef OBJ_Status_Word		= {OD_STATUS_WORD,	0x00, OBJ_READ_ONLY,  OBJ_DATA_NUM_2};

/* Modes of operation exp:PP、PV、TQ、IP 等 ***************************************************************************************** */
const ODOBJTypeDef OBJ_Operation_Modes			= {OD_OPERATION_MODES,         0x00, OBJ_READ_WRITE, OBJ_DATA_NUM_1};	// set Mode
const ODOBJTypeDef OBJ_Operation_Modes_Display	= {OD_OPERATION_MODES_DISPLAY, 0x00, OBJ_READ_ONLY,  OBJ_DATA_NUM_1};	// get Current Mode

/* Quick stop option code */
const ODOBJTypeDef OBJ_Quick_Stop_Option_Code	= {OD_QUICK_STOP_OPTION_CODE, 0x00, OBJ_READ_WRITE, OBJ_DATA_NUM_2};	//设置快速度停止的方式

/* Positioning Option Code */
const ODOBJTypeDef OBJ_Positioning_Option_Code	= {OD_PC_POSITIONING_OPTION_CODE, 0x00, OBJ_READ_WRITE, OBJ_DATA_NUM_2}; //set 定位选择码 rado rro cio ..

/* Profile Position 运动模式相关字典对象定义 ********************************************************************************************* */
const ODOBJTypeDef OBJ_PP_Target_Pos		= {OD_PP_TARGET_POSITION,   0x00, OBJ_READ_WRITE, OBJ_DATA_NUM_4};	// 目标位置
const ODOBJTypeDef OBJ_PP_Profile_Velocity	= {OD_PP_PROFILED_VELOCITY, 0x00, OBJ_READ_WRITE, OBJ_DATA_NUM_4};	// 轮廓速度
const ODOBJTypeDef OBJ_PP_End_Velocity		= {OD_PP_END_VELOCITY,      0x00, OBJ_READ_WRITE, OBJ_DATA_NUM_4};	// 结束时速度-到达目标位置时的速度
const ODOBJTypeDef OBJ_PP_Profile_Acc		= {OD_PP_PROFILED_ACC, 	    0x00, OBJ_READ_WRITE, OBJ_DATA_NUM_4};	// 到达轮廓速度过程加速度
const ODOBJTypeDef OBJ_PP_Profile_Dec		= {OD_PP_PROFILED_DEC, 	    0x00, OBJ_READ_WRITE, OBJ_DATA_NUM_4};	// 从轮廓速度减速过程减速度
const ODOBJTypeDef OBJ_PP_QUICK_STOP_Dec	= {OD_PP_QUICK_STOP_DEC,    0x00, OBJ_READ_WRITE, OBJ_DATA_NUM_4};	// 快速停止-减速到0过程减速度

/* Profile Velocity 运动模式相关字典对象定义 */
const ODOBJTypeDef OBJ_PV_Target_Velocity	= {OD_PV_TARGET_VELOCITY,   0x00, OBJ_READ_WRITE, OBJ_DATA_NUM_4};	// 目标速度-转速
const ODOBJTypeDef OBJ_PV_Profile_Acc		= {OD_PV_PROFILED_ACC, 	    0x00, OBJ_READ_WRITE, OBJ_DATA_NUM_4};	// 到达轮廓速度过程加速度
const ODOBJTypeDef OBJ_PV_Profile_Dec		= {OD_PV_PROFILED_DEC, 	    0x00, OBJ_READ_WRITE, OBJ_DATA_NUM_4};	// 从轮廓速度减速过程减速度
const ODOBJTypeDef OBJ_PV_QUICK_STOP_Dec	= {OD_PV_QUICK_STOP_DEC,    0x00, OBJ_READ_WRITE, OBJ_DATA_NUM_4};	// 快速停止-减速到0过程减速度

/* Profile Torque 运动模式相关字典对象定义 */
const ODOBJTypeDef OBJ_PT_Target_Torque			= {OD_PT_TARGET_TORQUE, 0x00, OBJ_READ_WRITE, OBJ_DATA_NUM_2};	// 目标扭矩-电流 需要做转换 不是直接设置电流值
const ODOBJTypeDef OBJ_PT_Motor_Rated_Current	= {OD_PT_MOTOR_RATED_CURRENT, 0x00, OBJ_READ_WRITE, OBJ_DATA_NUM_4};	// 电机的连续运行电流值 mA

/* Interpolation 运动模式相关的字典对象定义 ************************************************************************************************ */
//IP的轮廓加速度 轮廓减速度 快速停止减速度
const ODOBJTypeDef OBJ_IP_Max_Motor_Speed	= {OD_IP_MAX_MOTOR_SPEED,   0x00, OBJ_READ_WRITE, OBJ_DATA_NUM_4};
const ODOBJTypeDef OBJ_IP_Profile_Acc		= {OD_IP_PROFILED_ACC, 	    0x00, OBJ_READ_WRITE, OBJ_DATA_NUM_4};	// 到达轮廓速度过程加速度
const ODOBJTypeDef OBJ_IP_Profile_Dec		= {OD_IP_PROFILED_DEC, 	    0x00, OBJ_READ_WRITE, OBJ_DATA_NUM_4};	// 从轮廓速度减速过程减速度
const ODOBJTypeDef OBJ_IP_QUICK_STOP_Dec	= {OD_IP_QUICK_STOP_DEC,    0x00, OBJ_READ_WRITE, OBJ_DATA_NUM_4};	// 快速停止-减速到0过程减速度
//IP子模式选择0/-1
const ODOBJTypeDef OBJ_IP_Sub_Mode_Select	  			= {OD_IP_SUB_MODE_SELECT, 0x00,                    			OBJ_READ_WRITE, OBJ_DATA_NUM_2};
//IP数据记录 -- 位置数据记录对象
const ODOBJTypeDef OBJ_IP_Data_Record_Tag_Pos 			= {OD_IP_DATA_RECORD, IP_DATA_RECORD_TAG_POS_SUB1, 			OBJ_READ_WRITE, OBJ_DATA_NUM_4};
//IP数据记录 -- 速度数据记录对象
const ODOBJTypeDef OBJ_IP_Data_Record_Tag_Vel 			= {OD_IP_DATA_RECORD, IP_DATA_RECORD_TAG_VEL_SUB2, 			OBJ_READ_WRITE, OBJ_DATA_NUM_4};
//IP时间周期 --
const ODOBJTypeDef OBJ_IP_Time_Record_Period_Units 		= {OD_IP_TIME_PERIOD, IP_TIME_PERIOD_UNITS_SUB1, 			OBJ_READ_WRITE, OBJ_DATA_NUM_1};
//IP时间周期 -- 索引、上标值 10^-index ms
const ODOBJTypeDef OBJ_IP_Time_Record_Period_Index 		= {OD_IP_TIME_PERIOD, IP_TIME_PERIOD_INDEX_SUB2, 			OBJ_READ_WRITE, OBJ_DATA_NUM_1};
//IP数据配置 -- 数据记录buffer的最大值
const ODOBJTypeDef OBJ_IP_Data_Config_Max_Buf_Size 		= {OD_IP_DATA_CONFIG, IP_DATA_CONFIG_MAX_BUF_SIZE_SUB1, 	OBJ_READ_ONLY, 	OBJ_DATA_NUM_4};
//IP数据配置 -- 数据记录buffer的实际大小
const ODOBJTypeDef OBJ_IP_Data_Config_Actual_Buf_Size 	= {OD_IP_DATA_CONFIG, IP_DATA_CONFIG_ACTUAL_BUF_SIZE_SUB2,	OBJ_READ_WRITE,	OBJ_DATA_NUM_4};
//IP数据配置 -- 数据记录buffer的组织形式 FIFO、Ring
const ODOBJTypeDef OBJ_IP_Data_Config_Buf_Org 			= {OD_IP_DATA_CONFIG, IP_DATA_CONFIG_BUF_ORG_SUB3,			OBJ_READ_WRITE, OBJ_DATA_NUM_1};
//IP数据配置 -- 数据记录buffer的指针指定位置
const ODOBJTypeDef OBJ_IP_Data_Config_Buf_Pos			= {OD_IP_DATA_CONFIG, IP_DATA_CONFIG_BUF_POS_SUB4,			OBJ_READ_WRITE, OBJ_DATA_NUM_2};
//IP数据配置 -- IP数据记录单条占用字节数量
const ODOBJTypeDef OBJ_IP_Data_Config_Data_Record_Size	= {OD_IP_DATA_CONFIG, IP_DATA_CONFIG_DATA_RECORD_SIZE_SUB5, OBJ_READ_ONLY, 	OBJ_DATA_NUM_1};
//IP数据配置 -- buffer清空、禁止/使能操作
const ODOBJTypeDef OBJ_IP_Data_Config_Buf_Clear			= {OD_IP_DATA_CONFIG, IP_DATA_CONFIG_BUF_CLEAR_SUB6, 		OBJ_WRITE_ONLY, OBJ_DATA_NUM_1};

// 与IP相关的对象 -- 设置运动停止前要执行几个外推的周期个数，完后产生EMCY消息
const ODOBJTypeDef OBJ_Extrapolation_Cycles_Timeout		= {OD_IP_EXTRAPOLATION_CYCLES_TIMEOUT, 0x00, 				OBJ_READ_WRITE, OBJ_DATA_NUM_2};

/* rpdo 通信参数字典对象定义 ************************************************************************************************************ */
									/* 1400~1403 SUB0 */
const ODOBJTypeDef  OBJ_RPDO1_COMP_NUM_SUB0			= {OD_RPDO1_COMP, RPDO_COMP_NUM_SUB0,          OBJ_READ_ONLY, OBJ_DATA_NUM_1};
const ODOBJTypeDef  OBJ_RPDO2_COMP_NUM_SUB0			= {OD_RPDO2_COMP, RPDO_COMP_NUM_SUB0,          OBJ_READ_ONLY, OBJ_DATA_NUM_1};
const ODOBJTypeDef  OBJ_RPDO3_COMP_NUM_SUB0			= {OD_RPDO3_COMP, RPDO_COMP_NUM_SUB0,          OBJ_READ_ONLY, OBJ_DATA_NUM_1};
const ODOBJTypeDef  OBJ_RPDO4_COMP_NUM_SUB0			= {OD_RPDO4_COMP, RPDO_COMP_NUM_SUB0,          OBJ_READ_ONLY, OBJ_DATA_NUM_1};
									/* 1400~1403 SUB1 */
const ODOBJTypeDef	OBJ_RPDO1_COMP_COBID_Sub1 		= {OD_RPDO1_COMP, RPDO_COMP_COBID_PDO_SUB1,     OBJ_READ_WRITE, OBJ_DATA_NUM_4};
const ODOBJTypeDef	OBJ_RPDO2_COMP_COBID_Sub1 		= {OD_RPDO2_COMP, RPDO_COMP_COBID_PDO_SUB1,     OBJ_READ_WRITE, OBJ_DATA_NUM_4};
const ODOBJTypeDef	OBJ_RPDO3_COMP_COBID_Sub1 		= {OD_RPDO3_COMP, RPDO_COMP_COBID_PDO_SUB1,     OBJ_READ_WRITE, OBJ_DATA_NUM_4};
const ODOBJTypeDef	OBJ_RPDO4_COMP_COBID_Sub1 		= {OD_RPDO4_COMP, RPDO_COMP_COBID_PDO_SUB1,     OBJ_READ_WRITE, OBJ_DATA_NUM_4};
									/* 1400~1403 SUB2 */
const ODOBJTypeDef	OBJ_RPDO1_COMP_TRANS_TYPE_Sub2 	= {OD_RPDO1_COMP, RPDO_COMP_TRANSMIT_TYPE_SUB2, OBJ_READ_WRITE, OBJ_DATA_NUM_1};
const ODOBJTypeDef	OBJ_RPDO2_COMP_TRANS_TYPE_Sub2 	= {OD_RPDO2_COMP, RPDO_COMP_TRANSMIT_TYPE_SUB2, OBJ_READ_WRITE, OBJ_DATA_NUM_1};
const ODOBJTypeDef	OBJ_RPDO3_COMP_TRANS_TYPE_Sub2 	= {OD_RPDO3_COMP, RPDO_COMP_TRANSMIT_TYPE_SUB2, OBJ_READ_WRITE, OBJ_DATA_NUM_1};
const ODOBJTypeDef	OBJ_RPDO4_COMP_TRANS_TYPE_Sub2 	= {OD_RPDO4_COMP, RPDO_COMP_TRANSMIT_TYPE_SUB2, OBJ_READ_WRITE, OBJ_DATA_NUM_1};

/* tpdo 通信参数字典对象定义 ************************************************************************************************************ */
									/* 1800~1803 SUB0 */
const ODOBJTypeDef	obj_TPDO1_COMP_NUM_SUB0			= {OD_TPDO1_COMP, TPDO_COMP_NUM_SUB0,			OBJ_READ_ONLY, OBJ_DATA_NUM_1};
const ODOBJTypeDef	obj_TPDO2_COMP_NUM_SUB0			= {OD_TPDO2_COMP, TPDO_COMP_NUM_SUB0,			OBJ_READ_ONLY, OBJ_DATA_NUM_1};
const ODOBJTypeDef	obj_TPDO3_COMP_NUM_SUB0			= {OD_TPDO3_COMP, TPDO_COMP_NUM_SUB0,			OBJ_READ_ONLY, OBJ_DATA_NUM_1};
const ODOBJTypeDef	obj_TPDO4_COMP_NUM_SUB0			= {OD_TPDO4_COMP, TPDO_COMP_NUM_SUB0,			OBJ_READ_ONLY, OBJ_DATA_NUM_1};
									/* 1800~1803 SUB1 */
const ODOBJTypeDef	OBJ_TPDO1_COMP_COBID_Sub1		= {OD_TPDO1_COMP, TPDO_COMP_COBID_PDO_SUB1,		OBJ_READ_WRITE, OBJ_DATA_NUM_4};
const ODOBJTypeDef	OBJ_TPDO2_COMP_COBID_Sub1		= {OD_TPDO2_COMP, TPDO_COMP_COBID_PDO_SUB1,		OBJ_READ_WRITE, OBJ_DATA_NUM_4};
const ODOBJTypeDef	OBJ_TPDO3_COMP_COBID_Sub1		= {OD_TPDO3_COMP, TPDO_COMP_COBID_PDO_SUB1,		OBJ_READ_WRITE, OBJ_DATA_NUM_4};
const ODOBJTypeDef	OBJ_TPDO4_COMP_COBID_Sub1		= {OD_TPDO4_COMP, TPDO_COMP_COBID_PDO_SUB1,		OBJ_READ_WRITE, OBJ_DATA_NUM_4};
									/* 1800~1803 SUB2 */
const ODOBJTypeDef	OBJ_TPDO1_COMP_TRANS_TYPE_Sub2	= {OD_TPDO1_COMP, TPDO_COMP_TRANSMIT_TYPE_SUB2,	OBJ_READ_WRITE, OBJ_DATA_NUM_1};
const ODOBJTypeDef	OBJ_TPDO2_COMP_TRANS_TYPE_Sub2	= {OD_TPDO2_COMP, TPDO_COMP_TRANSMIT_TYPE_SUB2,	OBJ_READ_WRITE, OBJ_DATA_NUM_1};
const ODOBJTypeDef	OBJ_TPDO3_COMP_TRANS_TYPE_Sub2	= {OD_TPDO3_COMP, TPDO_COMP_TRANSMIT_TYPE_SUB2,	OBJ_READ_WRITE, OBJ_DATA_NUM_1};
const ODOBJTypeDef	OBJ_TPDO4_COMP_TRANS_TYPE_Sub2	= {OD_TPDO4_COMP, TPDO_COMP_TRANSMIT_TYPE_SUB2,	OBJ_READ_WRITE, OBJ_DATA_NUM_1};
									/* 1800~1803 SUB3 */
const ODOBJTypeDef	OBJ_TPDO1_COMP_INHIBIT_TIME_Sub3= {OD_TPDO1_COMP, TPDO_COMP_INHIBIT_TIME_SUB3,	OBJ_READ_WRITE,	OBJ_DATA_NUM_2};
const ODOBJTypeDef	OBJ_TPDO2_COMP_INHIBIT_TIME_Sub3= {OD_TPDO2_COMP, TPDO_COMP_INHIBIT_TIME_SUB3,	OBJ_READ_WRITE,	OBJ_DATA_NUM_2};
const ODOBJTypeDef	OBJ_TPDO3_COMP_INHIBIT_TIME_Sub3= {OD_TPDO3_COMP, TPDO_COMP_INHIBIT_TIME_SUB3,	OBJ_READ_WRITE,	OBJ_DATA_NUM_2};
const ODOBJTypeDef	OBJ_TPDO4_COMP_INHIBIT_TIME_Sub3= {OD_TPDO4_COMP, TPDO_COMP_INHIBIT_TIME_SUB3,	OBJ_READ_WRITE,	OBJ_DATA_NUM_2};
									/* 1800~1803 SUB5 */
const ODOBJTypeDef	OBJ_TPDO1_COMP_EVENT_TIMER_Sub5	= {OD_TPDO1_COMP, TPDO_COMP_EVENT_TIMER_SUB5,	OBJ_READ_WRITE,	OBJ_DATA_NUM_2};
const ODOBJTypeDef	OBJ_TPDO2_COMP_EVENT_TIMER_Sub5	= {OD_TPDO2_COMP, TPDO_COMP_EVENT_TIMER_SUB5,	OBJ_READ_WRITE,	OBJ_DATA_NUM_2};
const ODOBJTypeDef	OBJ_TPDO3_COMP_EVENT_TIMER_Sub5	= {OD_TPDO3_COMP, TPDO_COMP_EVENT_TIMER_SUB5,	OBJ_READ_WRITE,	OBJ_DATA_NUM_2};
const ODOBJTypeDef	OBJ_TPDO4_COMP_EVENT_TIMER_Sub5	= {OD_TPDO4_COMP, TPDO_COMP_EVENT_TIMER_SUB5,	OBJ_READ_WRITE,	OBJ_DATA_NUM_2};

/* rpdo 映射参数字典对象定义 ************************************************************************************************************ */
									/* 1600~1603 SUB0 */
const ODOBJTypeDef	OBJ_RPDO1_MAPPING_NUM_Sub0		= {OD_RPDO1_MAPPING, RPDO_MAPPING_NUM_SUB0, OBJ_READ_WRITE, OBJ_DATA_NUM_1};
const ODOBJTypeDef	OBJ_RPDO2_MAPPING_NUM_Sub0		= {OD_RPDO2_MAPPING, RPDO_MAPPING_NUM_SUB0, OBJ_READ_WRITE, OBJ_DATA_NUM_1};
const ODOBJTypeDef	OBJ_RPDO3_MAPPING_NUM_Sub0		= {OD_RPDO3_MAPPING, RPDO_MAPPING_NUM_SUB0, OBJ_READ_WRITE, OBJ_DATA_NUM_1};
const ODOBJTypeDef	OBJ_RPDO4_MAPPING_NUM_Sub0		= {OD_RPDO4_MAPPING, RPDO_MAPPING_NUM_SUB0, OBJ_READ_WRITE, OBJ_DATA_NUM_1};
									/* 1800 SUB1~8    */
const ODOBJTypeDef	OBJ_RPDO1_MAPPING_PDO_Sub1		= {OD_RPDO1_MAPPING, RPDO_MAPPING_PDO_SUB1, OBJ_READ_WRITE, OBJ_DATA_NUM_4};
const ODOBJTypeDef	OBJ_RPDO1_MAPPING_PDO_Sub2		= {OD_RPDO1_MAPPING, RPDO_MAPPING_PDO_SUB2, OBJ_READ_WRITE, OBJ_DATA_NUM_4};
const ODOBJTypeDef	OBJ_RPDO1_MAPPING_PDO_Sub3		= {OD_RPDO1_MAPPING, RPDO_MAPPING_PDO_SUB3, OBJ_READ_WRITE, OBJ_DATA_NUM_4};
const ODOBJTypeDef	OBJ_RPDO1_MAPPING_PDO_Sub4		= {OD_RPDO1_MAPPING, RPDO_MAPPING_PDO_SUB4, OBJ_READ_WRITE, OBJ_DATA_NUM_4};
const ODOBJTypeDef	OBJ_RPDO1_MAPPING_PDO_Sub5		= {OD_RPDO1_MAPPING, RPDO_MAPPING_PDO_SUB5, OBJ_READ_WRITE, OBJ_DATA_NUM_4};
const ODOBJTypeDef	OBJ_RPDO1_MAPPING_PDO_Sub6		= {OD_RPDO1_MAPPING, RPDO_MAPPING_PDO_SUB6, OBJ_READ_WRITE, OBJ_DATA_NUM_4};
const ODOBJTypeDef	OBJ_RPDO1_MAPPING_PDO_Sub7		= {OD_RPDO1_MAPPING, RPDO_MAPPING_PDO_SUB7, OBJ_READ_WRITE, OBJ_DATA_NUM_4};
const ODOBJTypeDef	OBJ_RPDO1_MAPPING_PDO_Sub8		= {OD_RPDO1_MAPPING, RPDO_MAPPING_PDO_SUB8, OBJ_READ_WRITE, OBJ_DATA_NUM_4};
									/* 1801 SUB1~8    */
const ODOBJTypeDef	OBJ_RPDO2_MAPPING_PDO_Sub1		= {OD_RPDO2_MAPPING, RPDO_MAPPING_PDO_SUB1, OBJ_READ_WRITE, OBJ_DATA_NUM_4};
const ODOBJTypeDef	OBJ_RPDO2_MAPPING_PDO_Sub2		= {OD_RPDO2_MAPPING, RPDO_MAPPING_PDO_SUB2, OBJ_READ_WRITE, OBJ_DATA_NUM_4};
const ODOBJTypeDef	OBJ_RPDO2_MAPPING_PDO_Sub3		= {OD_RPDO2_MAPPING, RPDO_MAPPING_PDO_SUB3, OBJ_READ_WRITE, OBJ_DATA_NUM_4};
const ODOBJTypeDef	OBJ_RPDO2_MAPPING_PDO_Sub4		= {OD_RPDO2_MAPPING, RPDO_MAPPING_PDO_SUB4, OBJ_READ_WRITE, OBJ_DATA_NUM_4};
const ODOBJTypeDef	OBJ_RPDO2_MAPPING_PDO_Sub5		= {OD_RPDO2_MAPPING, RPDO_MAPPING_PDO_SUB5, OBJ_READ_WRITE, OBJ_DATA_NUM_4};
const ODOBJTypeDef	OBJ_RPDO2_MAPPING_PDO_Sub6		= {OD_RPDO2_MAPPING, RPDO_MAPPING_PDO_SUB6, OBJ_READ_WRITE, OBJ_DATA_NUM_4};
const ODOBJTypeDef	OBJ_RPDO2_MAPPING_PDO_Sub7		= {OD_RPDO2_MAPPING, RPDO_MAPPING_PDO_SUB7, OBJ_READ_WRITE, OBJ_DATA_NUM_4};
const ODOBJTypeDef	OBJ_RPDO2_MAPPING_PDO_Sub8		= {OD_RPDO2_MAPPING, RPDO_MAPPING_PDO_SUB8, OBJ_READ_WRITE, OBJ_DATA_NUM_4};
									/* 1802 SUB1~8    */
const ODOBJTypeDef	OBJ_RPDO3_MAPPING_PDO_Sub1		= {OD_RPDO3_MAPPING, RPDO_MAPPING_PDO_SUB1, OBJ_READ_WRITE, OBJ_DATA_NUM_4};
const ODOBJTypeDef	OBJ_RPDO3_MAPPING_PDO_Sub2		= {OD_RPDO3_MAPPING, RPDO_MAPPING_PDO_SUB2, OBJ_READ_WRITE, OBJ_DATA_NUM_4};
const ODOBJTypeDef	OBJ_RPDO3_MAPPING_PDO_Sub3		= {OD_RPDO3_MAPPING, RPDO_MAPPING_PDO_SUB3, OBJ_READ_WRITE, OBJ_DATA_NUM_4};
const ODOBJTypeDef	OBJ_RPDO3_MAPPING_PDO_Sub4		= {OD_RPDO3_MAPPING, RPDO_MAPPING_PDO_SUB4, OBJ_READ_WRITE, OBJ_DATA_NUM_4};
const ODOBJTypeDef	OBJ_RPDO3_MAPPING_PDO_Sub5		= {OD_RPDO3_MAPPING, RPDO_MAPPING_PDO_SUB5, OBJ_READ_WRITE, OBJ_DATA_NUM_4};
const ODOBJTypeDef	OBJ_RPDO3_MAPPING_PDO_Sub6		= {OD_RPDO3_MAPPING, RPDO_MAPPING_PDO_SUB6, OBJ_READ_WRITE, OBJ_DATA_NUM_4};
const ODOBJTypeDef	OBJ_RPDO3_MAPPING_PDO_Sub7		= {OD_RPDO3_MAPPING, RPDO_MAPPING_PDO_SUB7, OBJ_READ_WRITE, OBJ_DATA_NUM_4};
const ODOBJTypeDef	OBJ_RPDO3_MAPPING_PDO_Sub8		= {OD_RPDO3_MAPPING, RPDO_MAPPING_PDO_SUB8, OBJ_READ_WRITE, OBJ_DATA_NUM_4};
									/* 1803 SUB1~8    */
const ODOBJTypeDef	OBJ_RPDO4_MAPPING_PDO_Sub1		= {OD_RPDO4_MAPPING, RPDO_MAPPING_PDO_SUB1, OBJ_READ_WRITE, OBJ_DATA_NUM_4};
const ODOBJTypeDef	OBJ_RPDO4_MAPPING_PDO_Sub2		= {OD_RPDO4_MAPPING, RPDO_MAPPING_PDO_SUB2, OBJ_READ_WRITE, OBJ_DATA_NUM_4};
const ODOBJTypeDef	OBJ_RPDO4_MAPPING_PDO_Sub3		= {OD_RPDO4_MAPPING, RPDO_MAPPING_PDO_SUB3, OBJ_READ_WRITE, OBJ_DATA_NUM_4};
const ODOBJTypeDef	OBJ_RPDO4_MAPPING_PDO_Sub4		= {OD_RPDO4_MAPPING, RPDO_MAPPING_PDO_SUB4, OBJ_READ_WRITE, OBJ_DATA_NUM_4};
const ODOBJTypeDef	OBJ_RPDO4_MAPPING_PDO_Sub5		= {OD_RPDO4_MAPPING, RPDO_MAPPING_PDO_SUB5, OBJ_READ_WRITE, OBJ_DATA_NUM_4};
const ODOBJTypeDef	OBJ_RPDO4_MAPPING_PDO_Sub6		= {OD_RPDO4_MAPPING, RPDO_MAPPING_PDO_SUB6, OBJ_READ_WRITE, OBJ_DATA_NUM_4};
const ODOBJTypeDef	OBJ_RPDO4_MAPPING_PDO_Sub7		= {OD_RPDO4_MAPPING, RPDO_MAPPING_PDO_SUB7, OBJ_READ_WRITE, OBJ_DATA_NUM_4};
const ODOBJTypeDef	OBJ_RPDO4_MAPPING_PDO_Sub8		= {OD_RPDO4_MAPPING, RPDO_MAPPING_PDO_SUB8, OBJ_READ_WRITE, OBJ_DATA_NUM_4};

/* Tpdo 映射参数字典对象定义 ************************************************************************************************************ */
									/* 1A00~1A03 SUB0   */
const ODOBJTypeDef	OBJ_TPDO1_MAPPING_NUM_Sub0		= {OD_TPDO1_MAPPING, TPDO_MAPPING_NUM_SUB0, OBJ_READ_WRITE, OBJ_DATA_NUM_1};
const ODOBJTypeDef	OBJ_TPDO2_MAPPING_NUM_Sub0		= {OD_TPDO2_MAPPING, TPDO_MAPPING_NUM_SUB0, OBJ_READ_WRITE, OBJ_DATA_NUM_1};
const ODOBJTypeDef	OBJ_TPDO3_MAPPING_NUM_Sub0		= {OD_TPDO3_MAPPING, TPDO_MAPPING_NUM_SUB0, OBJ_READ_WRITE, OBJ_DATA_NUM_1};
const ODOBJTypeDef	OBJ_TPDO4_MAPPING_NUM_Sub0		= {OD_TPDO4_MAPPING, TPDO_MAPPING_NUM_SUB0, OBJ_READ_WRITE, OBJ_DATA_NUM_1};
									/* 1A00 SUB1~8  	*/
const ODOBJTypeDef	OBJ_TPDO1_MAPPING_PDO_Sub1		= {OD_TPDO1_MAPPING, TPDO_MAPPING_PDO_SUB1, OBJ_READ_WRITE, OBJ_DATA_NUM_4};
const ODOBJTypeDef	OBJ_TPDO1_MAPPING_PDO_Sub2		= {OD_TPDO1_MAPPING, TPDO_MAPPING_PDO_SUB2, OBJ_READ_WRITE, OBJ_DATA_NUM_4};
const ODOBJTypeDef	OBJ_TPDO1_MAPPING_PDO_Sub3		= {OD_TPDO1_MAPPING, TPDO_MAPPING_PDO_SUB3, OBJ_READ_WRITE, OBJ_DATA_NUM_4};
const ODOBJTypeDef	OBJ_TPDO1_MAPPING_PDO_Sub4		= {OD_TPDO1_MAPPING, TPDO_MAPPING_PDO_SUB4, OBJ_READ_WRITE, OBJ_DATA_NUM_4};
const ODOBJTypeDef	OBJ_TPDO1_MAPPING_PDO_Sub5		= {OD_TPDO1_MAPPING, TPDO_MAPPING_PDO_SUB5, OBJ_READ_WRITE, OBJ_DATA_NUM_4};
const ODOBJTypeDef	OBJ_TPDO1_MAPPING_PDO_Sub6		= {OD_TPDO1_MAPPING, TPDO_MAPPING_PDO_SUB6, OBJ_READ_WRITE, OBJ_DATA_NUM_4};
const ODOBJTypeDef	OBJ_TPDO1_MAPPING_PDO_Sub7		= {OD_TPDO1_MAPPING, TPDO_MAPPING_PDO_SUB7, OBJ_READ_WRITE, OBJ_DATA_NUM_4};
const ODOBJTypeDef	OBJ_TPDO1_MAPPING_PDO_Sub8		= {OD_TPDO1_MAPPING, TPDO_MAPPING_PDO_SUB8, OBJ_READ_WRITE, OBJ_DATA_NUM_4};
									/* 1A01 SUB1~8      */
const ODOBJTypeDef	OBJ_TPDO2_MAPPING_PDO_Sub1		= {OD_TPDO2_MAPPING, TPDO_MAPPING_PDO_SUB1, OBJ_READ_WRITE, OBJ_DATA_NUM_4};
const ODOBJTypeDef	OBJ_TPDO2_MAPPING_PDO_Sub2		= {OD_TPDO2_MAPPING, TPDO_MAPPING_PDO_SUB2, OBJ_READ_WRITE, OBJ_DATA_NUM_4};
const ODOBJTypeDef	OBJ_TPDO2_MAPPING_PDO_Sub3		= {OD_TPDO2_MAPPING, TPDO_MAPPING_PDO_SUB3, OBJ_READ_WRITE, OBJ_DATA_NUM_4};
const ODOBJTypeDef	OBJ_TPDO2_MAPPING_PDO_Sub4		= {OD_TPDO2_MAPPING, TPDO_MAPPING_PDO_SUB4, OBJ_READ_WRITE, OBJ_DATA_NUM_4};
const ODOBJTypeDef	OBJ_TPDO2_MAPPING_PDO_Sub5		= {OD_TPDO2_MAPPING, TPDO_MAPPING_PDO_SUB5, OBJ_READ_WRITE, OBJ_DATA_NUM_4};
const ODOBJTypeDef	OBJ_TPDO2_MAPPING_PDO_Sub6		= {OD_TPDO2_MAPPING, TPDO_MAPPING_PDO_SUB6, OBJ_READ_WRITE, OBJ_DATA_NUM_4};
const ODOBJTypeDef	OBJ_TPDO2_MAPPING_PDO_Sub7		= {OD_TPDO2_MAPPING, TPDO_MAPPING_PDO_SUB7, OBJ_READ_WRITE, OBJ_DATA_NUM_4};
const ODOBJTypeDef	OBJ_TPDO2_MAPPING_PDO_Sub8		= {OD_TPDO2_MAPPING, TPDO_MAPPING_PDO_SUB8, OBJ_READ_WRITE, OBJ_DATA_NUM_4};
									/* 1A02 SUB1~8     */
const ODOBJTypeDef	OBJ_TPDO3_MAPPING_PDO_Sub1		= {OD_TPDO3_MAPPING, TPDO_MAPPING_PDO_SUB1, OBJ_READ_WRITE, OBJ_DATA_NUM_4};
const ODOBJTypeDef	OBJ_TPDO3_MAPPING_PDO_Sub2		= {OD_TPDO3_MAPPING, TPDO_MAPPING_PDO_SUB2, OBJ_READ_WRITE, OBJ_DATA_NUM_4};
const ODOBJTypeDef	OBJ_TPDO3_MAPPING_PDO_Sub3		= {OD_TPDO3_MAPPING, TPDO_MAPPING_PDO_SUB3, OBJ_READ_WRITE, OBJ_DATA_NUM_4};
const ODOBJTypeDef	OBJ_TPDO3_MAPPING_PDO_Sub4		= {OD_TPDO3_MAPPING, TPDO_MAPPING_PDO_SUB4, OBJ_READ_WRITE, OBJ_DATA_NUM_4};
const ODOBJTypeDef	OBJ_TPDO3_MAPPING_PDO_Sub5		= {OD_TPDO3_MAPPING, TPDO_MAPPING_PDO_SUB5, OBJ_READ_WRITE, OBJ_DATA_NUM_4};
const ODOBJTypeDef	OBJ_TPDO3_MAPPING_PDO_Sub6		= {OD_TPDO3_MAPPING, TPDO_MAPPING_PDO_SUB6, OBJ_READ_WRITE, OBJ_DATA_NUM_4};
const ODOBJTypeDef	OBJ_TPDO3_MAPPING_PDO_Sub7		= {OD_TPDO3_MAPPING, TPDO_MAPPING_PDO_SUB7, OBJ_READ_WRITE, OBJ_DATA_NUM_4};
const ODOBJTypeDef	OBJ_TPDO3_MAPPING_PDO_Sub8		= {OD_TPDO3_MAPPING, TPDO_MAPPING_PDO_SUB8, OBJ_READ_WRITE, OBJ_DATA_NUM_4};
									/* 1A03 SUB1~8     */
const ODOBJTypeDef	OBJ_TPDO4_MAPPING_PDO_Sub1		= {OD_TPDO4_MAPPING, TPDO_MAPPING_PDO_SUB1, OBJ_READ_WRITE, OBJ_DATA_NUM_4};
const ODOBJTypeDef	OBJ_TPDO4_MAPPING_PDO_Sub2		= {OD_TPDO4_MAPPING, TPDO_MAPPING_PDO_SUB2, OBJ_READ_WRITE, OBJ_DATA_NUM_4};
const ODOBJTypeDef	OBJ_TPDO4_MAPPING_PDO_Sub3		= {OD_TPDO4_MAPPING, TPDO_MAPPING_PDO_SUB3, OBJ_READ_WRITE, OBJ_DATA_NUM_4};
const ODOBJTypeDef	OBJ_TPDO4_MAPPING_PDO_Sub4		= {OD_TPDO4_MAPPING, TPDO_MAPPING_PDO_SUB4, OBJ_READ_WRITE, OBJ_DATA_NUM_4};
const ODOBJTypeDef	OBJ_TPDO4_MAPPING_PDO_Sub5		= {OD_TPDO4_MAPPING, TPDO_MAPPING_PDO_SUB5, OBJ_READ_WRITE, OBJ_DATA_NUM_4};
const ODOBJTypeDef	OBJ_TPDO4_MAPPING_PDO_Sub6		= {OD_TPDO4_MAPPING, TPDO_MAPPING_PDO_SUB6, OBJ_READ_WRITE, OBJ_DATA_NUM_4};
const ODOBJTypeDef	OBJ_TPDO4_MAPPING_PDO_Sub7		= {OD_TPDO4_MAPPING, TPDO_MAPPING_PDO_SUB7, OBJ_READ_WRITE, OBJ_DATA_NUM_4};
const ODOBJTypeDef	OBJ_TPDO4_MAPPING_PDO_Sub8		= {OD_TPDO4_MAPPING, TPDO_MAPPING_PDO_SUB8, OBJ_READ_WRITE, OBJ_DATA_NUM_4};


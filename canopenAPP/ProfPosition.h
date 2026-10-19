/*
 * ProfPosition.h
 *
 * Profile Position 模式相关的具体功能
 * SDO 方式下对电机做位置控制
 *
 *  Created on: 2021年10月22日
 *      Author: maxiufeng
 */

#ifndef APP_INC_PROFPOSITION_H_
#define APP_INC_PROFPOSITION_H_

#include "canopen.h"

/* PP 单个设定点初始化操作 【新点的执行是基于前一个点被舍弃的基础之上的】。
 * [#] 设置运动模式 @CAN_SDO_Motion_Mode -》MotionModeTypeDef
 * [#] 设置定位选项代码 @
 * [#] 设置加减速度 @CAN_SDO_PP_Profile_xxxxxx  @CAN_SDO_PP_Quick_Stop_Deceleration
 * [#] 设置控制字的值使驱动器处于 immediately=1，set-point=0， status：OP_EN
 * 		@CAN_SDO_ControlWord
 * 	*/
int PP_Single_SetPoint_Init(canDriver *driver, uint16_t NodeID);

/* PP set-points 目标位置 + 轮廓速度
 * [#] 检查状态字的bit12 set-point ACK, =0表示set-point缓存可用 ???
 * [#] 发送 目标位置 @CAN_SDO_PP_Target_Position
 * [#] 发送 轮廓速度 @CAN_SDO_PP_Profile_Velocity
 * [#] 发送 控制字bit4=1 使位置和速度被驱动器接收， 同时驱动器设置状态字bit12=1
 * [#] 发送 控制字bit4=0 为下一次发送数据做准备；让驱动器能够设置状态字的bit12=0
 * */
int PP_SetPoints_Value(canDriver *driver, uint16_t NodeID, int32_t tarPos, uint32_t profVel);

/* PP 点到点运动暂停方法 */
int PP_Point_Stop(canDriver *driver, uint16_t nodeID);

/* PP 模式下 回归 0 点，这个 0 点是通过elmo软件配置的，可以通过软件修改为任意位置成 0 位
 *    这是PP_SetPoints_Value() 的一个特例 PP_SetPoints_Value()也可实现归0功能*/
int PP_Home_Zero(canDriver *driver, uint16_t modeID);


/* ********************************************************************************************************************* */

/* PP 用相对运动实现定速转动 jogging。
 * [#] 设置运动模式 @CAN_SDO_Motion_Mode -》MotionModeTypeDef
 * [#] 设置定位选项代码 @ POCTypeDef
 * [#] 设置加减速度 @CAN_SDO_PP_Profile_xxxxxx  @CAN_SDO_PP_Quick_Stop_Deceleration
 * [#] 设置控制字的值使驱动器处于 immediately=1，set-point=0， status：OP_EN
 * 		@CAN_SDO_ControlWord
 * 	*/
int PP_Jogging_Init(canDriver *driver, uint16_t NodeID);

/* PP 模式 jogging相对运动 设置速度值
 *   速度设置正值 -- 电机正转
 *   速度设置负值 -- 电机反转 */
int PP_Set_Jogging(canDriver *driver, uint16_t NodeID, int32_t jogVel);

/* PP 模式下 jogging暂停运动 */
int PP_Jogging_Stop(canDriver *driver, uint16_t nodeId);

/* ********************************************************************************************************************* */

/* PP 模式退出操作 也就是驱动器退出 operation enable 状态 */
int PP_Driver_Disable(canDriver *driver, uint16_t NodeID);

#endif /* APP_INC_PROFPOSITION_H_ */

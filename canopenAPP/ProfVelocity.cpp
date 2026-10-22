#include "ProfVelocity.h"

int PV_Init(canDriver *driver, uint16_t NodeID)
{
    
    uint16_t controlResult = 0;
    uint32_t result = 0;
    //FAULT——RESET，上升触发，所以先置0。
    if (CAN_SDO_ControlWord(driver, NodeID, CW_CLEAR, &controlResult) != 1) return 0;
    if (CAN_SDO_ControlWord(driver, NodeID, CW_FAULT_RESELT, &controlResult) != 1) return 0;
    if (CAN_SDO_PV_Target_Velocity(driver, NodeID, 0, &result) != 1) return 0;
    if (PV_Set_Profile(driver, NodeID,1000,1000,1000) !=1) return 0;
    if (PV_Set_Velocity_Monitoring(driver, NodeID) != 1) return 0;
    if (CAN_SDO_Motion_Mode(driver, NodeID, MOTION_MODE_PV, &result) != 1) return 0;
    if (CAN_SDO_ControlWord(driver, NodeID, CW_SHUT_DOWN, &controlResult) != 1) return 0;
    if (CAN_SDO_ControlWord(driver, NodeID, CW_SWITCH_ON, &controlResult) != 1) return 0;
    return 1;
}

int PV_Enable(canDriver *driver, uint16_t NodeID)
{
    uint32_t result = 0;
    uint16_t controlResult = 0;
    if (CAN_SDO_PV_Target_Velocity(driver, NodeID, 0, &result) != 1) return 0;
    //到ready to switch on -> swith on -> enable operation
    //驱动器有错误时这个不行，必须第7位上升触发 FAULT——RESET。
    if (CAN_SDO_ControlWord(driver, NodeID, CW_SHUT_DOWN, &controlResult) != 1) return 0;
    if (CAN_SDO_ControlWord(driver, NodeID, CW_SWITCH_ON, &controlResult) != 1) return 0;
    if (CAN_SDO_ControlWord(driver, NodeID, CW_EN_OP, &controlResult) != 1) return 0;
    return 1;
}

int PV_Set_Profile(canDriver *driver, uint16_t NodeID,
                   uint32_t acceleration, uint32_t deceleration,
                   uint32_t quickStopDeceleration)
{
    if (acceleration == 0 || deceleration == 0 || quickStopDeceleration == 0) return 0;
    uint32_t result = 0;
    if (CAN_SDO_PV_Profile_Acceleration(driver, NodeID, acceleration, &result) != 1) return 0;
    if (CAN_SDO_PV_Profile_Deceleration(driver, NodeID, deceleration, &result) != 1) return 0;
    if (CAN_SDO_PV_Quick_Stop_Deceleration(driver, NodeID, quickStopDeceleration, &result) != 1) return 0;
    return 1;
}

// velocityWindow、velocityThreshold：驱动器速度单位；两个 TimeMs 参数：毫秒（ms）。
int PV_Set_Velocity_Monitoring(canDriver *driver, uint16_t NodeID,
                              uint16_t velocityWindow, uint16_t velocityWindowTimeMs,
                              uint16_t velocityThreshold, uint16_t velocityThresholdTimeMs)
{
    uint32_t result = 0;
    if (CAN_SDO_PV_Velocity_Window(driver, NodeID, velocityWindow, &result) != 1) return 0;
    if (CAN_SDO_PV_Velocity_Window_Time(driver, NodeID, velocityWindowTimeMs, &result) != 1) return 0;
    if (CAN_SDO_PV_Velocity_Threshold(driver, NodeID, velocityThreshold, &result) != 1) return 0;
    if (CAN_SDO_PV_Velocity_Threshold_Time(driver, NodeID, velocityThresholdTimeMs, &result) != 1) return 0;
    return 1;
}

int PV_Set_Target_Velocity(canDriver *driver, uint16_t NodeID, int32_t tarVal)
{
    uint32_t result = 0;
    return CAN_SDO_PV_Target_Velocity(driver, NodeID, tarVal, &result) == 1 ? 1 : 0;
}

int PV_Stop(canDriver *driver, uint16_t NodeID)
{
    return PV_Set_Target_Velocity(driver, NodeID, 0);
}

int PV_Halt(canDriver *driver, uint16_t NodeID)
{
    uint16_t result = 0;
    const auto controlWord = static_cast<CWValTypeDef>(CW_EN_OP | CW_PV_HALT);
    return CAN_SDO_ControlWord(driver, NodeID, controlWord, &result) == 1 ? 1 : 0;
}

int PV_Disable(canDriver *driver, uint16_t NodeID)
{
    uint32_t result = 0;
    uint16_t controlResult = 0;
    int success = 1;
    if (CAN_SDO_PV_Target_Velocity(driver, NodeID, 0, &result) != 1) success = 0;
    // 即使清零目标失败，也尝试去使能。
    if (CAN_SDO_ControlWord(driver, NodeID, CW_SHUT_DOWN, &controlResult) != 1) success = 0;
    return success;
}

int PV_Config_PDO(canDriver *driver, uint16_t NodeID)
{
    // 调用方先将节点切换到 NMT Pre-Operational，配置完成后再切回 Operational。
    // TPDO1：状态字（16 bit）+ 实际电流（16 bit），与 IP 模式相同。
    if (CAN_PDO_Set_ComParam(driver, PDO_T1, NodeID, PDO_FUNCTION_DISABLE, PDO_Tx_1SYNC, 0, 0) != 1) return 0;
    ODOBJTypeDef tpdo1Objects[2] = {OBJ_Status_Word, OBJ_Current_Actual_Value};
    if (CAN_PDO_Set_MapParam(driver, PDO_T1, NodeID, 2, tpdo1Objects) != 1) return 0;
    if (CAN_PDO_Set_ComParam(driver, PDO_T1, NodeID, PDO_FUNCTION_SET_TRANS_TYPE, PDO_Tx_1SYNC, 0, 0) != 1) return 0;
    // TODO：TPDO1 inhibit time / event timer，如有需要在此配置。
    if (CAN_PDO_Set_ComParam(driver, PDO_T1, NodeID, PDO_FUNCTION_ENABLE, PDO_Tx_1SYNC, 0, 0) != 1) return 0;

    // TPDO2：实际位置（32 bit）+ 实际速度（32 bit），与现有反馈解析格式相同。
    if (CAN_PDO_Set_ComParam(driver, PDO_T2, NodeID, PDO_FUNCTION_DISABLE, PDO_Tx_1SYNC, 0, 0) != 1) return 0;
    ODOBJTypeDef tpdo2Objects[2] = {OBJ_Actual_Position_User, OBJ_Actual_Velocity_User};
    if (CAN_PDO_Set_MapParam(driver, PDO_T2, NodeID, 2, tpdo2Objects) != 1) return 0;
    if (CAN_PDO_Set_ComParam(driver, PDO_T2, NodeID, PDO_FUNCTION_SET_TRANS_TYPE, PDO_Tx_1SYNC, 0, 0) != 1) return 0;
    // TODO：确认 TPDO2 inhibit time；IP 中为 6000（单位 100 us，即 600 ms），此处暂不写入。
    if (CAN_PDO_Set_ComParam(driver, PDO_T2, NodeID, PDO_FUNCTION_ENABLE, PDO_Tx_1SYNC, 0, 0) != 1) return 0;

    // RPDO1：控制字（16 bit），与 IP 模式相同。
    if (CAN_PDO_Set_ComParam(driver, PDO_R1, NodeID, PDO_FUNCTION_DISABLE, PDO_Rx_1_SYNC, 0, 0) != 1) return 0;
    ODOBJTypeDef rpdo1Objects[1] = {OBJ_Control_Word};
    if (CAN_PDO_Set_MapParam(driver, PDO_R1, NodeID, 1, rpdo1Objects) != 1) return 0;
    if (CAN_PDO_Set_ComParam(driver, PDO_R1, NodeID, PDO_FUNCTION_SET_TRANS_TYPE, PDO_Rx_1_SYNC, 0, 0) != 1) return 0;
    if (CAN_PDO_Set_ComParam(driver, PDO_R1, NodeID, PDO_FUNCTION_ENABLE, PDO_Rx_1_SYNC, 0, 0) != 1) return 0;

    // RPDO2：PV 目标速度（0x60FF，32 bit），替换 IP 的插补位置对象。
    if (CAN_PDO_Set_ComParam(driver, PDO_R2, NodeID, PDO_FUNCTION_DISABLE, PDO_Rx_1_SYNC, 0, 0) != 1) return 0;
    ODOBJTypeDef rpdo2Objects[1] = {OBJ_PV_Target_Velocity};
    if (CAN_PDO_Set_MapParam(driver, PDO_R2, NodeID, 1, rpdo2Objects) != 1) return 0;
    if (CAN_PDO_Set_ComParam(driver, PDO_R2, NodeID, PDO_FUNCTION_SET_TRANS_TYPE, PDO_Rx_1_SYNC, 0, 0) != 1) return 0;
    if (CAN_PDO_Set_ComParam(driver, PDO_R2, NodeID, PDO_FUNCTION_ENABLE, PDO_Rx_1_SYNC, 0, 0) != 1) return 0;
    return 1;
}

int PV_Set_Target_Velocity_PDO(canDriver *driver, uint16_t NodeID, int32_t tarVal)
{
    if (driver == nullptr || NodeID < 1 || NodeID > 127) return 0;
    const uint32_t value = static_cast<uint32_t>(tarVal);
    uint8_t sendData[4] = {
        static_cast<uint8_t>(value & 0xFF),
        static_cast<uint8_t>((value >> 8) & 0xFF),
        static_cast<uint8_t>((value >> 16) & 0xFF),
        static_cast<uint8_t>((value >> 24) & 0xFF)
    };
    return CAN_PDO_Send_Data(driver, PDO_R2, NodeID, 4, sendData) == 1 ? 1 : 0;
}

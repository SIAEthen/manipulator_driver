#include "ProfVelocity.h"

int PV_Init(canDriver *driver, uint16_t NodeID)
{
    uint16_t controlResult = 0;
    uint32_t result = 0;
    if (CAN_SDO_ControlWord(driver, NodeID, CW_CLEAR, &controlResult) != 1) return 0;
    if (CAN_SDO_ControlWord(driver, NodeID, CW_FAULT_RESELT, &controlResult) != 1) return 0;
    if (CAN_SDO_PV_Target_Velocity(driver, NodeID, 0, &result) != 1) return 0;
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

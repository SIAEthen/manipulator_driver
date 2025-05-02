#include "canDriver.h"

canDriver::canDriver(int canID,int baudrate)
{
    m_canID = canID;
    m_baudrate = baudrate;

}
canDriver::~canDriver()
{
    deinitialize();
}
int canDriver::initialize()
{
    // The CANlib library is initialized by a call to canInitializeLibrary().
    canInitializeLibrary();
    //A channel to a CAN circuit is opened. In this case we open channel 0 which should be the first
    // channel on the CAN interface. canOPEN_EXCLUSIVE means we don't want to share this channel with
    // any other currently executing program.
    hnd = canOpenChannel(m_canID, canOPEN_EXCLUSIVE);
    if (hnd < 0) {
        char msg[64];
        canGetErrorText((canStatus)hnd, msg, sizeof(msg));
        fprintf(stderr, "canOpenChannel failed (%s)\n", msg);
        exit(1);
    }
    switch (m_baudrate)
    {
    case 1000:
        canSetBusParams(hnd, canBITRATE_1M, 0, 0, 0, 0, 0);
        break;
    case 500:
        canSetBusParams(hnd, canBITRATE_500K, 0, 0, 0, 0, 0);
        break;
    default:
        canSetBusParams(hnd, canBITRATE_500K, 0, 0, 0, 0, 0);
        break;
    }
    // The CAN bus driver type is set.
    canSetBusOutputControl(hnd, canDRIVER_NORMAL);
    // The CAN chip is activated.
    canBusOn(hnd);
    // // try to send a frame data
    // uint8_t data[8] = {0};
    // data[0] = 0x22;
    // data[1] = 0xc1;
    // data[2] = 0x60;
    // canWrite(hnd, 0x67f, data, 8, 0);
    // // Wait until the message is sent or at most 500 ms.
    // canWriteSync(hnd, 500);
    return 1;
}
int canDriver::deinitialize()
{
    // Inactivate the CAN chip.
    canBusOff(hnd);
    // Close the channel.
    canClose(hnd);
    return 1;
}

int canDriver::canDriverRead(long *cobid, u_char *rxdata, uint *dlc, uint *flags)
{
    if(canRead(hnd, cobid, rxdata, dlc, flags, &timestamp)==canOK)
        return 1;
    return 0;
}

    // time_wait
    // [in]	hnd	A handle to an open circuit.
    // [out]	id	Pointer to a buffer which receives the CAN identifier. This buffer will only get the identifier. To determine whether this identifier was standard (11-bit) or extended (29-bit), and/or whether it was remote or not, or if it was an error frame, examine the contents of the flag argument.
    // [out]	msg	Pointer to the buffer which receives the message data. This buffer must be large enough (i.e. 8 bytes for classic CAN and up to 64 bytes for CAN FD).
    // [out]	dlc	Pointer to a buffer which receives the message length.
    // [out]	flag	Pointer to a buffer which receives the message flags, which is a combination of the canMSG_xxx (including canFDMSG_xxx if the CAN FD protocol is enabled) and canMSGERR_xxx values.
    // [out]	time	Pointer to a buffer which receives the message time stamp. The unit of the time stamp is configurable using canIOCTL_SET_TIMER_SCALE, default is 1 ms.
    // [in]	timeout	If no message is immediately available, this parameter gives the number of milliseconds to wait for a message before returning. 0xFFFFFFFF gives an infinite timeout.
int canDriver::canDriverReadWait(long *cobid, u_char *rxdata, uint *dlc, uint *flags, unsigned long time_wait_ms)
{
    if(canReadWait(hnd, cobid, rxdata, dlc, flags, &timestamp,time_wait_ms)==canOK)
    {return 1; }
    return 0;
}




int canDriver::canDriverWrite(CAN_TxHeaderTypeDef header, u_char *txdata)
{
    stat = canWrite(hnd, header.StdId, txdata, header.DLC, 0);
    // Wait until the message is sent or at most 5 ms.
    // canWriteSync(hnd, header.time_ms);
    if (stat < 0) {
        printf("Write Failed, status == %d\n", stat);
        return 0;
    }
    return 1;
}

// wait at most time_wait_ms to get an msg, dont read the msg
int canDriver::canDriverReadSync(unsigned long time_wait_ms)
{
    if(canReadSync(hnd, time_wait_ms) == canOK)
    {
        return 1;
    }
    return 0;
}
    // Waits until all CAN messages for the specified handle are sent, or the timeout period expires
int canDriver::canDriverWriteSync(unsigned long time_wait_ms)
{
    if(canWriteSync(hnd, time_wait_ms) == canOK)
    {
        return 1;
    }
    return 0;
}





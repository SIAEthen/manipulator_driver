#ifndef __CANDRIVER_H__
#define __CANDRIVER_H__


#include <canlib.h>
#include <stdio.h>
struct CAN_TxHeaderTypeDef
{
    uint16_t StdId; // COBID of can
    uint8_t DLC;    // Data length
    uint8_t time_ms; //wait until it is sent
};
struct CAN_RxHeaderTypeDef
{
    long        StdId; // COBID of can
    uint        DLC;    // Data length
    uint        flags;
};

class canDriver
{   
public:
    canDriver(int canID,int baudrate);
    ~canDriver();
    int canDriverRead(long *cobid, u_char *rxdata, uint *dlc, uint *flags);
    // time_wait
    // [in]	hnd	A handle to an open circuit.
    // [out]	id	Pointer to a buffer which receives the CAN identifier. This buffer will only get the identifier. To determine whether this identifier was standard (11-bit) or extended (29-bit), and/or whether it was remote or not, or if it was an error frame, examine the contents of the flag argument.
    // [out]	msg	Pointer to the buffer which receives the message data. This buffer must be large enough (i.e. 8 bytes for classic CAN and up to 64 bytes for CAN FD).
    // [out]	dlc	Pointer to a buffer which receives the message length.
    // [out]	flag	Pointer to a buffer which receives the message flags, which is a combination of the canMSG_xxx (including canFDMSG_xxx if the CAN FD protocol is enabled) and canMSGERR_xxx values.
    // [out]	time	Pointer to a buffer which receives the message time stamp. The unit of the time stamp is configurable using canIOCTL_SET_TIMER_SCALE, default is 1 ms.
    // [in]	timeout	If no message is immediately available, this parameter gives the number of milliseconds to wait for a message before returning. 0xFFFFFFFF gives an infinite timeout.
    int canDriverReadWait(long *cobid, u_char *rxdata, uint *dlc, uint *flags, unsigned long time_wait_ms);
    int canDriverWrite(CAN_TxHeaderTypeDef header, u_char *txdata);
    // wait at most time_wait_ms to get an msg, dont read the msg
    int canDriverReadSync(unsigned long time_wait_ms);
        // Waits until all CAN messages for the specified handle are sent, or the timeout period expires
    int canDriverWriteSync(unsigned long time_wait_ms);
    int initialize();
    int deinitialize();
private:
    canHandle hnd;
    // unsigned char rxdata[8];
    // unsigned int dlc, flags;
    unsigned long timestamp;
    canStatus stat;
    int m_canID;
    int m_baudrate;

};

#endif  //__CANDRIVER_H__

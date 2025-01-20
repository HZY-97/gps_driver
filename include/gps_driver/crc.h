#ifndef __CRC_H
#define __CRC_H

#ifdef __cplusplus
extern "C" {
#endif

extern const unsigned long ulCrcTable[256];

unsigned int CalcMsgCRC(char *Msg, unsigned int len);
unsigned int CalculateCRC24(unsigned char *Buffer, unsigned int iLen);
unsigned short sensor_msg_crc16(const unsigned char *buf, unsigned int len);

#ifdef __cplusplus
}
#endif

#endif

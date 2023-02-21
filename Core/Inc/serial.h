#ifndef INC_SERIAL_H_
#define INC_SERIAL_H_

#include <stdint.h>
#include <stdbool.h>

#define MAX_LEN 22



typedef enum
{
    NONE = 1,
    SPID,
    VTUN,
    PTUN,
	STOP,
}PROCESS_t;


extern void SerialInit(void);
extern uint8_t *subString(uint8_t *pBuff, int Pos, int nIndex);
extern bool StrCompare(uint8_t *pBuff, uint8_t *pSample, uint8_t nSize);

extern void SerialParse(uint8_t *pBuff);
extern void SerialAcceptReceive(void);

#endif /* INC_SERIAL_H_ */

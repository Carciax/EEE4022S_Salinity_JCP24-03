#ifndef __RS485_H
#define __RS485_H

typedef enum {
    COMMAND_GET_STATUS = 0x00,
    COMMAND_GET_TEMPERATURE = 0x01,
    COMMAND_GET_DEPTH = 0x02,
    COMMAND_GET_SALINITY = 0x03,
    COMMAND_RESET = 0x04,
} RS485_Command;

typedef enum {
    STATUS_IDLE = 0x00,
    STATUS_ERROR = 0x01,
    STATUS_BUSY = 0x02,
} RS485_Status;

#endif
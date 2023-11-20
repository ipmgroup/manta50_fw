#ifndef ARCH_BYTE16
#include <arduino.h>
#include <CAN_BUS_Shield/mcp_can.h>

#include "can.h"

int8_t can_init(uint32_t speed) {
    return CAN.begin(speed);
}

uint8_t can_read(uint16_t *id, void *data, uint8_t *len) {
    if (CAN.checkReceive() != CAN_MSGAVAIL)
        return 0;

    if (CAN.readMsgBuf(len, (uint8_t *)data) != CAN_OK)
        return 0;

    *id = CAN.getCanId();

    return 1;
}

uint8_t can_write(uint16_t id, void *data, uint8_t len) {
    return CAN.sendMsgBuf(id, 0, len, (uint8_t *)data) == CAN_OK;
}
#endif

// #include <can.h>
#include "can.h"

#include "mcp_can.h"
#include "hal_obj.h"
#include "hal.h"

extern HAL_Handle halHandle;

uint8_t can_init(uint32_t bitrate)
{
    return 0;
}

uint8_t can_read(uint32_t *id, void *data, uint8_t *len)
{
    int rs = 1;
    // HAL_setupSpi_MCP2515(halHandle);
    if (MCP2515_checkReceive(halHandle->mcp2515Handle) != CAN_MSGAVAIL)
    {
        rs = 0;
        goto error;
    }

    if (MCP2515_readMsgBuf(halHandle->mcp2515Handle, len, (uint8_t *)data) != CAN_OK)
    {
        rs = 0;
        goto error;
    }

    *id = MCP2515_getCanId(/* halHandle->mcp2515Handle */);

error:
    // HAL_setupSpiA(halHandle);
    return rs;
}

uint8_t can_write(uint32_t id, void *data, uint8_t len)
{
    // HAL_setupSpi_MCP2515(halHandle);
    int rs = MCP2515_sendMsgBuf(halHandle->mcp2515Handle, id, 1, len, (uint8_t *)data);
    // HAL_setupSpiA(halHandle);
    return rs == CAN_OK ? 1 : 0;
}

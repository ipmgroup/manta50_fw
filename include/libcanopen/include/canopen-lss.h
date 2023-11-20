#ifndef __CANOPEN_LSS_H__
#define __CANOPEN_LSS_H__

#include "canopen.h"

#ifdef __cplusplus
extern "C" {
#endif

void lss_init(uint8_t cid);
ptype_t lss_dispatch(void *data, uint8_t *co_flags);

/*
 * Can be redefined by user. Default implementation return 1.
 * NOTE: implement this funtion in extern "C" block.
 *
 * @return: error. if first byte:
 *                 0   - success.
 *                 1   - store configuration is no supported
 *                 2   - storage media access error
 *                 255 - implementation specific error occured.
 *      if first byte 255, second byte must containe specific error code.
 **/
uint16_t lss_store_config(uint8_t node_id);

#ifdef __cplusplus
}
#endif

#endif

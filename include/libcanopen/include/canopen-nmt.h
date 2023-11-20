#ifndef __CANOPEN_NMT_H__
#define __CANOPEN_NMT_H__

#include "canopen.h"

#ifdef __cplusplus
extern "C" {
#endif

void nmt_init(uint8_t cid);
ptype_t nmt_dispatch(void *data, uint8_t *co_flags);

#ifdef __cplusplus
}
#endif

#endif

#ifndef __CANOPEN_PDO_H__
#define __CANOPEN_PDO_H__

#include "canopen.h"

#ifdef __cplusplus
extern "C" {
#endif

enum EDIR {
    RX_MAP, TX_MAP, RX_CONF, TX_CONF
};

#ifndef PDO_NUM
#define PDO_NUM 4
#endif

#define PDO_MAXNUM 4

#if (PDO_NUM < 0 || PDO_NUM > PDO_MAXNUM)
#   error PDO_NUM is incorrect
#endif

int pdo_init(pdo_map_t *pdo_map, uint8_t pdo_num);
ptype_t pdo_dispatch(uint8_t pdo_n, void *data, uint8_t len);
ptype_t sync_dispatch(void);
void pdo_write(uint8_t pdo_n);

#ifdef __cplusplus
}
#endif

#endif

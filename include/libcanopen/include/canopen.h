#ifndef __CANOPEN_H__
#define __CANOPEN_H__

#include <stdint.h>
#include <string.h> // NULL, size_t, memcpy()

#ifdef __cplusplus
extern "C" {
#endif

#define CO_FLAGS_CONF (1 << 0)
#define CO_FLAGS_PDO  (1 << 1)
#define CO_FLAGS_SDO  (1 << 2)

#define PDO_DEFAULT(n)      \
    .ptype   = CO_PDO##n,   \
    .rx_map  = {0},         \
    .tx_map  = {0},         \
    .rx_conf = 255,         \
    .tx_conf = 255

#ifdef ARCH_BYTE16
#   ifndef uint8_t
#   define uint8_t uint_least8_t
#   endif
#   ifndef int8_t
#   define int8_t int_least8_t
#   endif
extern uint8_t co_mbyte;

void *co_memcpy_pack(void *dest, const void *src, size_t len);
void *co_memcpy_unpack(void *dest, const void *src, size_t len);
void *co_memcpy_align(void *dest, const void *src, size_t len);
#else
#   define co_memcpy_pack   memcpy
#   define co_memcpy_unpack memcpy
#   define co_memcpy_align  memcpy
#endif

typedef enum { MOD_BOOT, MOD_PREOP, MOD_OP, MOD_CONF } opmode_t;

// NOTE: do not change the order of PDO1..4,SDO entries
//       and do not insert new items between them
typedef enum {
    CO_NONE, CO_FAIL, CO_SYNC, CO_LSS, CO_NMT,
    CO_PDO1, CO_PDO2, CO_PDO3, CO_PDO4, CO_SDO
} ptype_t;

typedef struct {
    ptype_t ptype;
    uint8_t rx_map[17], tx_map[17];
    uint8_t rx_conf, tx_conf;
} pdo_map_t;

int co_init(uint16_t cid, uint8_t init_nmt, pdo_map_t *pdo_map, uint8_t pdo_num);
ptype_t co_dispatch(uint16_t cob_id, void *data, uint8_t len);
uint8_t co_write(ptype_t ptype, void *data, uint8_t len);
void co_handle(void);

#ifdef __cplusplus
}
#endif

#include "canopen-dict.h"
#endif

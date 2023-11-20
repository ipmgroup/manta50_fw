#ifndef ARCH_BYTE16
#include <avr/pgmspace.h>
#endif

#include "can.h"
#include "canopen.h"
#include "canopen-sdo.h"
#include "canopen-pdo.h"
#include "canopen-lss.h"
#include "canopen-nmt.h"

#ifdef CO_MASTER
#   define TX_FLAG(flag) !(flag)
#else
#   define TX_FLAG(flag) (flag)
#endif

static ptype_t cob2ptype(uint16_t cob_id);
static uint16_t ptype2cob(ptype_t ptype);

static uint8_t co_flags = CO_FLAGS_SDO;
static uint16_t can_id;

int co_init(uint16_t cid, uint8_t init_nmt, pdo_map_t *pdo_map, uint8_t pdo_num) {
    can_id = cid;
    if (init_nmt)
        nmt_init(cid);
    lss_init(cid);

    return pdo_init(pdo_map, pdo_num);
}

void co_handle(void) {
    uint8_t buf[8], len;
    uint16_t cid;

    while (can_read(&cid, buf, &len))
        co_dispatch(cid, buf, len);
}

ptype_t co_dispatch(uint16_t cob_id, void *data, uint8_t len) {
    ptype_t ptype = cob2ptype(cob_id);

    switch (ptype) {
    case CO_NMT:
        if (!(co_flags & CO_FLAGS_CONF) && len == 2)
            return nmt_dispatch(data, &co_flags);

        break;

    case CO_LSS:
        if (len == 8)
            return lss_dispatch(data, &co_flags);

        break;

    case CO_SDO:
        if ((co_flags & CO_FLAGS_CONF))
            break;

        if ((co_flags & CO_FLAGS_SDO) && len == 8)
            return sdo_dispatch(data);

        break;

    case CO_SYNC:
        if ((co_flags & CO_FLAGS_CONF) || !(co_flags & CO_FLAGS_PDO))
            break;

        return sync_dispatch();

    case CO_PDO1:
    case CO_PDO2:
    case CO_PDO3:
    case CO_PDO4:
        if ((co_flags & CO_FLAGS_CONF) || !(co_flags & CO_FLAGS_PDO))
            break;

        return pdo_dispatch(ptype - CO_PDO1, data, len);
    }

    return CO_NONE;
}

uint8_t co_write(ptype_t ptype, void *data, uint8_t len) {
    uint16_t cob_id = ptype2cob(ptype);

    if (cob_id == UINT16_MAX){
#ifdef DEBUG
    	extern int canWriteErrorCounter;
    	canWriteErrorCounter++;
#endif
        return -1;
    }else if (ptype == CO_NMT || ptype >= CO_PDO1 && ptype <= CO_SDO)
        cob_id |= can_id;

    return can_write(cob_id, data, len);
}

static ptype_t cob2ptype(uint16_t cob_id) {
    switch (cob_id) {
    case 0x000: return CO_NMT;
    case 0x080: return CO_SYNC;
    case 0x7E5: return CO_LSS;
    default:
        if ((cob_id & 0x80) == 0 && (cob_id & 0x7F) == can_id) {
            // this ^^^^^^ matches slave RxPDOs and RxSDO only

            ptype_t ptype = (cob_id >> 8) + CO_PDO1 - 2;
            // ptype = CO_PDO1 + (((cob_id >> 7) + TX_FLAG(tx_flag)) >> 1) - 0x02;

            if (ptype >= CO_PDO1 && ptype <= CO_SDO)
                return ptype;
        }

        break;
    }

    return CO_NONE;
}

static uint16_t ptype2cob(ptype_t ptype) {
    switch (ptype) {
    case CO_NMT: return 0x700;
    case CO_LSS: return 0x7E4;
    default:
        if (ptype >= CO_PDO1 && ptype <= CO_SDO)
            return ((ptype - CO_PDO1 + 1) << 8) | 0x80;
            // return (((ptype + 0x02 - CO_PDO1) << 1) - TX_FLAG(tx_flag)) << 7;

        break;
    }

    return UINT16_MAX;
}

#ifdef ARCH_BYTE16
uint8_t co_mbyte = 0;

void *co_memcpy_pack(void *dest, const void *src, size_t len) {
    const uint16_t *psrc  = (const uint16_t *)src;
    uint16_t *pdest = (uint16_t *)dest;
    size_t i;

    for (i = 0; i < len; ++i, co_mbyte = !co_mbyte) {
        *pdest &= co_mbyte ? 0xFF : ~0xFF;
        *pdest |= (*psrc++ & 0xFF) << (co_mbyte ? 8 : 0);

        if (co_mbyte)
            pdest++;
    }

    return dest;
}

void *co_memcpy_unpack(void *dest, const void *src, size_t len) {
    const uint16_t *psrc  = (const uint16_t *)src;
    uint16_t *pdest = (uint16_t *)dest;
    size_t i;

    for (i = 0; i < len; ++i, co_mbyte = !co_mbyte) {
        if (co_mbyte)
            *pdest++ = *psrc++ >> 8;
        else
            *pdest++ = *psrc & 0xFF;
    }

    return dest;
}

void *co_memcpy_align(void *dest, const void *src, size_t len) {
    const uint16_t *psrc  = (const uint16_t *)src;
    uint16_t *pdest = (uint16_t *)dest;
    size_t i;
    uint8_t al_mbyte = 0, val;

    for (i = 0; i < len; ++i, co_mbyte = !co_mbyte, al_mbyte = !al_mbyte) {
        if (co_mbyte)
            val = *psrc++ >> 8;
        else
            val = *psrc & 0xFF;

        *pdest &= al_mbyte ? 0xFF : ~0xFF;
        *pdest |= val << (al_mbyte ? 8 : 0);

        if (al_mbyte)
            pdest++;
    }

    return dest;
}
#endif

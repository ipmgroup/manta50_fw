#include <string.h>
#include "canopen-lss.h"
#include "canopen-dict.h"

static enum { SSTA_VENDOR, SSTA_PRODUCT, SSTA_REV, SSTA_SN } sel_state = SSTA_VENDOR;
uint8_t node_id = 0;

static uint8_t descr_val[17] = { 5 };
static uint8_t descr_len[5] = { 1, 4, 4, 4, 4 };

void lss_init(uint8_t cid) {
    node_id = cid;
    dict_add(0x1018, descr_val, descr_len, 5, NULL, CODF_READ);
}

#ifndef EEPROM_25AA02
/*
 * default handler, can be redefined by user.
 */
__attribute__ ((weak)) uint16_t lss_store_config(uint8_t node_id)
{
    return 1; /* store configuration is not supported */
}
#endif

ptype_t lss_dispatch(void *data, uint8_t *co_flags) {
    uint8_t *p = data, cs = *p++, buf[8] = {0};
    uint32_t tmp_a, tmp_b;

#ifdef ARCH_BYTE16
    co_mbyte = 0;
#endif

    buf[0] = cs;
    switch (cs) {
    case 0x04: // Switch Mode Global
        if (*p == 1)
            *co_flags |= CO_FLAGS_CONF;
        else if (*p == 0)
            *co_flags &= ~CO_FLAGS_CONF;

        sel_state = SSTA_VENDOR;
        break;

    case 0x11: // Configure Node-ID
        if ((*co_flags & CO_FLAGS_CONF)) {
            if (*p == 0 || *p > 0x7F)
                buf[1] = 1; // Out-of-range
            else
                node_id = *p;

            co_write(CO_LSS, buf, 8);
        }
        break;

    // case 0x13: // Configure Bit Timing (Baudrate)
    //     if ((*co_flags & CO_FLAGS_CONF)) {
    //         uint8_t table_id = *p++, table_idx = *p;
    //         if (table_id != 0 || table_idx > TABLE_SIZE)
    //             buf[1] = 1; // Out-of-range
    //         else {
    //             baudrate = table[table_idx];
    //         }

    //         co_write(CO_LSS, buf, 8);
    //     }

    //     break;

    case 0x17: // Store Configuration
        if ((*co_flags & CO_FLAGS_CONF)) {
            if (node_id == 0) {
                buf[1] = 1; // Out-of-range
                buf[2] = 1; // Node-ID Out-of-range
            } else {
                uint16_t rc = lss_store_config(node_id);
                if (rc != 0) {
                    uint8_t err = rc & 0xff;
                    buf[1] = err;
                    if (err == 255)
                        buf[2] = rc >> 8;
                }
            }

            co_write(CO_LSS, buf, 8);
        }

        break;

    case 0x40: // Switch Mode Selective (Vendor-ID check)
        co_memcpy_pack(&tmp_b, p, 4);
        co_memcpy_align(&tmp_a, dict_data(0x1018, 1), 4);

        if (tmp_a == tmp_b)
            sel_state = SSTA_PRODUCT;

        break;

    case 0x41: // Switch Mode Selective (Product-ID cleck)
        co_memcpy_pack(&tmp_b, p, 4);
        co_memcpy_align(&tmp_a, dict_data(0x1018, 2), 4);

        if (sel_state >= SSTA_PRODUCT && tmp_a == tmp_b)
            sel_state = SSTA_REV;

        break;

    case 0x42: // Switch Mode Selective (Revision-Num check)
        co_memcpy_pack(&tmp_b, p, 4);
        co_memcpy_align(&tmp_a, dict_data(0x1018, 3), 4);

        if (sel_state >= SSTA_REV && tmp_a == tmp_b)
            sel_state = SSTA_SN;

        break;

    case 0x43: // Switch Mode Selective (Serian-Number check)
        co_memcpy_pack(&tmp_b, p, 4);
        co_memcpy_align(&tmp_a, dict_data(0x1018, 4), 4);

        if (sel_state >= SSTA_SN && tmp_a == tmp_b) {
            *co_flags ^= CO_FLAGS_CONF;

            buf[1] = !(*co_flags & CO_FLAGS_CONF);
            co_write(CO_LSS, buf, 8);

            sel_state = SSTA_VENDOR;
        }

        break;

    case 0x5A: // Query Vendor-ID
        if ((*co_flags & CO_FLAGS_CONF)) {
            co_memcpy_unpack(buf + 1, dict_data(0x1018, 1), 4);
            co_write(CO_LSS, buf, 8);
        }

        break;

    case 0x5B: // Query Product-ID
        if ((*co_flags & CO_FLAGS_CONF)) {
            co_memcpy_unpack(buf + 1, dict_data(0x1018, 2), 4);
            co_write(CO_LSS, buf, 8);
        }
        break;

    case 0x5C: // Query Revision-Num
        if ((*co_flags & CO_FLAGS_CONF)) {
            co_memcpy_unpack(buf + 1, dict_data(0x1018, 3), 4);
            co_write(CO_LSS, buf, 8);
        }
        break;

    case 0x5D: // Query Serial-Number
        if ((*co_flags & CO_FLAGS_CONF)) {
            co_memcpy_unpack(buf + 1, dict_data(0x1018, 4), 4);
            co_write(CO_LSS, buf, 8);
        }
        break;

    case 0x5E: // Query Node-ID
        if ((*co_flags & CO_FLAGS_CONF)) {
            buf[1] = node_id;
            co_write(CO_LSS, buf, 8);
        }
    }

    return CO_LSS;
}

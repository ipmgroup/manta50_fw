#include <stdio.h>
#include <string.h>

#include "canopen-sdo.h"
#include "canopen-dict.h"

// SDO expedited: cs il ih ss xx xx xx xx
// upload/download init request/response
//  cs:                     command/status specifier    (uint8_t)
//  il ih:                  registry object index       (uint16_t)
//  ss:                     registry objecy aubindex    (uint8_t)
//  xx xx xx xx:            data (unused bytes are filled by zeros)

// SDO segmented: cs xx xx xx xx xx xx xx
// upload/download segment request/response
//  cs:                     command/status specifier    (uint8_t)
//  xx xx xx xx xx xx xx:   data (unused bytes are filled by zeros)

static enum {
    SDO_IDLE,
    SDO_GOBJ, SDO_GOBJ_SEGM,
    SDO_SOBJ, SDO_SOBJ_SEGM
} sdo_state = SDO_IDLE;

#ifdef ARCH_BYTE16
#define eprintf(...)
#else
static char err[] = "%cOBJ %04" PRIX16 ".%02" PRIX8 " fail rs=%" PRIu8 "\n";
#define eprintf(...) printf(err, __VA_ARGS__);
#endif

static uint16_t sdo_index = 0;
static uint8_t sdo_subindex = 0, sdo_io_toggle = 0;
static uint8_t sdo_io_remain = 0, sdo_io_offset = 0;

#define SDO_CHECK(idx, sidx, op, data, len)                     \
    if ((rs = dict_check(CO_SDO, idx, sidx, op, data, len))) {  \
        eprintf(op == CO_READ ? 'G' : 'S', idx, sidx, rs); \
        if (rs == 1)                                            \
            goto abort;                                         \
        else                                                    \
            return CO_FAIL;                                     \
    }

ptype_t sdo_dispatch(void *data) {
    uint8_t *p = (uint8_t *)data, buf[8] = {0};
    uint8_t cmd = *p++, len, rs;
    void *tmp;

#ifdef CO_MASTER
    uint8_t scs = cmd >> 5;

    // NOTE: master-node code is just a reference
    switch (sdo_state) {
    case SDO_GOBJ:
        // status: AAABCCES
        //  AAA:    scs = 2 (upload init response)
        //  B:      0, not used
        //  CC:     number of unused bytes in data
        //  E:      1 - expedited, 0 - segmented
        //  S:      1, data size is indicated in "CC"
        if (scs != 2 || sdo_index != *(uint16_t *)p || sdo_subindex != p[2])
            goto abort;

        p += 3; // skip index/subindex
        if (((cmd >> 1) & 0x01) == 0x01) { // bit E is set
            sdo_state = SDO_IDLE;

            if ((len = (4 - ((cmd >> 2) & 0x03))) > dict_len(sdo_index, sdo_subindex))
                return CO_FAIL;

            memcpy(dict_data(sdo_index, sdo_subindex), p, len);
        } else {
            sdo_state = SDO_GOBJ_SEGM;

            if ((len = (uint8_t)*(uint32_t *)p) > dict_len(sdo_index, sdo_subindex))
                goto abort;

            sdo_io_remain = len;
            sdo_io_toggle = sdo_io_offset = 0;

            // command: AAABCCCC
            //  AAA:    ccs = 3 (upload segment request)
            //  B:      0/1 - toggle bit
            //  CCCC:   0, not used
            buf[0] = 0x60;
            co_write(CO_SDO, buf, 8);
        }

        break;

    case SDO_GOBJ_SEGM:
        // status: AAABCCCE
        //  AAA:    scs = 0 (upload segment response)
        //  B:      0/1 - toggle bit
        //  CCC:    number of unused bytes
        //  E:      0 - not a last segment, 1 - a last segment
        if (scs == 4) {
            sdo_state = SDO_IDLE;
            return CO_FAIL;
        }

        if (scs != 0 || ((cmd >> 4) & 0x01) != sdo_io_toggle)
            goto abort;

        if ((len = 7 - ((cmd >> 1) & 0x07)) > sdo_io_remain) {
            if ((cmd & 0x01) == 0x01) // bit E is set
                return CO_FAIL;

            goto abort;
        }

        memcpy((uint8_t *)dict_data(sdo_index, sdo_subindex) + sdo_io_offset, p, len);
        sdo_io_remain -= len;
        sdo_io_offset += len;

        if ((cmd & 0x01) == 0x01 || sdo_io_remain == 0) { // bit E is set or ...
            sdo_state = SDO_IDLE;

            if (sdo_io_remain > 0)
                return CO_FAIL;

            if ((cmd & 0x01) == 0x00) // bit E is not set, but should
                goto abort;
        } else {
            // command: AAABCCCC
            //  AAA:    ccs = 3 (upload segment request)
            //  B:      0/1 - toggle bit
            //  CCCC:   0, not used
            buf[0] = 0x60 | ((sdo_io_toggle = !sdo_io_toggle) << 4);
            co_write(CO_SDO, buf, 8);
        }

        break;

    case SDO_SOBJ:
        // status: AAACCCCC
        //  AAA:    scs = 3 (download init resonse)
        //  CCCCC:  0, not used
        if (scs == 4) {
            sdo_state = SDO_IDLE;
            return CO_FAIL;
        }

        if (scs != 3 || sdo_index != *(uint16_t *)p || sdo_subindex != p[2])
            goto abort;

        if (sdo_io_remain > 0) { // segmented download
            sdo_state = SDO_SOBJ_SEGM;

            len = sdo_io_remain > 7 ? 7 : sdo_io_remain;
            sdo_io_toggle = sdo_io_offset = 0;

            // command: AAABCCCE
            //  AAA:    ccs = 0 (download segment request)
            //  B:      0/1 - toggle bit
            //  CCC:    number of unused bytes
            //  E:      0 - not a last segment, 1 - a last segment
            buf[0] = 0x00 | (sdo_io_toggle << 4) | ((7 - len) << 1) | (sdo_io_remain - len == 0);
            memcpy(buf + 1, dict_data(sdo_index, sdo_subindex), len);
            co_write(CO_SDO, buf, 8);

            sdo_io_remain -= len;
            sdo_io_offset += len;
        } else
            sdo_state = SDO_IDLE;

        break;

    case SDO_SOBJ_SEGM:
        // status: AAABCCCC
        //  AAA:    scs = 1 (download segment response)
        //  B:      0/1 - toggle bit
        //  CCCC:   0, not used
        if (scs == 4) {
            sdo_state = SDO_IDLE;
            return CO_FAIL;
        }

        if (scs != 1 || ((cmd >> 4) & 0x01) != sdo_io_toggle)
            goto abort;

        if (sdo_io_remain > 0) {
            len = sdo_io_remain > 7 ? 7 : sdo_io_remain;
            sdo_io_toggle = !sdo_io_toggle;

            // command: AAABCCCE
            //  AAA:    ccs = 0 (download segment request)
            //  B:      0/1 - toggle bit
            //  CCC:    number of unused bytes
            //  E:      0 - not a last segment, 1 - a last segment
            buf[0] = 0x00 | (sdo_io_toggle << 4) | ((7 - len) << 1) | (sdo_io_remain - len == 0);
            memcpy(buf + 1, (uint8_t *)dict_data(sdo_index, sdo_subindex) + sdo_io_offset, len);
            co_write(CO_SDO, buf, 8);

            sdo_io_remain -= len;
            sdo_io_offset += len;
        } else
            sdo_state = SDO_IDLE;

        break;
    }
#else
    uint8_t ccs = cmd >> 5;

    switch (sdo_state) {
    case SDO_IDLE:
    default:
        if (ccs == 1 || ccs == 2) {
            sdo_index = p[0] | (p[1] << 8);
            sdo_subindex = p[2];
            p += 3;

            buf[1] = sdo_index & 0xFF;
            buf[2] = sdo_index >> 8;
            buf[3] = sdo_subindex;
        } else
            goto abort;

        if (ccs == 1) { // SOBJ
            // command: AAABCCES
            //  AAA:    ccs = 1 (download init request)
            //  B:      0, not used
            //  CC:     number of unused bytes
            //  E:      1 - expedited, 0 - segmented
            //  S:      1, data size is indicated in "CC"
            if (((cmd >> 1) & 0x01) == 0x01) {  // expedited download
                if ((len = (4 - ((cmd >> 2) & 0x03))) > dict_len(sdo_index, sdo_subindex))
                    goto abort;

                SDO_CHECK(sdo_index, sdo_subindex, CO_WRITE, p, len);
                co_memcpy_pack(dict_data(sdo_index, sdo_subindex), p, len);
            } else {                            // segmented download
                sdo_state = SDO_SOBJ_SEGM;

#ifdef ARCH_BYTE16
                co_mbyte = 0; // to co_memcpy_pack() start from bit 0, not bit 8
#endif

                co_memcpy_pack(&len, p, 4);
                if (len > dict_len(sdo_index, sdo_subindex))
                    goto abort;

                SDO_CHECK(sdo_index, sdo_subindex, CO_WRITE, NULL, len);
                sdo_io_remain = len;
                sdo_io_toggle = sdo_io_offset = 0;
            }

            // status: AAACCCCC
            //  AAA:    scs = 3 (download init resonse)
            //  CCCCC:  0, not used
            buf[0] = 0x60;
            co_write(CO_SDO, buf, 8);
        } else if (ccs == 2) { // GOBJ
            // command: AAACCCCC
            //  AAA:    ccs = 2 (upload init request)
            //  CCCCC:  0, not used
            len = dict_len(sdo_index, sdo_subindex);
            SDO_CHECK(sdo_index, sdo_subindex, CO_READ, NULL, len);

            // status: AAABCCES
            //  AAA:    scs = 2 (upload init response)
            //  B:      0, not used
            //  CC:     number of unused bytes in data
            //  E:      1 - expedited, 0 - segmented
            //  S:      1, data size is indicated in "CC"
            if (len <= 4) { // expedited upload
                buf[0] = 0x43 | ((4 - len) << 2);
                co_memcpy_unpack(buf + 4, dict_data(sdo_index, sdo_subindex), len);
            } else {        // segmented upload
                sdo_state = SDO_GOBJ_SEGM;

#ifdef ARCH_BYTE16
                co_mbyte = 0; // to co_memcpy_unpack() start from bit 0, not bit 8
#endif

                buf[0] = 0x40;
                co_memcpy_unpack(buf + 4, &len, 4);

                sdo_io_remain = len;
                sdo_io_toggle = sdo_io_offset = 0;
            }

            co_write(CO_SDO, buf, 8);
        }

        break;

    case SDO_SOBJ_SEGM:
        // command: AAABCCCE
        //  AAA:    ccs = 0 (download segment request)
        //  B:      0/1 - toggle bit
        //  CCC:    number of unused bytes
        //  E:      0 - not a last segment, 1 - a last segment
        if (ccs != 0 || ((cmd >> 4) & 0x01) != sdo_io_toggle)
            goto abort;

        // if there are more bytes than we expected
        if ((len = 7 - ((cmd >> 1) & 0x07)) > sdo_io_remain)
            goto abort;

#ifdef ARCH_BYTE16
        tmp = (uint16_t *)dict_data(sdo_index, sdo_subindex) + sdo_io_offset / 2;

        if (sdo_io_offset % 2 != 0)
            co_mbyte = !co_mbyte;
#else
        tmp = (uint8_t *)dict_data(sdo_index, sdo_subindex) + sdo_io_offset;
#endif

        co_memcpy_pack(tmp, p, len);
        sdo_io_remain -= len;
        sdo_io_offset += len;

        if ((cmd & 0x01) == 0x01 || sdo_io_remain == 0) { // bit E is set
            sdo_state = SDO_IDLE;

            if (sdo_io_remain > 0 || (cmd & 0x01) == 0x00)
                goto abort;
        }

        // status: AAABCCCC
        //  AAA:    scs = 1 (download segment response)
        //  B:      0/1 - toggle bit
        //  CCCC:   0, not used
        buf[0] = 0x20 | (sdo_io_toggle << 4);
        co_write(CO_SDO, buf, 8);
        sdo_io_toggle = !sdo_io_toggle;
        break;

    case SDO_GOBJ_SEGM:
        // command: AAABCCCC
        //  AAA:    ccs = 3 (upload segment request)
        //  B:      0/1 - toggle bit
        //  CCCC:   0, not used
        if (ccs != 3 || ((cmd >> 4) & 0x01) != sdo_io_toggle)
            goto abort;

        len = sdo_io_remain > 7 ? 7 : sdo_io_remain;

#ifdef ARCH_BYTE16
        tmp = (uint16_t *)dict_data(sdo_index, sdo_subindex) + sdo_io_offset / 2;

        if (sdo_io_offset % 2 != 0)
            co_mbyte = !co_mbyte;
#else
        tmp = (uint8_t *)dict_data(sdo_index, sdo_subindex) + sdo_io_offset;
#endif

        co_memcpy_unpack(buf + 1, tmp, len);
        sdo_io_remain -= len;
        sdo_io_offset += len;

        if (sdo_io_remain == 0)
            sdo_state = SDO_IDLE;

        // status: AAABCCCE
        //  AAA:    scs = 0 (upload segment response)
        //  B:      0/1 - toggle bit
        //  CCC:    number of unused bytes
        //  E:      0 - not a last segment, 1 - a last segment
        buf[0] = 0x00 | (sdo_io_toggle << 4) | ((7 - len) << 1) | (sdo_io_remain == 0);
        co_write(CO_SDO, buf, 8);
        sdo_io_toggle = !sdo_io_toggle;
        break;
    }
#endif

    return sdo_state == SDO_IDLE ? CO_SDO : CO_NONE;

abort:
    // command: AAACCCCC
    //  AAA:    ccs/scs = 4 (abort)
    //  CCCCC:  0, not used
    buf[0] = 0x80;

    buf[1] = sdo_index & 0xFF;
    buf[2] = sdo_index >> 8;
    buf[3] = sdo_subindex;

    co_write(CO_SDO, buf, 8);
    sdo_state = SDO_IDLE;
    return CO_FAIL;
}

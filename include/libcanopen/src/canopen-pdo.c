#include <stdio.h>
#include <string.h>

#include "canopen-dict.h"
#include "canopen-pdo.h"

#ifdef ARCH_BYTE16
#define eprintf(...)
#else
static char err[] = "%cxPDO%" PRIu8 ":%" PRIu8 " fail\n";
#define eprintf(...) printf(err, __VA_ARGS__)
#endif

static uint8_t sync_cnt_tx[PDO_NUM], sync_cnt_rx[PDO_NUM];
static uint8_t rx_buf[PDO_NUM][8];
static int8_t pdo_idx[PDO_MAXNUM];

static uint8_t map_len[5] = { 1, 4, 4, 4, 4 };
static uint8_t conf_len[3] = { 0, 0, 1 };

static void pdo_exec(uint8_t pdo_n);

int pdo_init(pdo_map_t *pdo_map, uint8_t pdo_num) {
    int i;

    for (i = 0; i < PDO_NUM; i++)
        pdo_idx[i] = -1;

    if (pdo_num > PDO_NUM)
        return -2;

    if (pdo_map == NULL)
        return 0;

    for (i = 0; i < pdo_num; i++) {
        uint8_t pdo_n = pdo_map[i].ptype - CO_PDO1;

        if (pdo_n >= PDO_MAXNUM)
            return -1;

        pdo_idx[pdo_n] = i;
        sync_cnt_rx[i] = 0;
        sync_cnt_tx[i] = 0;

        dict_add(0x1600 + pdo_n, pdo_map[i].rx_map,  map_len,  5, NULL, CODF_RW);
        dict_add(0x1A00 + pdo_n, pdo_map[i].tx_map,  map_len,  5, NULL, CODF_RW);
        dict_add(0x1400 + pdo_n, &pdo_map[i].rx_conf, conf_len, 3, NULL, CODF_RW);
        dict_add(0x1800 + pdo_n, &pdo_map[i].tx_conf, conf_len, 3, NULL, CODF_RW);
    }
    return 0;
}

ptype_t pdo_dispatch(uint8_t pdo_n, void *data, uint8_t len) {
    int8_t idx;
    uint16_t mode = 0;

    if (pdo_n >= PDO_MAXNUM || (idx = pdo_idx[pdo_n]) < 0)
        return CO_NONE;

    memcpy(rx_buf[idx], data, len);

    co_memcpy_align(&mode, dict_data(0x1400 + pdo_n, 2), 1);
    mode = mode & 0xFF;
    if (mode == 255) // mode == async PDOrx
        pdo_exec(pdo_n);

    return (ptype_t)(CO_PDO1 + pdo_n);
}

ptype_t sync_dispatch(void) {
    int8_t idx;
    uint8_t pdo_n;
	uint16_t mode = 0;

#ifdef DEBUG
	extern long mainIsrCounter;
	extern long syncTimes[12];
	extern int nextSyncPos;
	extern long maxSyncTime;
	extern long diff;
	extern long averageSyncDiff;

	if(nextSyncPos < 0){
		nextSyncPos = 1;
		syncTimes[0] = mainIsrCounter;
	}else{
		syncTimes[nextSyncPos] = mainIsrCounter;
		diff = syncTimes[nextSyncPos] - syncTimes[(nextSyncPos + 11) % 12];
		nextSyncPos = (nextSyncPos+1) % 12;
		if(diff > maxSyncTime){
			maxSyncTime = diff;
		}
	}
#endif

    for (pdo_n = 0; pdo_n < PDO_MAXNUM; pdo_n++) {
        if ((idx = pdo_idx[pdo_n]) < 0){
            continue;
        }

        co_memcpy_align(&mode, dict_data(0x1400 + pdo_n, 2), 1);
        mode = mode & 0xFF; //FIXME: remove after testing
        if (mode < 240 && mode > 0) { // sync PDOrx (after each N-th SYNC)
            if (sync_cnt_rx[idx]-- == 0) {
                pdo_exec(pdo_n);
                sync_cnt_rx[idx] = mode - 1;
            }
        }

        co_memcpy_align(&mode, dict_data(0x1800 + pdo_n, 2), 1);
        mode = mode & 0xFF; //FIXME: remove after testing
        if (mode < 240 && mode > 0) { // sync PDOtx (after each N-th SYNC)
            if (sync_cnt_tx[idx]-- == 0) {
                pdo_write(pdo_n);
                sync_cnt_tx[idx] = mode - 1;
            }
        }
    }

    return CO_SYNC;
}

void pdo_write(uint8_t pdo_n) {
    uint8_t buf[8] = {0}, len = 0;

    if (pdo_n >= PDO_MAXNUM || pdo_idx[pdo_n] < 0)
        return;

    uint8_t i;
    uint16_t n = 0;
    co_memcpy_align(&n, dict_data(0x1A00 + pdo_n, 0), 1);
    n = n & 0xFF; //FIXME: remove after testing
    for (i = 0; i < n; i++) {
        uint32_t mapping;
        co_memcpy_align(&mapping, dict_data(0x1A00 + pdo_n, i + 1), 4);

        uint16_t d_idx = (mapping >> 16) & 0xFFFF;
        uint8_t d_sidx = (mapping >> 8) & 0xFF;
        uint8_t d_len = (mapping & 0xFF) >> 3;

        if (dict_len(d_idx, d_sidx) < d_len ||
            dict_check(CO_PDO1 + pdo_n, d_idx, d_sidx, CO_READ, NULL, d_len)) {
            eprintf('T', pdo_n, i);
            return;
        }

        void *p = dict_data(d_idx, d_sidx);
        if (p == NULL || len + d_len > 8)
            return;

        co_memcpy_unpack(buf + len, p, d_len);
        len += d_len;
    }

    co_write((ptype_t)(CO_PDO1 + pdo_n), buf, len);
}

static void pdo_exec(uint8_t pdo_n) {
    int8_t idx;
    uint8_t len = 0;

    if (pdo_n >= PDO_MAXNUM || (idx = pdo_idx[pdo_n]) < 0)
        return;

    uint8_t i;
    uint16_t n = 0;
    co_memcpy_align(&n, dict_data(0x1600 + pdo_n, 0), 1);
    n = n & 0xFF;  //FIXME: remove after testing
    for (i = 0; i < n; i++) {
        uint32_t mapping;
        co_memcpy_align(&mapping, dict_data(0x1600 + pdo_n, i + 1), 4);

        uint16_t d_idx = (mapping >> 16) & 0xFFFF;
        uint8_t d_sidx = (mapping >> 8) & 0xFF;
        uint8_t d_len = (mapping & 0xFF) >> 3;

        if (dict_len(d_idx, d_sidx) < d_len ||
            dict_check(CO_PDO1 + pdo_n, d_idx, d_sidx, CO_WRITE, rx_buf[idx] + len, d_len)) {
            eprintf('R', pdo_n, i);
            return;
        }

        void *p = dict_data(d_idx, d_sidx);
        if (p == NULL || len + d_len > 8)
            return;

        co_memcpy_pack(p, rx_buf[idx] + len, d_len);
        len += d_len;
    }
}

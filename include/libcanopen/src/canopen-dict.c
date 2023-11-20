#include <string.h>
#include "canopen-dict.h"

#define DICT_MAX_LEN 64

typedef struct {
    void *data;
    uint8_t *len;
    uint16_t index;
    uint8_t num, flags;
    dictcb_t cb;
} dict_t;

static dict_t dict[DICT_MAX_LEN];
static uint16_t dict_entries = 0;

uint8_t dict_byte = 1, dict_word = 2, dict_dword = 4;

void dict_add(uint16_t index, void *data, uint8_t *len, uint8_t num, dictcb_t cb, uint8_t flags) {
    if (dict_entries >= DICT_MAX_LEN)
        return;

    dict[dict_entries].index = index;
    dict[dict_entries].data  = data;
    dict[dict_entries].len   = len;
    dict[dict_entries].num   = num;
    dict[dict_entries].cb    = cb;
    dict[dict_entries].flags = flags;
    dict_entries++;
}

void *dict_data(uint16_t index, uint8_t subindex) {
    uint16_t i;

    for (i = 0; i < dict_entries; ++i)
        if (index == dict[i].index) {
            if (subindex >= dict[i].num)
                return NULL;

            uint8_t j, offset = 0;
            for (j = 0; j < subindex; ++j)
                offset += dict[i].len[j];

#ifdef ARCH_BYTE16
            co_mbyte = offset % 2 != 0;
            return (uint16_t *)dict[i].data + offset / 2;
#else
            return (uint8_t *)dict[i].data + offset;
#endif
        }

    return NULL;
}

uint8_t dict_len(uint16_t index, uint8_t subindex) {
    uint16_t i;

    for (i = 0; i < dict_entries; ++i)
        if (index == dict[i].index) {
            if (subindex < (dict[i].num & 0xFF))
                return (dict[i].len[subindex]) & 0xFF;

            break;
        }

    return 0;
}

uint8_t dict_check(ptype_t ptype, uint16_t index, uint8_t subindex, dictop_t op, void *data, size_t len) {
	uint16_t i = 0;

    for (i = 0; i < dict_entries; ++i)
        if (index == dict[i].index) {
            if (subindex < (dict[i].num & 0xFF)) {
                if (op == CO_READ && !(dict[i].flags & CODF_READ)){
                	return 1;
                }

                if (op == CO_WRITE && !(dict[i].flags & CODF_WRITE)){
                    return 1;
                }

                if (dict[i].cb != NULL){
                	return ((*dict[i].cb)(ptype, index, subindex, op, data, len)) & 0xFF;
                }

                return 0;
            }

            break;
        }

    return 1;
}

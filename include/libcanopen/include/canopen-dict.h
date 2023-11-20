#ifndef __CANOPEN_DICT_H__
#define __CANOPEN_DICT_H__

#include <stdint.h>
#include <string.h> // size_t
#include "canopen.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CODF_READ   (1 << 0)
#define CODF_WRITE (1 << 1)
#define CODF_RW (CODF_READ | CODF_WRITE)

typedef enum { CO_READ, CO_WRITE } dictop_t;
typedef uint8_t (*dictcb_t)(ptype_t ptype, uint16_t idx, uint8_t sidx, dictop_t op, void *data, size_t len);

extern uint8_t dict_byte, dict_word, dict_dword;
void dict_add(uint16_t index, void *data, uint8_t *len, uint8_t num, dictcb_t cb, uint8_t flags);

void *dict_data(uint16_t index, uint8_t subindex);
uint8_t dict_len(uint16_t index, uint8_t subindex);

uint8_t dict_check(ptype_t ptype, uint16_t index, uint8_t subindex, dictop_t op, void *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif

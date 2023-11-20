#include <arch/arch.h>
#include "canopen-lss.h"
#include "canopen-dict.h"

static uint8_t node_id = 0;

void nmt_init(uint8_t cid) {
    uint8_t zero = 0;
    node_id = cid;
    co_write(CO_NMT, &zero, 1);
}

ptype_t nmt_dispatch(void *data, uint8_t *co_flags) {
    uint8_t *p = data, cs = *p++, cid = *p++;

    if (cid == node_id || cid == 0) switch (cs) {
    case 0x01: // Start Node (enable PDO and SDO, Operational Mode)
        *co_flags |= CO_FLAGS_PDO | CO_FLAGS_SDO;
        break;

    case 0x02: // Stop Node (disable PDO and SDO, Stopped Node)
        *co_flags &= ~(CO_FLAGS_PDO | CO_FLAGS_SDO);
        break;

    case 0x80: // Enter Pre-Operational mode (disable PDO, enable SDO)
        *co_flags &= ~CO_FLAGS_PDO;
        *co_flags |= CO_FLAGS_SDO;
        break;

    case 0x81: // Reset Device
        soft_restart();
        break;
    }

    return CO_NMT;
}

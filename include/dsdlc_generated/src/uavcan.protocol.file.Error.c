

#define CANARD_DSDLC_INTERNAL
#include <uavcan.protocol.file.Error.h>

#include <string.h>

#ifdef CANARD_DSDLC_TEST_BUILD
#include <test_helpers.h>
#endif

uint32_t uavcan_protocol_file_Error_encode(struct uavcan_protocol_file_Error* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
) {
    uint32_t bit_ofs = 0;
    memset(buffer, 0, UAVCAN_PROTOCOL_FILE_ERROR_MAX_SIZE);
    _uavcan_protocol_file_Error_encode(buffer, &bit_ofs, msg, 
#if CANARD_ENABLE_TAO_OPTION
    tao
#else
    true
#endif
    );
    return ((bit_ofs+7)/8);
}

/*
  return true if the decode is invalid
 */
bool uavcan_protocol_file_Error_decode(const CanardRxTransfer* transfer, struct uavcan_protocol_file_Error* msg) {
    uint32_t bit_ofs = 0;
    _uavcan_protocol_file_Error_decode(transfer, &bit_ofs, msg, 
#if CANARD_ENABLE_TAO_OPTION
    transfer->tao
#else
    true
#endif
    );

    const uint32_t byte_len = (bit_ofs+7U)/8U;
#if CANARD_ENABLE_TAO_OPTION
    // if this could be CANFD then the dlc could indicating more bytes than
    // we actually have
    if (!transfer->tao) {
        return byte_len > transfer->payload_len;
    }
#endif
    return byte_len != transfer->payload_len;
}

#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_protocol_file_Error sample_uavcan_protocol_file_Error_msg(void) {

    struct uavcan_protocol_file_Error msg;






    msg.value = (int16_t)random_bitlen_signed_val(16);





    return msg;

}
#endif

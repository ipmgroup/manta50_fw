/*
 * This demo application is distributed under the terms of CC0 (public domain dedication).
 * More info: https://creativecommons.org/publicdomain/zero/1.0/
 */

// This is needed to enable necessary declarations in sys/
#ifndef _GNU_SOURCE
# define _GNU_SOURCE
#endif

#include <canard.h>
#include <canard_c2000.h>      // CAN backend driver for C2000 DSP

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include <errno.h>

#include <can.h>

// XXX TODO
#define GIT_HASH 0x00c0ffee

// XXX TODO Save some memory...
#define fprintf(...) (void)(0)
#define printf(...) (void)(0)
#define puts(...) (void)(0)
#ifdef assert
#undef assert
#define assert(...) (void)(0)
#endif

// Fake target RPM
volatile int32_t target_rpm;

/*
 * Application constants
 */
#define APP_VERSION_MAJOR                                           1
#define APP_VERSION_MINOR                                           0
#define APP_NODE_NAME                                               "org.uavcan.libcanard.demo"

/*
 * Some useful constants defined by the UAVCAN specification.
 * Data type signature values can be easily obtained with the script show_data_type_info.py
 */
#ifdef UAVCAN_DYNAMIC_NODE
#define UAVCAN_NODE_ID_ALLOCATION_DATA_TYPE_ID                      1
#define UAVCAN_NODE_ID_ALLOCATION_DATA_TYPE_SIGNATURE               0x0b2a812620a11d40
#define UAVCAN_NODE_ID_ALLOCATION_RANDOM_TIMEOUT_RANGE_USEC         400000UL
#define UAVCAN_NODE_ID_ALLOCATION_REQUEST_DELAY_OFFSET_USEC         600000UL
#endif

#define UAVCAN_NODE_STATUS_MESSAGE_SIZE                             7
#define UAVCAN_NODE_STATUS_DATA_TYPE_ID                             341
#define UAVCAN_NODE_STATUS_DATA_TYPE_SIGNATURE                      0x0f0868d0c1a7c6f1

#define UAVCAN_NODE_HEALTH_OK                                       0
#define UAVCAN_NODE_HEALTH_WARNING                                  1
#define UAVCAN_NODE_HEALTH_ERROR                                    2
#define UAVCAN_NODE_HEALTH_CRITICAL                                 3

#define UAVCAN_NODE_MODE_OPERATIONAL                                0
#define UAVCAN_NODE_MODE_INITIALIZATION                             1

// XXX TODO FIXME Investigate why the original size corrupts library/program structures/memory.
// #define UAVCAN_GET_NODE_INFO_RESPONSE_MAX_SIZE                      ((3015 + 7) / 8)
#define UAVCAN_GET_NODE_INFO_RESPONSE_MAX_SIZE                      128
#define UAVCAN_GET_NODE_INFO_DATA_TYPE_SIGNATURE                    0xee468a8121c46a9e
#define UAVCAN_GET_NODE_INFO_DATA_TYPE_ID                           1

#define UAVCAN_ESC_RPM_COMMAND_DATA_TYPE_SIGNATURE                  0xce0f9f621cf7e70b
#define UAVCAN_ESC_RPM_COMMAND_DATA_TYPE_ID                         1031

#define UNIQUE_ID_LENGTH_BYTES                                      16


/*
 * Library instance.
 * In simple applications it makes sense to make it static, but it is not necessary.
 */
static CanardInstance canard;                       ///< The library instance
static uint8_t canard_memory_pool[512];            ///< Arena for memory allocation, used by the library

#ifdef UAVCAN_DYNAMIC_NODE
/*
 * Variables used for dynamic node ID allocation.
 * RTFM at http://uavcan.org/Specification/6._Application_level_functions/#dynamic-node-id-allocation
 */
static uint64_t send_next_node_id_allocation_request_at;    ///< When the next node ID allocation request should be sent
static uint8_t node_id_allocation_unique_id_offset;         ///< Depends on the stage of the next request
#endif

/*
 * Node status variables
 */
static uint8_t node_health = UAVCAN_NODE_HEALTH_OK;
static uint8_t node_mode   = UAVCAN_NODE_MODE_INITIALIZATION;


uint64_t getMonotonicTimestampUSec(void)
{
  // XXX TODO
  static uint64_t t = 0;
  return t += 10;
}


/**
 * Returns a pseudo random float in the range [0, 1].
 */
float getRandomFloat(void)
{
    static bool initialized = false;
    if (!initialized)                   // This is not thread safe, but a race condition here is not harmful.
    {
        initialized = true;
        srand((unsigned)time(NULL));
    }
    // coverity[dont_call]
    return (float)rand() / (float)RAND_MAX;
}


/**
 * This function uses a mock unique ID, this is not allowed in real applications!
 */
void readUniqueID(uint8_t* out_uid)
{
    uint8_t i;
    for (i = 0; i < UNIQUE_ID_LENGTH_BYTES; i++)
    {
        out_uid[i] = i;
    }
}


void makeNodeStatusMessage(uint8_t buffer[UAVCAN_NODE_STATUS_MESSAGE_SIZE])
{
    memset(buffer, 0, UAVCAN_NODE_STATUS_MESSAGE_SIZE);

    static uint32_t started_at_sec = 0;
    if (started_at_sec == 0)
    {
        started_at_sec = (uint32_t)(getMonotonicTimestampUSec() / 1000000U);
    }

    const uint32_t uptime_sec = (uint32_t)((getMonotonicTimestampUSec() / 1000000U) - started_at_sec);

    /*
     * Here we're using the helper for demonstrational purposes; in this simple case it could be preferred to
     * encode the values manually.
     */
    canardEncodeScalar(buffer,  0, 32, &uptime_sec);
    canardEncodeScalar(buffer, 32,  2, &node_health);
    canardEncodeScalar(buffer, 34,  3, &node_mode);
}


/**
 * This callback is invoked by the library when a new message or request or response is received.
 */
static void onTransferReceived(CanardInstance* ins,
                               CanardRxTransfer* transfer)
{
#ifdef UAVCAN_DYNAMIC_NODE
    /*
     * Dynamic node ID allocation protocol.
     * Taking this branch only if we don't have a node ID, ignoring otherwise.
     */
    if ((canardGetLocalNodeID(ins) == CANARD_BROADCAST_NODE_ID) &&
        (transfer->transfer_type == CanardTransferTypeBroadcast) &&
        (transfer->data_type_id == UAVCAN_NODE_ID_ALLOCATION_DATA_TYPE_ID))
    {
        // Rule C - updating the randomized time interval
        send_next_node_id_allocation_request_at =
            getMonotonicTimestampUSec() + UAVCAN_NODE_ID_ALLOCATION_REQUEST_DELAY_OFFSET_USEC +
            (uint64_t)(getRandomFloat() * UAVCAN_NODE_ID_ALLOCATION_RANDOM_TIMEOUT_RANGE_USEC);

        if (transfer->source_node_id == CANARD_BROADCAST_NODE_ID)
        {
            puts("Allocation request from another allocatee");
            node_id_allocation_unique_id_offset = 0;
            return;
        }

        // Copying the unique ID from the message
        static const unsigned UniqueIDBitOffset = 8;
        uint8_t received_unique_id[UNIQUE_ID_LENGTH_BYTES];
        uint8_t received_unique_id_len = 0;
        for (; received_unique_id_len < (transfer->payload_len - (UniqueIDBitOffset / 8U)); received_unique_id_len++)
        {
            assert(received_unique_id_len < UNIQUE_ID_LENGTH_BYTES);
            const uint8_t bit_offset = (uint8_t)(UniqueIDBitOffset + received_unique_id_len * 8U);
            (void) canardDecodeScalar(transfer, bit_offset, 8, false, &received_unique_id[received_unique_id_len]);
        }

        // Obtaining the local unique ID
        uint8_t my_unique_id[UNIQUE_ID_LENGTH_BYTES];
        readUniqueID(my_unique_id);

        // Matching the received UID against the local one
        if (memcmp(received_unique_id, my_unique_id, received_unique_id_len) != 0)
        {
            printf("Mismatching allocation response from %d:", transfer->source_node_id);
            int i;
            for (i = 0; i < received_unique_id_len; i++)
            {
                printf(" %02x/%02x", received_unique_id[i], my_unique_id[i]);
            }
            puts("");
            node_id_allocation_unique_id_offset = 0;
            return;         // No match, return
        }

        if (received_unique_id_len < UNIQUE_ID_LENGTH_BYTES)
        {
            // The allocator has confirmed part of unique ID, switching to the next stage and updating the timeout.
            node_id_allocation_unique_id_offset = received_unique_id_len;
            send_next_node_id_allocation_request_at -= UAVCAN_NODE_ID_ALLOCATION_REQUEST_DELAY_OFFSET_USEC;

            printf("Matching allocation response from %d offset %d\n",
                   transfer->source_node_id, node_id_allocation_unique_id_offset);
        }
        else
        {
            // Allocation complete - copying the allocated node ID from the message
            uint8_t allocated_node_id = 0;
            (void) canardDecodeScalar(transfer, 0, 7, false, &allocated_node_id);
            assert(allocated_node_id <= 127);

            canardSetLocalNodeID(ins, allocated_node_id);
            printf("Node ID %d allocated by %d\n", allocated_node_id, transfer->source_node_id);
        }
    }
#endif

    if ((transfer->transfer_type == CanardTransferTypeRequest) &&
        (transfer->data_type_id == UAVCAN_GET_NODE_INFO_DATA_TYPE_ID))
    {
        printf("GetNodeInfo request from %d\n", transfer->source_node_id);

        uint8_t buffer[UAVCAN_GET_NODE_INFO_RESPONSE_MAX_SIZE];
        memset(buffer, 0, UAVCAN_GET_NODE_INFO_RESPONSE_MAX_SIZE);

        // NodeStatus
        makeNodeStatusMessage(buffer);

        // SoftwareVersion
        buffer[7] = APP_VERSION_MAJOR;
        buffer[8] = APP_VERSION_MINOR;
        buffer[9] = 1;                          // Optional field flags, VCS commit is set
        uint32_t u32 = GIT_HASH;
        canardEncodeScalar(buffer, 80, 32, &u32);
        // Image CRC skipped

        // HardwareVersion
        // Major skipped
        // Minor skipped
        readUniqueID(&buffer[24]);
        // Certificate of authenticity skipped

        // Name
        const size_t name_len = strlen(APP_NODE_NAME);
        memcpy(&buffer[41], APP_NODE_NAME, name_len);

        const size_t total_size = 41 + name_len;

        /*
         * Transmitting; in this case we don't have to release the payload because it's empty anyway.
         */
        const int resp_res = canardRequestOrRespond(ins,
                                                    transfer->source_node_id,
                                                    UAVCAN_GET_NODE_INFO_DATA_TYPE_SIGNATURE,
                                                    UAVCAN_GET_NODE_INFO_DATA_TYPE_ID,
                                                    &transfer->transfer_id,
                                                    transfer->priority,
                                                    CanardResponse,
                                                    &buffer[0],
                                                    (uint16_t)total_size);
        if (resp_res <= 0)
        {
            (void)fprintf(stderr, "Could not respond to GetNodeInfo; error %d\n", resp_res);
        }
    }

    if ((transfer->transfer_type == CanardTransferTypeBroadcast) &&
            (transfer->data_type_id == UAVCAN_ESC_RPM_COMMAND_DATA_TYPE_ID))
    {
        // int18[<=20] rpm
        // use just the first (if more)
        uint8_t tmp;
        uint32_t rpm;

        // NOTE Below stuff could be easily implemented without heavy canardDecodeScalar()
        (void) canardDecodeScalar(transfer, 0, 8, false, &tmp);
        rpm = tmp;
        (void) canardDecodeScalar(transfer, 8, 8, false, &tmp);
        rpm |= (uint16_t)tmp << 8;
        (void) canardDecodeScalar(transfer, 16, 2, false, &tmp);
        rpm |= (uint32_t)(tmp & 0x01) << 16;
        target_rpm = tmp & 0x02 ? rpm | 0xfffe0000 : rpm;
    }
}


/**
 * This callback is invoked by the library when it detects beginning of a new transfer on the bus that can be received
 * by the local node.
 * If the callback returns true, the library will receive the transfer.
 * If the callback returns false, the library will ignore the transfer.
 * All transfers that are addressed to other nodes are always ignored.
 */
static bool shouldAcceptTransfer(const CanardInstance* ins,
                                 uint64_t* out_data_type_signature,
                                 uint16_t data_type_id,
                                 CanardTransferType transfer_type,
                                 uint8_t source_node_id)
{
    (void)source_node_id;

#ifdef UAVCAN_DYNAMIC_NODE
    if (canardGetLocalNodeID(ins) == CANARD_BROADCAST_NODE_ID)
    {
        /*
         * If we're in the process of allocation of dynamic node ID, accept only relevant transfers.
         */
        if ((transfer_type == CanardTransferTypeBroadcast) &&
            (data_type_id == UAVCAN_NODE_ID_ALLOCATION_DATA_TYPE_ID))
        {
            *out_data_type_signature = UAVCAN_NODE_ID_ALLOCATION_DATA_TYPE_SIGNATURE;
            return true;
        }
    }
    else
#endif
    {
        if ((transfer_type == CanardTransferTypeRequest) &&
            (data_type_id == UAVCAN_GET_NODE_INFO_DATA_TYPE_ID))
        {
            *out_data_type_signature = UAVCAN_GET_NODE_INFO_DATA_TYPE_SIGNATURE;
            return true;
        }
        else if ((transfer_type == CanardTransferTypeBroadcast) &&
                (data_type_id == UAVCAN_ESC_RPM_COMMAND_DATA_TYPE_ID))
        {
            *out_data_type_signature = UAVCAN_ESC_RPM_COMMAND_DATA_TYPE_SIGNATURE;
            return true;
        }
    }

    return false;
}


/**
 * This function is called at 1 Hz rate from the main loop.
 */
void process1HzTasks(uint64_t timestamp_usec)
{
    /*
     * Purging transfers that are no longer transmitted. This will occasionally free up some memory.
     */
    canardCleanupStaleTransfers(&canard, timestamp_usec);

    /*
     * Printing the memory usage statistics.
     */
    {
        const CanardPoolAllocatorStatistics stats = canardGetPoolAllocatorStatistics(&canard);
        const unsigned peak_percent = 100U * stats.peak_usage_blocks / stats.capacity_blocks;

        printf("Memory pool stats: capacity %u blocks, usage %u blocks, peak usage %u blocks (%u%%)\n",
               stats.capacity_blocks, stats.current_usage_blocks, stats.peak_usage_blocks, peak_percent);

        /*
         * The recommended way to establish the minimal size of the memory pool is to stress-test the application and
         * record the worst case memory usage.
         */
        if (peak_percent > 70)
        {
            puts("WARNING: ENLARGE MEMORY POOL");
        }
    }

    /*
     * Transmitting the node status message periodically.
     */
    {
        uint8_t buffer[UAVCAN_NODE_STATUS_MESSAGE_SIZE];
        makeNodeStatusMessage(buffer);

        static uint8_t transfer_id;

        const int bc_res = canardBroadcast(&canard, UAVCAN_NODE_STATUS_DATA_TYPE_SIGNATURE,
                                           UAVCAN_NODE_STATUS_DATA_TYPE_ID, &transfer_id, CANARD_TRANSFER_PRIORITY_LOW,
                                           buffer, UAVCAN_NODE_STATUS_MESSAGE_SIZE);
        if (bc_res <= 0)
        {
            (void)fprintf(stderr, "Could not broadcast node status; error %d\n", bc_res);
        }
    }

    node_mode = UAVCAN_NODE_MODE_OPERATIONAL;
}

/**
 * Transmits all frames from the TX queue, receives up to one frame.
 */
void processTxRxOnce(int timeout_msec)
{
    // Transmitting
    const CanardCANFrame* txf;
    for (txf = NULL; (txf = canardPeekTxQueue(&canard)) != NULL;)
    {
        // XXX TODO It helps CAN messages reordering! Investigate mcp2515 send priorities!
        uint64_t c;
        for (c = 1000LL * 32; c; c--)
        {}

        const int tx_res = canardC2000Transmit(txf);

        if (tx_res > 0)    // Success - just drop the frame
        {
            canardPopTxQueue(&canard);
        }
        else                    // Timeout - just exit and try again later
        {
            break;
        }
    }

    // Receiving
    CanardCANFrame rx_frame;
    const uint64_t timestamp = getMonotonicTimestampUSec();
    const int rx_res = canardC2000Receive(&rx_frame);
    if (rx_res < 0)             // Failure - report
    {
        (void)fprintf(stderr, "Receive error %d, errno '%s'\n", rx_res, strerror(errno));
    }
    else if (rx_res > 0)        // Success - process the frame
    {
        canardHandleRxFrame(&canard, &rx_frame, timestamp);
    }
    else
    {
        ;                       // Timeout - nothing to do
    }
}


int canard_main()
{
    /*
     * Initializing the CAN backend driver
     */
    canardC2000Init(CAN_500KBPS);

    /*
     * Initializing the Libcanard instance.
     */
    canardInit(&canard, canard_memory_pool, sizeof(canard_memory_pool), onTransferReceived, shouldAcceptTransfer, NULL);

#ifdef UAVCAN_DYNAMIC_NODE
    /*
     * Performing the dynamic node ID allocation procedure.
     */
    static const uint8_t PreferredNodeID = CANARD_BROADCAST_NODE_ID;    ///< This can be made configurable, obviously

    node_id_allocation_unique_id_offset = 0;

    uint8_t node_id_allocation_transfer_id = 0;

    while (canardGetLocalNodeID(&canard) == CANARD_BROADCAST_NODE_ID)
    {
        puts("Waiting for dynamic node ID allocation...");

        send_next_node_id_allocation_request_at =
            getMonotonicTimestampUSec() + UAVCAN_NODE_ID_ALLOCATION_REQUEST_DELAY_OFFSET_USEC +
            (uint64_t)(getRandomFloat() * UAVCAN_NODE_ID_ALLOCATION_RANDOM_TIMEOUT_RANGE_USEC);

        while ((getMonotonicTimestampUSec() < send_next_node_id_allocation_request_at) &&
               (canardGetLocalNodeID(&canard) == CANARD_BROADCAST_NODE_ID))
        {
            processTxRxOnce(1);
        }

        if (canardGetLocalNodeID(&canard) != CANARD_BROADCAST_NODE_ID)
        {
            break;
        }

        // Structure of the request is documented in the DSDL definition
        // See http://uavcan.org/Specification/6._Application_level_functions/#dynamic-node-id-allocation
        uint8_t allocation_request[CANARD_CAN_FRAME_MAX_DATA_LEN - 1];
        allocation_request[0] = PreferredNodeID << 1;

        if (node_id_allocation_unique_id_offset == 0)
        {
            allocation_request[0] |= 1;     // First part of unique ID
        }

        uint8_t my_unique_id[UNIQUE_ID_LENGTH_BYTES];
        readUniqueID(my_unique_id);

        static const uint8_t MaxLenOfUniqueIDInRequest = 6;
        uint8_t uid_size = (uint8_t)(UNIQUE_ID_LENGTH_BYTES - node_id_allocation_unique_id_offset);
        if (uid_size > MaxLenOfUniqueIDInRequest)
        {
            uid_size = MaxLenOfUniqueIDInRequest;
        }

        // Paranoia time
        assert(node_id_allocation_unique_id_offset < UNIQUE_ID_LENGTH_BYTES);
        assert(uid_size <= MaxLenOfUniqueIDInRequest);
        assert(uid_size > 0);
        assert((uid_size + node_id_allocation_unique_id_offset) <= UNIQUE_ID_LENGTH_BYTES);

        memmove(&allocation_request[1], &my_unique_id[node_id_allocation_unique_id_offset], uid_size);

        // Broadcasting the request
        const int bcast_res = canardBroadcast(&canard,
                                              UAVCAN_NODE_ID_ALLOCATION_DATA_TYPE_SIGNATURE,
                                              UAVCAN_NODE_ID_ALLOCATION_DATA_TYPE_ID,
                                              &node_id_allocation_transfer_id,
                                              CANARD_TRANSFER_PRIORITY_LOW,
                                              &allocation_request[0],
                                              (uint16_t) (uid_size + 1));
        if (bcast_res < 0)
        {
            (void)fprintf(stderr, "Could not broadcast dynamic node ID allocation request; error %d\n", bcast_res);
        }

        // Preparing for timeout; if response is received, this value will be updated from the callback.
        node_id_allocation_unique_id_offset = 0;
    }

    printf("Dynamic node ID allocation complete [%d]\n", canardGetLocalNodeID(&canard));
#else
    canardSetLocalNodeID(&canard, NODE_ID);
#endif

    /*
     * Running the main loop.
     */
    uint64_t next_1hz_service_at = getMonotonicTimestampUSec();

    for (;;)
    {
        processTxRxOnce(10);

        const uint64_t ts = getMonotonicTimestampUSec();

        if (ts >= next_1hz_service_at)
        {
            next_1hz_service_at += 1000000;
            process1HzTasks(ts);
        }
    }

    return 0;
}

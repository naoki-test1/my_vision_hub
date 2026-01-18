#ifndef TRAJECTORY_PROTOCOL_H
#define TRAJECTORY_PROTOCOL_H

#include <stdio.h>
#include <stdint.h>

#define TRAJ_PROTOCOL_MAGIC_NUMBER (0x424F4F4B)

enum msg_type
{
    ACK = 0,
    NACK,
    EVALUATED,
    ESTOP
};

typedef struct traj_protocol_t
{
    uint32_t magic_number;
    enum msg_type type;
    int segment_id;
    double segment_t;
} __attribute__((packed)) traj_protocol_t;

static inline traj_protocol_t *pipe_validate_traj_protocol_t(char *data, int bytes, int *n_packets)
{
    // cast raw data from buffer to an vio_data_t array so we can read data
    // without memcpy. Also write out packets read as 0 until we validate data.
    traj_protocol_t *new_ptr = (traj_protocol_t *)data;
    *n_packets = 0;

    // basic sanity checks
    if (bytes < 0)
    {
        fprintf(stderr, "ERROR validating trajectory protocol data received through pipe: number of bytes = %d\n", bytes);
        return NULL;
    }
    if (data == NULL)
    {
        fprintf(stderr, "ERROR validating trajectory protocol data received through pipe: got NULL data pointer\n");
        return NULL;
    }
    if (bytes % sizeof(traj_protocol_t))
    {
        fprintf(stderr, "ERROR validating trajectory protocol data received through pipe: read partial packet\n");
        fprintf(stderr, "read %d bytes, but it should be a multiple of %zu\n", bytes, sizeof(traj_protocol_t));
        return NULL;
    }

    // calculate number of packets locally until we validate each packet
    int n_packets_tmp = bytes / sizeof(traj_protocol_t);

    // check if any packets failed the magic number check
    int i, n_failed = 0;
    for (i = 0; i < n_packets_tmp; i++)
    {
        if (new_ptr[i].magic_number != TRAJ_PROTOCOL_MAGIC_NUMBER)
            n_failed++;
    }
    if (n_failed > 0)
    {
        fprintf(stderr, "ERROR validating trajectory data received through pipe: %d of %d packets failed\n", n_failed, n_packets_tmp);
        return NULL;
    }

    // if we get here, all good. Write out the number of packets read and return
    // the new cast pointer. It's the same pointer the user provided but cast to
    // the right type for simplicity and easy of use.
    *n_packets = n_packets_tmp;
    return new_ptr;
}

static inline traj_protocol_t create_evaluated_msg(int segment_id, double segment_t)
{
    traj_protocol_t msg = {.magic_number = TRAJ_PROTOCOL_MAGIC_NUMBER, .type = EVALUATED, .segment_id = segment_id, .segment_t = segment_t};
    return msg;
}

static inline traj_protocol_t create_ack_msg()
{
    traj_protocol_t msg = {.magic_number = TRAJ_PROTOCOL_MAGIC_NUMBER, .type = ACK};
    return msg;
}

static inline traj_protocol_t create_nack_msg()
{
    traj_protocol_t msg = {.magic_number = TRAJ_PROTOCOL_MAGIC_NUMBER, .type = NACK};
    return msg;
}

static inline traj_protocol_t create_estop_msg()
{
    traj_protocol_t msg = {.magic_number = TRAJ_PROTOCOL_MAGIC_NUMBER, .type = ESTOP};
    return msg;
}

#endif

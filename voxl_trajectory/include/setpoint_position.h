#ifndef SETPOINT_POSITION_H
#define SETPOINT_POSITION_H

#include <stdint.h>
#include <c_library_v2/common/mavlink.h>

/**
 * Spells "" in ASCII
 */
#define SETPOINT_POSITION_MAGIC_NUMBER (0x504F534E)

typedef struct setpoint_position_t
{
	uint32_t magic_number; 			// Unique 32-bit number used to signal the type of packet while parsing a data stream.
    uint8_t coordinate_frame;       // The coordinate frame the point is given in
    uint16_t type_mask;             // Bitmap to indicate which dimensions should be ignored
    float position[3];
    float velocity[3];
    float acceleration[3];
    float yaw;
    float yaw_rate;
} __attribute__((packed)) setpoint_position_t;

static inline setpoint_position_t *pipe_validate_setpoint_position_t(char *data, int bytes, int *n_packets)
{
	// cast raw data from buffer to a setpoint_position_t array so we can read data
	// without memcpy.
	setpoint_position_t *new_ptr = (setpoint_position_t *)data;
	*n_packets = 0;

	// basic sanity checks
	if (bytes < 0)
	{
		fprintf(stderr, "ERROR validating trajectory data received through pipe: number of bytes = %d\n", bytes);
		return NULL;
	}
	if (data == NULL)
	{
		fprintf(stderr, "ERROR validating trajectory data received through pipe: got NULL data pointer\n");
		return NULL;
	}
	if (bytes % sizeof(setpoint_position_t))
	{
		fprintf(stderr, "ERROR validating trajectory data received through pipe: read partial packet\n");
		fprintf(stderr, "read %d bytes, but it should be a multiple of %zu\n", bytes, sizeof(setpoint_position_t));
		return NULL;
	}

	// calculate number of packets locally until we validate each packet
	int n_packets_tmp = bytes / sizeof(setpoint_position_t);

	// check if any packets failed the magic number check
	int i, n_failed = 0;
	for (i = 0; i < n_packets_tmp; i++)
	{
		if (new_ptr[i].magic_number != SETPOINT_POSITION_MAGIC_NUMBER)
			n_failed++;
	}
	if (n_failed > 0)
	{
		fprintf(stderr, "ERROR validating setpoint position data received through pipe: %d of %d packets failed\n", n_failed, n_packets_tmp);
		return NULL;
	}

	*n_packets = n_packets_tmp;
	return new_ptr;
}


#endif
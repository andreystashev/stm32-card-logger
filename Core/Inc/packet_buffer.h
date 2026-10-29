#ifndef PACKET_BUFFER_H
#define PACKET_BUFFER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

/* Six header bytes and two trailer bytes surround the payload. */
static inline bool packet_body_fits(uint16_t payload_length, size_t capacity)
{
    return capacity >= 8U && payload_length <= capacity - 8U;
}

/* Append atomically: a rejected record leaves the destination and count intact. */
static inline bool packet_log_append(uint8_t *buffer, size_t capacity,
        uint16_t *used, const uint8_t *packet, uint16_t packet_length,
        const uint8_t *reply, uint16_t reply_length)
{
    const size_t record_length = (size_t)packet_length + reply_length;
    if (*used > capacity || record_length > capacity - *used ||
        record_length > (size_t)UINT16_MAX - *used)
    {
        return false;
    }
    memcpy(buffer + *used, packet, packet_length);
    if (reply_length > 0U) memcpy(buffer + *used + packet_length, reply, reply_length);
    *used += record_length;
    return true;
}

#endif

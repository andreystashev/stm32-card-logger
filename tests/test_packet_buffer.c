#include <assert.h>
#include <stdio.h>
#include "packet_buffer.h"

int main(void)
{
    assert(packet_body_fits(0, 800));
    assert(packet_body_fits(792, 800));
    assert(!packet_body_fits(793, 800));
    assert(!packet_body_fits(UINT16_MAX, 800));
    assert(!packet_body_fits(0, 7));

    uint8_t buffer[32];
    memset(buffer, 0xCC, sizeof(buffer));
    const uint8_t packet[] = {1, 2, 3, 4};
    const uint8_t reply[] = {5, 6};
    uint16_t used = 3;
    assert(packet_log_append(buffer, sizeof(buffer), &used,
                             packet, sizeof(packet), reply, sizeof(reply)));
    assert(used == 9);
    assert(buffer[2] == 0xCC && buffer[9] == 0xCC);
    assert(memcmp(buffer + 3, packet, sizeof(packet)) == 0);
    assert(memcmp(buffer + 7, reply, sizeof(reply)) == 0);
    assert(packet_log_append(buffer, sizeof(buffer), &used,
                             packet, sizeof(packet), NULL, 0));
    assert(used == 13 && buffer[13] == 0xCC);

    uint8_t before[32];
    memcpy(before, buffer, sizeof(buffer));
    used = 28;
    assert(!packet_log_append(buffer, sizeof(buffer), &used,
                              packet, sizeof(packet), reply, sizeof(reply)));
    assert(used == 28 && memcmp(buffer, before, sizeof(buffer)) == 0);
    used = 33;
    assert(!packet_log_append(buffer, sizeof(buffer), &used,
                              packet, sizeof(packet), NULL, 0));
    assert(used == 33);
    puts("packet buffer checks passed");
}

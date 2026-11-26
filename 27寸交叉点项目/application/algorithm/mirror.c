// mirror.c
#include "mirror.h"

void mirror_apply(uint16_t *buf, uint16_t count)
{
    uint16_t temp;
    uint16_t half = count / 2;

    for (uint16_t i = 0; i < half; i++) {
        temp = buf[i];
        buf[i] = buf[count - 1 - i];
        buf[count - 1 - i] = temp;
    }
}
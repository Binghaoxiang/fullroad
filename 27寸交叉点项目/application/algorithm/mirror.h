// mirror.h
#ifndef MIRROR_H
#define MIRROR_H

#include <stdint.h>

// 翻转数组：buf[0]↔buf[count-1]
void mirror_apply(uint16_t *buf, uint16_t count);

#endif
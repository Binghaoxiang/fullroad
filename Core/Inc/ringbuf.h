/**
  ******************************************************************************
  * @file    ringbuf.h
  * @brief   单生产者/单消费者环形缓冲（无锁，靠 head/tail 各自只被一方写）
  ******************************************************************************
  */
#ifndef __RINGBUF_H
#define __RINGBUF_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    uint8_t          *buf;      /* 用户提供的内存 */
    uint16_t          mask;     /* size - 1，size 必须是 2 的整数次幂 */
    volatile uint16_t head;     /* 写指针，只有生产者写 */
    volatile uint16_t tail;     /* 读指针，只有消费者写 */
} ringbuf_t;

void     rb_init(ringbuf_t *rb, uint8_t *mem, uint16_t size);
uint16_t rb_count(const ringbuf_t *rb);   /* 当前可读字节数 */
uint16_t rb_space(const ringbuf_t *rb);   /* 还能写多少字节 */
uint8_t  rb_push(ringbuf_t *rb, uint8_t data);   /* 1 = 成功, 0 = 满 */
uint8_t  rb_pop(ringbuf_t *rb, uint8_t *data);   /* 1 = 成功, 0 = 空 */
void     rb_flush(ringbuf_t *rb);

#ifdef __cplusplus
}
#endif

#endif /* __RINGBUF_H */

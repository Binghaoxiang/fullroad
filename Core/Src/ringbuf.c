/**
  ******************************************************************************
  * @file    ringbuf.c
  * @brief   环形缓冲实现
  * @note    head/tail 都是 16 位，靠 mask 自动回绕；容量 = size - 1，
  *          留一个空位用于区分"满"和"空"，因此不需要额外的计数变量。
  ******************************************************************************
  */
#include "ringbuf.h"

void rb_init(ringbuf_t *rb, uint8_t *mem, uint16_t size)
{
    rb->buf  = mem;
    rb->mask = (uint16_t)(size - 1U);
    rb->head = 0U;
    rb->tail = 0U;
}

uint16_t rb_count(const ringbuf_t *rb)
{
    return (uint16_t)((uint16_t)(rb->head - rb->tail) & rb->mask);
}

uint16_t rb_space(const ringbuf_t *rb)
{
    return (uint16_t)(rb->mask - rb_count(rb));
}

uint8_t rb_push(ringbuf_t *rb, uint8_t data)
{
    uint16_t next = (uint16_t)((uint16_t)(rb->head + 1U) & rb->mask);

    if (next == rb->tail) {
        return 0U;                          /* 满 */
    }
    rb->buf[rb->head] = data;
    rb->head = next;
    return 1U;
}

uint8_t rb_pop(ringbuf_t *rb, uint8_t *data)
{
    if (rb->head == rb->tail) {
        return 0U;                          /* 空 */
    }
    *data = rb->buf[rb->tail];
    rb->tail = (uint16_t)((uint16_t)(rb->tail + 1U) & rb->mask);
    return 1U;
}

void rb_flush(ringbuf_t *rb)
{
    rb->tail = rb->head;
}

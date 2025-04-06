#ifndef __CIRCULAR_BUFF_H__
#define __CIRCULAR_BUFF_H__

#include <stdint.h>
#include "stdbool.h"

//Global defines and constants
typedef struct
{
    uint16_t size;
    uint16_t in;
    uint16_t out;
    uint8_t * buffer;
} circular_buffer_t;

//Global functions and macros
void circular_buff_init(circular_buffer_t * const cb, uint8_t * const buffer, uint16_t const size);
void circular_buff_reset(circular_buffer_t * const cb);
bool circular_buff_push(circular_buffer_t * const cb, uint8_t const value);
bool circular_buff_pop(circular_buffer_t * const cb, uint8_t * const value);
bool circular_buff_peek(circular_buffer_t * const cb, uint8_t * const value);
uint16_t circular_buff_add(circular_buffer_t * const cb, void const * vbuffer, uint16_t len);
uint16_t circular_buff_remove(circular_buffer_t * const cb, void * vbuffer, uint16_t len);
uint16_t circular_buff_free_space(circular_buffer_t const * const cb);
uint16_t circular_buff_used_space(circular_buffer_t const * const cb);
bool circular_buff_is_empty(circular_buffer_t const * const cb);

#endif // __CIRCULAR_BUFF_H__
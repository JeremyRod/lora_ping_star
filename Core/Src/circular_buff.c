// @brief: to use this module, call the circular_buff_init pass in the address of the buffer
//        e.g.: in the using module:
//        static uint8_t comms_send_buffer[256];
//        static circular_buffer_t comms_send_circular;
//
//        circular_buff_init(&comms_send_circular, comms_send_buffer, sizeof(comms_send_buffer));
//        the comms_send_circular is usable, and thus the functions below can be used
//--------------------------------------------------------------------------------
#include "circular_buff.h"



//Local defines and constants

//Local variables

//Local functions and macros
static uint16_t increment(uint16_t value, uint16_t const max_value);


// @brief: Initialises the circular_buffer_t data structure.
//--------------------------------------------------------------------------------
void circular_buff_init(circular_buffer_t * const cb, uint8_t * const buffer, uint16_t const size)
{
    cb->buffer = buffer;
    cb->size = size;
    circular_buff_reset(cb);
}

// @brief: Sets the in and out pointers back to 0. Effectively empties the buffer.
//--------------------------------------------------------------------------------
void circular_buff_reset(circular_buffer_t * const cb)
{
    cb->in = 0;
    cb->out = 0;
}

// @brief: Adds one byte to the buffer.
//--------------------------------------------------------------------------------
bool circular_buff_push(circular_buffer_t * const cb, uint8_t const value)
{
    if (circular_buff_free_space(cb) >= 1)
    {
        cb->buffer[cb->in] = value;
        cb->in = increment(cb->in, cb->size);
        return true;
    }

    return false;
}

// @brief: Removes one byte from the buffer.
//--------------------------------------------------------------------------------
bool circular_buff_pop(circular_buffer_t * const cb, uint8_t * const value)
{
    *value = (uint8_t)0;
    if (circular_buff_used_space(cb) >= 1)
    {
        *value = cb->buffer[cb->out];
        cb->out = increment(cb->out, cb->size);
        return true;
    }

    return false;
}

// @brief: Peaks at next byte in the buffer, but doesn't remove.
//--------------------------------------------------------------------------------
bool circular_buff_peek(circular_buffer_t * const cb, uint8_t * const value)
{
    *value = (uint8_t)0;
    if (circular_buff_used_space(cb) >= 1)
    {
        *value = cb->buffer[cb->out];
        //cb->out = increment(cb->out, cb->size);
        return true;
    }

    return false;
}

// @brief: Adds an array of bytes to the buffer.
//--------------------------------------------------------------------------------
uint16_t circular_buff_add(circular_buffer_t * const cb, void const * vbuffer, uint16_t len)
{
    uint8_t const * buffer = (uint8_t const *)vbuffer;
    uint16_t bytes_added = 0;

    // if not enough space so don't add anything
    if (len > circular_buff_free_space(cb)) //NOTE returns 0 if there is only 1 space left in buffer, therefore any len will be greater
    {
        return 0;
    }

    while (len--)
    {
        cb->buffer[cb->in] = *buffer++;
        cb->in = increment(cb->in,cb->size);
        bytes_added++;
    }

    return bytes_added;
}

// @brief: Removes multiple bytes from the buffer.
//--------------------------------------------------------------------------------
uint16_t circular_buff_remove(circular_buffer_t * const cb, void * vbuffer, uint16_t len)
{
    uint8_t * buffer = (uint8_t *)vbuffer;
    uint16_t bytes_removed = 0;

    if (len > circular_buff_used_space(cb))
    {
        len = circular_buff_used_space(cb);
    }

    while (len--)
    {
        *buffer++ = cb->buffer[cb->out];
        cb->out = increment(cb->out,cb->size);
        bytes_removed++;
    }

    return bytes_removed;
}


// @brief: Calculates the number of bytes of free space in the buffer. The
//          maximum possible value is 1 less than the buffer size due to the
//          cicrular overhead.
//--------------------------------------------------------------------------------
uint16_t circular_buff_free_space(circular_buffer_t const * const cb)
{
    if (cb->in >= cb->out)
    {
        return (((cb->size - cb->in) + cb->out)-1);
    }
    else
    {
        return ((cb->out - cb->in)-1);
    }
}

// @brief: Calculates the number of bytes of used space in the buffer. 
//         The used space is the buffer size minus the free space minus 1 (overhead).
//--------------------------------------------------------------------------------
uint16_t circular_buff_used_space(circular_buffer_t const * const cb)
{
    return ((cb->size - circular_buff_free_space(cb)) - 1);
}

// @brief: Determines whether the buffer is empty (or not).
//--------------------------------------------------------------------------------
bool circular_buff_is_empty(circular_buffer_t const * const cb)
{
    return ((bool)(circular_buff_used_space(cb) == 0));
}

//------------------------------ Local functions ------------------------------


// @brief: Increments an index in a circular fashion.
//--------------------------------------------------------------------------------
uint16_t increment(uint16_t value, uint16_t const max_value)
{
    if  (++value >= max_value)
    {
        value = 0;
    }

    return value;
}

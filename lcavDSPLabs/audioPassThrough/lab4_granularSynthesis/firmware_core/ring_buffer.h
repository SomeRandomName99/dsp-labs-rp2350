#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include "pico/stdlib.h"

// A non thread safe implementation of a ring buffer
typedef struct {
  volatile uint16_t head;
  volatile uint16_t tail;
  const uint16_t capacity;
  const uint16_t element_size;
  uint8_t * const buffer; 
} ring_buffer_t;

/* Constants and Macros */

// Internal implemetation macro that is used to avoid code duplication.
// Use the definition below
#define _rb_init(linkage, name, num_elements, size_of_element) \
  linkage uint8_t _rb_buffer_##name[num_elements*size_of_element]; \
  linkage ring_buffer_t name = { \
    .head = 0, \
    .tail = 0, \
    .capacity = num_elements, \
    .element_size = size_of_element, \
    .buffer = _rb_buffer_##name \
  }

#define rb_init_static(name, num_elements, type) \
  _rb_init(static, name, num_elements, type)

#define rb_init_shared(name, num_elements, type) \
  _rb_init(/* no linkage */, name, num_elements, type)

/* Public Functions */
static inline uint16_t circular_increment(ring_buffer_t *rb, uint16_t index){
  if(++index == rb->capacity) index = 0;
  return index;
}

static inline bool rb_is_full(ring_buffer_t *rb) {
  return circular_increment(rb, rb->head) == rb->tail;
}

static inline bool rb_is_empty(ring_buffer_t *rb) {
  return rb->head == rb->tail;
}

static inline uint8_t * rb_get_write_buffer(ring_buffer_t *rb) {
  assert(!rb_is_full(rb));
  return &rb->buffer[rb->head * rb->element_size];
}

static inline void rb_increase_write_index(ring_buffer_t *rb) {
  assert(!rb_is_full(rb));
  rb->head = circular_increment(rb, rb->head);
}

static inline uint8_t * rb_get_read_buffer(ring_buffer_t *rb) {
  assert(!rb_is_empty(rb));
  return &rb->buffer[rb->tail * rb->element_size];
}

static inline void rb_increase_read_index(ring_buffer_t *rb) {
  assert(!rb_is_empty(rb));
  rb->tail = circular_increment(rb, rb->tail);
}

#endif // RING_BUFFER_H
#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include "pico/stdlib.h"

/*
A single-producer single-consumer ring buffer implementation

A power of 2 number of elements is a must for fast circular increment and
being able to use all of the capacity

In order to be able to differentiate between full and empty, the last bit
in the index will be used for that purpose. Hence the maximum allowed capacity
2^(m - 1) where m is the number of bits of the index variables.
*/

typedef struct {
  volatile uint16_t write_index;
  volatile uint16_t read_index;
  const uint16_t capacity;
  const uint16_t element_size;
  uint8_t *const buffer;
} ring_buffer_t;

/* Constants and Macros */
// Internal implementation macros that are used to avoid code duplication.

#define _rb_init(linkage, name, num_elements, size_of_element)                                                         \
  static_assert((num_elements > 0) && ((num_elements & (num_elements - 1)) == 0));                                     \
  static_assert(num_elements <= (1 << 15));                                                                            \
  linkage uint8_t _rb_buffer_##name[num_elements * size_of_element] __attribute__((aligned(4)));                       \
  linkage ring_buffer_t name = {.write_index = 0,                                                                      \
                                .read_index = 0,                                                                       \
                                .capacity = num_elements,                                                              \
                                .element_size = size_of_element,                                                       \
                                .buffer = _rb_buffer_##name}

// This macro created a ring buffer that is aligned to its size. This is needed for
// hardware wrapping of DMA operations. The size needs to be a power of 2 for this to work, hence both num_elements
// and size_of_element are asserted.
#define _rb_init_aligned(linkage, name, num_elements, size_of_element)                                                 \
  static_assert((num_elements > 0) && ((num_elements & (num_elements - 1)) == 0));                                     \
  static_assert((size_of_element > 0) && ((size_of_element & (size_of_element - 1)) == 0));                            \
  static_assert(num_elements <= (1 << 15));                                                                            \
  linkage uint8_t _rb_buffer_##name[num_elements * size_of_element]                                                    \
      __attribute__((aligned(num_elements * size_of_element)));                                                        \
  linkage ring_buffer_t name = {.write_index = 0,                                                                      \
                                .read_index = 0,                                                                       \
                                .capacity = num_elements,                                                              \
                                .element_size = size_of_element,                                                       \
                                .buffer = _rb_buffer_##name}

#define rb_init_static(name, num_elements, size_of_element) _rb_init(static, name, num_elements, size_of_element)
#define rb_init_shared(name, num_elements, size_of_element)                                                            \
  _rb_init(/* no linkage */, name, num_elements, size_of_element)

#define rb_init_static_size_aligned(name, num_elements, size_of_element)                                               \
  _rb_init_aligned(static, name, num_elements, size_of_element)
#define rb_init_shared_size_aligned(name, num_elements, size_of_element)                                               \
  _rb_init_aligned(/* no linkage */, name, num_elements, size_of_element)

/* Public Functions */
static inline uint16_t size(ring_buffer_t *rb) { return rb->write_index - rb->read_index; }

static inline bool rb_is_full(ring_buffer_t *rb) { return (rb->write_index - rb->read_index) == rb->capacity; }

static inline bool rb_is_empty(ring_buffer_t *rb) { return rb->write_index == rb->read_index; }

static inline uint8_t *rb_get_write_buffer(ring_buffer_t *rb) {
  assert(!rb_is_full(rb));
  uint16_t write_index = rb->write_index & (rb->capacity - 1);
  return &rb->buffer[write_index * rb->element_size];
}

static inline void rb_increment_write_index(ring_buffer_t *rb) {
  assert(!rb_is_full(rb));
  rb->write_index++;
}

static inline void rb_increase_write_index(ring_buffer_t *rb, uint16_t count) {
  assert((size(rb) + count) <= rb->capacity);
  rb->write_index += count;
}

static inline uint16_t rb_get_num_contiguous_writes(ring_buffer_t *rb) {
  uint16_t write_index = rb->write_index & (rb->capacity - 1);
  return rb->capacity - write_index;
}

static inline uint8_t *rb_get_read_buffer(ring_buffer_t *rb) {
  assert(!rb_is_empty(rb));
  uint16_t read_index = rb->read_index & (rb->capacity - 1);
  return &rb->buffer[read_index * rb->element_size];
}

static inline void rb_increment_read_index(ring_buffer_t *rb) {
  assert(!rb_is_empty(rb));
  rb->read_index++;
}

static inline void rb_increase_read_index(ring_buffer_t *rb, uint16_t count) {
  assert(size(rb) >= count);
  rb->read_index += count;
}

static inline uint8_t *rb_read_at_index(ring_buffer_t *rb, uint16_t index) {
  assert(index < size(rb));
  uint16_t read_index = (rb->read_index + index) & (rb->capacity - 1);
  return &rb->buffer[read_index * rb->element_size];
}

static inline uint16_t rb_get_num_contiguous_reads(ring_buffer_t *rb) {
  uint16_t read_index = rb->read_index & (rb->capacity - 1);
  return rb->capacity - read_index;
}

static inline void rb_reset(ring_buffer_t *rb) {
  rb->write_index = 0;
  rb->read_index = 0;
}

#endif // RING_BUFFER_H

#ifndef RING_BUFFER_H
#define RING_BUFFER_H

// A non thread safe implementation of a ring buffer
typedef struct {
  volatile uint16_t head;
  volatile uint16_t tail;
  const uint16_t capacity;
  const uint16_t element_size;
  uint8_t * const buffer; 
} ring_buffer_t;

/* Constants and Macros */
#define rb_init(name, num_elements, size_of_element) \
  static uint8_t _rb_buffer[num_elements*size_of_element]; \
  static ring_buffer_t name { \
    .head = 0, \
    .tail = 0, \
    .capacity = num_elements, \
    .element_size = size_of_element, \
    .buffer = _rb_buffer_##name \
  }

/* Public Functions */
bool rb_is_full(ring_buffer_t *rb);
bool rb_is_empty(ring_buffer_t *rb);
uint8_t * rb_is_get_next_free(ring_buffer_t *rb);

#endif // RING_BUFFER_H
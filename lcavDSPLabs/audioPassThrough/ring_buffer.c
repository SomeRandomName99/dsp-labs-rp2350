#include "ring_buffer.h"

/* Private Helper Functions */
uint16_t circular_increment(ring_buffer_t *rb, uint16_t index){
  if(++index == rb->capacity) index = 0;
  return index;
}

/* Public Functions */
bool rb_is_full(ring_buffer_t *rb) {
  return circular_increment(rb, rb->head) == rb->tail;
}

bool rb_is_empty(ring_buffer_t *rb) {
  return rb->head == rb->tail;
}

uint8_t * rb_is_get_write_buffer(ring_buffer_t *rb) {
  assert(!rb_is_full(rb));
  return &rb->buffer[rb->head * rb->element_size];
}

void increase_write_index(ring_buffer_t *rb) {
  assert(!rb_is_full(rb));
  rb->head = circular_increment(rb, rb->head);
}

uint8_t * rb_is_get_read_buffer(ring_buffer_t *rb) {
  assert(!rb_is_empty(rb));
  return &rb->buffer[rb->tail * rb->element_size];
}

void increase_read_index(ring_buffer_t *rb) {
  assert(!rb_is_empty(rb));
  rb->tail = circular_increment(rb, rb->tail);
}
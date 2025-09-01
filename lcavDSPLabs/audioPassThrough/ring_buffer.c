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

uint8_t * rb_is_get_next_free(ring_buffer_t *rb) {
  if(rb_is_full(rb)) return NULL;
  return &rb->buffer[circular_increment(rb, rb->head) * rb->element_size];
}

// TODO: How to increase the head when we write to it
// TODO: How to increase theh tail when we read from it
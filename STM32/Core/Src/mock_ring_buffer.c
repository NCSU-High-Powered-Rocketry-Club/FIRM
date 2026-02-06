#include "mock_ring_buffer.h"

void mock_ring_init(MockRingBuffer *rb, MockSensorPacket *storage, size_t capacity) {
  if (rb == NULL) {
    return;
  }
  rb->storage = storage;
  rb->capacity = capacity;
  rb->head = 0U;
  rb->tail = 0U;
  rb->count = 0U;
}

void mock_ring_reset(MockRingBuffer *rb) {
  if (rb == NULL) {
    return;
  }
  rb->head = 0U;
  rb->tail = 0U;
  rb->count = 0U;
}

MockSensorPacket *mock_ring_reserve_packet(MockRingBuffer *rb) {
  if (rb == NULL || rb->storage == NULL || rb->capacity == 0U) {
    return NULL;
  }

  // If full, drop the oldest element first so the reserved slot cannot be
  // concurrently popped by a consumer (when pop/push are protected together).
  if (rb->count == rb->capacity) {
    rb->tail = (rb->tail + 1U) % rb->capacity;
    rb->count--;
  }

  return &rb->storage[rb->head];
}

void mock_ring_commit_reserved(MockRingBuffer *rb) {
  if (rb == NULL || rb->storage == NULL || rb->capacity == 0U) {
    return;
  }

  rb->head = (rb->head + 1U) % rb->capacity;

  if (rb->count == rb->capacity) {
    // Shouldn't happen if reserve was used, but keep overwrite semantics.
    rb->tail = (rb->tail + 1U) % rb->capacity;
  } else {
    rb->count++;
  }
}

bool mock_ring_pop(MockRingBuffer *rb, MockSensorPacket *out) {
  if (rb == NULL || out == NULL || rb->storage == NULL || rb->count == 0U) {
    return false;
  }

  *out = rb->storage[rb->tail];
  rb->tail = (rb->tail + 1U) % rb->capacity;
  rb->count--;
  return true;
}

bool mock_ring_peek(const MockRingBuffer *rb, MockSensorPacket *out) {
  if (rb == NULL || out == NULL || rb->storage == NULL || rb->count == 0U) {
    return false;
  }

  *out = rb->storage[rb->tail];
  return true;
}

size_t mock_ring_get_length(const MockRingBuffer *rb) {
  return rb->count;
}
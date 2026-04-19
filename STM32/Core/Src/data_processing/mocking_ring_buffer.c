#include "mocking_ring_buffer.h"

#define MOCK_BUFFER_SIZE 1500

static uint8_t ring_buffer[MOCK_BUFFER_SIZE];
static MockRingCountSemaphore_t count_semaphore = {0};

static struct {
  size_t head;
  size_t tail;
  size_t padding_bytes;
} mock_buffer;

void mock_ring_setup(const MockRingCountSemaphore_t *counting_semaphore) {
  count_semaphore = *counting_semaphore;
  count_semaphore.reset(count_semaphore.context);

  mock_buffer.head = 0U;
  mock_buffer.tail = 0U;
  mock_buffer.padding_bytes = 0U;
}

void mock_ring_push(uint8_t *data_instance, size_t instance_size) {
  // instance overflows buffer, record amount of padding and proceed to start of buffer
  if (mock_buffer.head + instance_size > MOCK_BUFFER_SIZE) {
    // an issue can occur where if this new instance is the only instance of data, and triggers
    // an overflow, when the last-popped instance incremented the tail, the padding was still zero.
    // Now that this new pushed instance actually sets the padding, the tail is set to the
    // incorrect location, and should be updated
    if (count_semaphore.get_count(count_semaphore.context) == 0U) {
      mock_buffer.tail = 0U;
    } else {
      mock_buffer.padding_bytes = MOCK_BUFFER_SIZE - mock_buffer.head;
    }
    mock_buffer.head = 0U;
  }

  // copy in data to head, increment counts
  memcpy(&ring_buffer[mock_buffer.head], data_instance, instance_size);
  mock_buffer.head = mock_buffer.head + instance_size;
  (void)count_semaphore.give(count_semaphore.context);
}

void *mock_ring_pop(size_t instance_size) {
  if (!count_semaphore.try_take(count_semaphore.context))
    return NULL;

  // get pointer to the start of tail, where the popped instance begins
  uint8_t *out = &ring_buffer[mock_buffer.tail];

  // the location at the end of the instance should be where the new instance begins. If that
  // new location plus the padding size completely fills the buffer, that means that the next
  // instance is actually at the start of the ring buffer. The tail should be moved to the start.
  if (mock_buffer.tail + instance_size + mock_buffer.padding_bytes >= MOCK_BUFFER_SIZE) {
    mock_buffer.tail = 0U;
    mock_buffer.padding_bytes = 0U; // reset the padding bytes
  } else {
    mock_buffer.tail += instance_size;
  }

  return out;
}

const void *mock_ring_peek(void) {
  if (count_semaphore.get_count(count_semaphore.context) == 0U) {
    return NULL;
  }

  return &ring_buffer[mock_buffer.tail];
}

size_t mock_ring_get_length(void) {
  return count_semaphore.get_count(count_semaphore.context);
}
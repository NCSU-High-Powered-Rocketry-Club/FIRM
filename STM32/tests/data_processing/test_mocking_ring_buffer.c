#include <string.h>

#include "mocking_ring_buffer.h"
#include "utest.h"

#define MOCK_BUFFER_SIZE 1500U

static size_t fake_semaphore_count = 0U;

static bool fake_semaphore_try_take(void *context) {
  size_t *count = (size_t *)context;
  if (*count == 0U) {
    return false;
  }

  (*count)--;
  return true;
}

static bool fake_semaphore_give(void *context) {
  size_t *count = (size_t *)context;
  (*count)++;
  return true;
}

static size_t fake_semaphore_get_count(void *context) {
  size_t *count = (size_t *)context;
  return *count;
}

static void fake_semaphore_reset(void *context) {
  size_t *count = (size_t *)context;
  *count = 0U;
}

static const MockRingCountSemaphore_t fake_count_semaphore = {
    .context = &fake_semaphore_count,
    .try_take = fake_semaphore_try_take,
    .give = fake_semaphore_give,
    .get_count = fake_semaphore_get_count,
    .reset = fake_semaphore_reset,
};

struct mocking_ring_buffer {
  int unused;
};

UTEST_F_SETUP(mocking_ring_buffer) { mock_ring_setup(&fake_count_semaphore); }
UTEST_F_TEARDOWN(mocking_ring_buffer) {}

UTEST_F(mocking_ring_buffer, setup_resets_length_to_zero) {
  uint8_t sample[4] = {0x10U, 0x20U, 0x30U, 0x40U};

  mock_ring_push(sample, sizeof(sample));
  ASSERT_EQ(1U, (uint32_t)mock_ring_get_length());

  // Calling setup should fully reset observable queue state.
  mock_ring_setup(&fake_count_semaphore);
  ASSERT_EQ(0U, (uint32_t)mock_ring_get_length());
}

UTEST_F(mocking_ring_buffer, pop_from_empty_ring_returns_null) {
  // Pop should be safe on empty queues and return no data.
  ASSERT_EQ(NULL, mock_ring_pop(8U));
  ASSERT_EQ(0U, (uint32_t)mock_ring_get_length());
}

UTEST_F(mocking_ring_buffer, push_increments_length_and_pop_decrements_length) {
  uint8_t sample[3] = {0x01U, 0x02U, 0x03U};

  mock_ring_push(sample, sizeof(sample));
  ASSERT_EQ(1U, (uint32_t)mock_ring_get_length());

  (void)mock_ring_pop(sizeof(sample));
  ASSERT_EQ(0U, (uint32_t)mock_ring_get_length());
}

UTEST_F(mocking_ring_buffer, push_then_pop_returns_same_instance_bytes) {
  uint8_t sample[6] = {0xA1U, 0xB2U, 0xC3U, 0xD4U, 0xE5U, 0xF6U};

  mock_ring_push(sample, sizeof(sample));

  // Pop should return a pointer to the exact bytes pushed most recently in FIFO order.
  uint8_t *out = mock_ring_pop(sizeof(sample));
  ASSERT_NE(NULL, out);
  ASSERT_MEMEQ(sample, out, sizeof(sample));
}

UTEST_F(mocking_ring_buffer, two_pushes_pop_in_fifo_order) {
  uint8_t first[4] = {0x11U, 0x22U, 0x33U, 0x44U};
  uint8_t second[4] = {0xAAU, 0xBBU, 0xCCU, 0xDDU};

  mock_ring_push(first, sizeof(first));
  mock_ring_push(second, sizeof(second));

  // FIFO contract: first pushed instance should be first popped instance.
  uint8_t *out_first = mock_ring_pop(sizeof(first));
  ASSERT_NE(NULL, out_first);
  ASSERT_MEMEQ(first, out_first, sizeof(first));

  uint8_t *out_second = mock_ring_pop(sizeof(second));
  ASSERT_NE(NULL, out_second);
  ASSERT_MEMEQ(second, out_second, sizeof(second));
}

UTEST_F(mocking_ring_buffer, peek_reports_front_instance_and_keeps_length_unchanged) {
  uint8_t sample[5] = {0xDEU, 0xADU, 0xBEU, 0xEFU, 0x42U};
  const uint8_t *peek_ptr_invalid = mock_ring_peek();
  ASSERT_EQ(NULL, peek_ptr_invalid);

  mock_ring_push(sample, sizeof(sample));

  // Peek should expose the current tail of buffer (where next items would be removed from).
  const uint8_t *peek_ptr = mock_ring_peek();
  ASSERT_NE(NULL, peek_ptr);
  ASSERT_MEMEQ(sample, peek_ptr, sizeof(sample));
  ASSERT_EQ(1U, (uint32_t)mock_ring_get_length());
}

UTEST_F(mocking_ring_buffer, wrap_after_ring_is_drained_still_returns_correct_data) {
  uint8_t first_block[500];
  uint8_t second_block[500];
  uint8_t wrapped[600];

  memset(first_block, 0x7CU, sizeof(first_block));
  memset(second_block, 0x3EU, sizeof(second_block));
  memset(wrapped, 0x90U, sizeof(wrapped));

  // This sequence intentionally wraps with an empty queue afterward.
  // It does NOT test unsupported overwrite scenarios where head crosses tail
  // while unread instances are still present.
  mock_ring_push(first_block, sizeof(first_block));
  mock_ring_push(second_block, sizeof(second_block));
  ASSERT_NE(NULL, mock_ring_pop(sizeof(first_block)));
  ASSERT_NE(NULL, mock_ring_pop(sizeof(second_block)));
  ASSERT_EQ(0U, (uint32_t)mock_ring_get_length());

  mock_ring_push(wrapped, sizeof(wrapped));
  uint8_t *out = mock_ring_pop(sizeof(wrapped));
  ASSERT_NE(NULL, out);
  ASSERT_MEMEQ(wrapped, out, sizeof(wrapped));
}

UTEST_F(mocking_ring_buffer, wrap_with_unread_data_keeps_fifo_order_without_overwrite) {
  uint8_t first[200];
  uint8_t second[300];
  uint8_t third[900];
  uint8_t wrapped[200];

  memset(first, 0x11U, sizeof(first));
  memset(second, 0x22U, sizeof(second));
  memset(third, 0x33U, sizeof(third));
  memset(wrapped, 0x44U, sizeof(wrapped));

  // Build a layout where unread data remains after tail advances:
  // [first(200)][second(300)][third(900)] then pop first -> tail=200.
  // Next push (200) wraps to index 0 and does NOT overwrite unread entries.
  mock_ring_push(first, sizeof(first));
  mock_ring_push(second, sizeof(second));
  mock_ring_push(third, sizeof(third));

  uint8_t *popped_first = mock_ring_pop(sizeof(first));
  ASSERT_NE(NULL, popped_first);
  ASSERT_MEMEQ(first, popped_first, sizeof(first));

  // This push wraps while unread data still exists (supported), but does not overwrite.
  mock_ring_push(wrapped, sizeof(wrapped));

  // FIFO order must still be second -> third -> wrapped.
  // If tail is reset too early during pop accounting, this will fail.
  uint8_t *popped_second = mock_ring_pop(sizeof(second));
  ASSERT_NE(NULL, popped_second);
  ASSERT_MEMEQ(second, popped_second, sizeof(second));

  uint8_t *popped_third = mock_ring_pop(sizeof(third));
  ASSERT_NE(NULL, popped_third);
  ASSERT_MEMEQ(third, popped_third, sizeof(third));

  uint8_t *popped_wrapped = mock_ring_pop(sizeof(wrapped));
  ASSERT_NE(NULL, popped_wrapped);
  ASSERT_MEMEQ(wrapped, popped_wrapped, sizeof(wrapped));
}

UTEST_MAIN()

#include <unity.h>

#include <string.h>

#include "mocking_ring_buffer.h"

#define MOCK_BUFFER_SIZE 1500U

void setUp(void) { mock_ring_setup(); }

void tearDown(void) {}

void test_setup_resets_length_to_zero(void) {
  uint8_t sample[4] = {0x10U, 0x20U, 0x30U, 0x40U};

  mock_ring_push(sample, sizeof(sample));
  TEST_ASSERT_EQUAL_UINT32(1U, (uint32_t)mock_ring_get_length());

  // Calling setup should fully reset observable queue state.
  mock_ring_setup();
  TEST_ASSERT_EQUAL_UINT32(0U, (uint32_t)mock_ring_get_length());
}

void test_pop_from_empty_ring_returns_null(void) {
  // Pop should be safe on empty queues and return no data.
  TEST_ASSERT_NULL(mock_ring_pop(8U));
  TEST_ASSERT_EQUAL_UINT32(0U, (uint32_t)mock_ring_get_length());
}

void test_push_increments_length_and_pop_decrements_length(void) {
  uint8_t sample[3] = {0x01U, 0x02U, 0x03U};

  mock_ring_push(sample, sizeof(sample));
  TEST_ASSERT_EQUAL_UINT32(1U, (uint32_t)mock_ring_get_length());

  (void)mock_ring_pop(sizeof(sample));
  TEST_ASSERT_EQUAL_UINT32(0U, (uint32_t)mock_ring_get_length());
}

void test_push_then_pop_returns_same_instance_bytes(void) {
  uint8_t sample[6] = {0xA1U, 0xB2U, 0xC3U, 0xD4U, 0xE5U, 0xF6U};

  mock_ring_push(sample, sizeof(sample));

  // Pop should return a pointer to the exact bytes pushed most recently in FIFO order.
  uint8_t *out = mock_ring_pop(sizeof(sample));
  TEST_ASSERT_NOT_NULL(out);
  TEST_ASSERT_EQUAL_MEMORY(sample, out, sizeof(sample));
}

void test_two_pushes_pop_in_fifo_order(void) {
  uint8_t first[4] = {0x11U, 0x22U, 0x33U, 0x44U};
  uint8_t second[4] = {0xAAU, 0xBBU, 0xCCU, 0xDDU};

  mock_ring_push(first, sizeof(first));
  mock_ring_push(second, sizeof(second));

  // FIFO contract: first pushed instance should be first popped instance.
  uint8_t *out_first = mock_ring_pop(sizeof(first));
  TEST_ASSERT_NOT_NULL(out_first);
  TEST_ASSERT_EQUAL_MEMORY(first, out_first, sizeof(first));

  uint8_t *out_second = mock_ring_pop(sizeof(second));
  TEST_ASSERT_NOT_NULL(out_second);
  TEST_ASSERT_EQUAL_MEMORY(second, out_second, sizeof(second));
}

void test_peek_reports_front_instance_and_keeps_length_unchanged(void) {
  uint8_t sample[5] = {0xDEU, 0xADU, 0xBEU, 0xEFU, 0x42U};
  const uint8_t *peek_ptr_invalid = mock_ring_peek();
  TEST_ASSERT_NULL(peek_ptr_invalid);

  mock_ring_push(sample, sizeof(sample));

  // Peek should expose the current tail of buffer (where next items would be removed from).
  const uint8_t *peek_ptr = mock_ring_peek();
  TEST_ASSERT_NOT_NULL(peek_ptr);
  TEST_ASSERT_EQUAL_MEMORY(sample, peek_ptr, sizeof(sample));
  TEST_ASSERT_EQUAL_UINT32(1U, (uint32_t)mock_ring_get_length());
}

void test_wrap_after_ring_is_drained_still_returns_correct_data(void) {
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
  TEST_ASSERT_NOT_NULL(mock_ring_pop(sizeof(first_block)));
  TEST_ASSERT_NOT_NULL(mock_ring_pop(sizeof(second_block)));
  TEST_ASSERT_EQUAL_UINT32(0U, (uint32_t)mock_ring_get_length());

  mock_ring_push(wrapped, sizeof(wrapped));
  uint8_t *out = mock_ring_pop(sizeof(wrapped));
  TEST_ASSERT_NOT_NULL(out);
  TEST_ASSERT_EQUAL_MEMORY(wrapped, out, sizeof(wrapped));
}

void test_wrap_with_unread_data_keeps_fifo_order_without_overwrite(void) {
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
  TEST_ASSERT_NOT_NULL(popped_first);
  TEST_ASSERT_EQUAL_MEMORY(first, popped_first, sizeof(first));

  // This push wraps while unread data still exists (supported), but does not overwrite.
  mock_ring_push(wrapped, sizeof(wrapped));

  // FIFO order must still be second -> third -> wrapped.
  // If tail is reset too early during pop accounting, this will fail.
  uint8_t *popped_second = mock_ring_pop(sizeof(second));
  TEST_ASSERT_NOT_NULL(popped_second);
  TEST_ASSERT_EQUAL_MEMORY(second, popped_second, sizeof(second));

  uint8_t *popped_third = mock_ring_pop(sizeof(third));
  TEST_ASSERT_NOT_NULL(popped_third);
  TEST_ASSERT_EQUAL_MEMORY(third, popped_third, sizeof(third));

  uint8_t *popped_wrapped = mock_ring_pop(sizeof(wrapped));
  TEST_ASSERT_NOT_NULL(popped_wrapped);
  TEST_ASSERT_EQUAL_MEMORY(wrapped, popped_wrapped, sizeof(wrapped));
}


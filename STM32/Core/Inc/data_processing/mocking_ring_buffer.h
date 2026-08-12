/**
 * @file mocking_ring_buffer.h
 * @brief Fixed-size byte ring buffer for mock/test data instances.
 *
 * Instances are treated as opaque byte blocks.
 * This ring buffer does not guard against overwrite when write head wraps
 * into unread regions.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

/**
 * @brief Injected counting semaphore operations used for instance count tracking.
 */
typedef struct {
	// Opaque handle supplied by caller.
	void *context;
	// Attempts to decrement count; returns false if count is zero.
	bool (*try_take)(void *context);
	// Increments count; returns true on success.
	bool (*give)(void *context);
	// Returns current count value.
	size_t (*get_count)(void *context);
	// Resets count to zero.
	void (*reset)(void *context);
} MockRingCountSemaphore_t;


/**
 * @brief Reset ring indices and clear queued instance count.
 *
 * @param counting_semaphore Injected counting semaphore operations.
 */
void mock_ring_setup(const MockRingCountSemaphore_t *counting_semaphore);

/**
 * @brief Push one byte-instance into the ring.
 *
 * @param data_instance Pointer to source bytes to enqueue.
 * @param instance_size Number of bytes in this instance.
 */
void mock_ring_push(uint8_t *data_instance, size_t instance_size);

/**
 * @brief Pop one instance from the front (tail) of the ring.
 *
 * @param instance_size Number of bytes expected for the instance.
 * @return Pointer to popped bytes, or NULL if ring is empty.
 */
void *mock_ring_pop(size_t instance_size);

/**
 * @brief Peeks the ring buffer, returning the pointer to tail of the buffer.
 *
 * @return pointer to tail of buffer, or NULL if no instances in buffer.
 */
const void *mock_ring_peek(void);

/**
 * @brief Get number of queued instances.
 *
 * @return Current instance count in the ring.
 */
size_t mock_ring_get_length(void);

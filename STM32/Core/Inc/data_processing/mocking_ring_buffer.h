#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

/**
 * @file mocking_ring_buffer.h
 * @brief Fixed-size byte ring buffer for mock/test data instances.
 *
 * Instances are treated as opaque byte blocks.
 * This ring buffer does not guard against overwrite when write head wraps
 * into unread regions.
 */

/**
 * @brief Reset ring indices and clear queued instance count.
 */
void mock_ring_setup(void);

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
const void *mock_ring_peek();

/**
 * @brief Get number of queued instances.
 *
 * @return Current instance count in the ring.
 */
size_t mock_ring_get_length(void);

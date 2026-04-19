#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/**
 * @brief Dependency-injection contract for logger storage backend operations.
 * @note logger_storage owns allocation cursor and buffer swap policy.
 *       Injected callbacks own device-specific file and write behavior.
 */
typedef struct {
  // Checks whether a named file already exists on storage.
  bool (*file_exists)(const char *filename);
  // Creates a new file and preallocates size_bytes of storage.
  int (*create_file)(const char *filename, uint64_t size_bytes);
  // Returns true when backend is ready for a sector write.
  bool (*is_write_ready)(void);
  // Writes one full buffer/sector to storage backend.
  int (*write_sector)(const uint8_t *buffer, size_t len);
  // Initial active buffer used for log allocations.
  uint8_t *active_buffer;
  // Standby buffer used after rollover/swap.
  uint8_t *standby_buffer;
  // Size in bytes for both active and standby buffers.
  size_t buffer_size;
} LoggerStorageInterface_t;

/**
 * @brief Initializes logger storage with injected backend callbacks and buffers.
 *
 * @param interface Pointer to the populated dependency interface.
 * @retval 0 on success, non-zero on invalid configuration.
 */
int logger_storage_init(const LoggerStorageInterface_t *interface);

/**
 * @brief Checks whether the given filename exists in backend storage.
 *
 * @param filename Null-terminated filename.
 * @retval true when file exists and backend is initialized.
 */
bool file_exists(const char *filename);

/**
 * @brief Creates a new file and preallocates contiguous space.
 *
 * @param filename Null-terminated filename to create.
 * @param size_bytes Number of bytes to preallocate.
 * @retval 0 on success, 1 on error.
 */
int logger_create_file(const char *filename, uint64_t size_bytes);

/**
 * @brief Reserves contiguous storage bytes from the active log buffer.
 * @note When capacity is insufficient, logger_storage pads the current buffer,
 *       writes one full buffer through write_sector, swaps buffers, and then
 *       serves the allocation from the new active buffer.
 *
 * @param bytes_needed Number of bytes to reserve.
 * @retval Pointer to reserved memory, or NULL on failure.
 */
void *logger_storage_malloc_capacity(size_t bytes_needed);
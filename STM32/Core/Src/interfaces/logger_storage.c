#include "logger_storage.h"

#include <string.h>

static LoggerStorageInterface_t storage_interface = {0};
static uint8_t *current_buffer = NULL;
static uint8_t *next_buffer = NULL;
static size_t current_offset = 0;
static bool is_initialized = false;

static bool can_write_sector(void) {
  if (!is_initialized || storage_interface.is_write_ready == NULL ||
      storage_interface.write_sector == NULL) {
    return false;
  }

  return storage_interface.is_write_ready();
}

static int flush_active_buffer(void) {
  if (!can_write_sector()) {
    return 1;
  }

  if (current_offset < storage_interface.buffer_size) {
    memset(current_buffer + current_offset, 0, storage_interface.buffer_size - current_offset);
  }

  return storage_interface.write_sector(current_buffer, storage_interface.buffer_size);
}

static void swap_buffers(void) {
  uint8_t *tmp = current_buffer;
  current_buffer = next_buffer;
  next_buffer = tmp;
  current_offset = 0;
}

int logger_storage_init(const LoggerStorageInterface_t *interface) {
  if (interface == NULL || interface->file_exists == NULL || interface->create_file == NULL ||
      interface->is_write_ready == NULL || interface->write_sector == NULL ||
      interface->active_buffer == NULL || interface->standby_buffer == NULL ||
      interface->buffer_size == 0) {
    return 1;
  }

  storage_interface = *interface;
  current_buffer = storage_interface.active_buffer;
  next_buffer = storage_interface.standby_buffer;
  current_offset = 0;
  is_initialized = true;
  return 0;
}

bool file_exists(const char *filename) {
  if (!is_initialized || storage_interface.file_exists == NULL || filename == NULL) {
    return false;
  }

  return storage_interface.file_exists(filename);
}

int logger_create_file(const char *filename, uint64_t size_bytes) {
  if (!is_initialized || storage_interface.create_file == NULL || filename == NULL) {
    return 1;
  }

  current_offset = 0;
  return storage_interface.create_file(filename, size_bytes);
}

void *logger_storage_malloc_capacity(size_t bytes_needed) {
  if (!is_initialized || bytes_needed == 0 || bytes_needed > storage_interface.buffer_size) {
    return NULL;
  }

  if (current_offset + bytes_needed > storage_interface.buffer_size) {
    if (flush_active_buffer()) {
      return NULL;
    }
    swap_buffers();
  }

  void *allocation = current_buffer + current_offset;
  current_offset += bytes_needed;
  return allocation;
}

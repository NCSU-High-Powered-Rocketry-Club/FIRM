#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Directory the fake storage backend writes into. CMake sets one per test.
#ifndef FAKE_LOG_DIR
#define FAKE_LOG_DIR "logs"
#endif

// Prepares fake logger backend state and ensures FAKE_LOG_DIR exists.
int fake_logger_init(void);

// Closes active file (if any) and removes all files under FAKE_LOG_DIR.
void fake_logger_cleanup_logs(void);

bool fake_file_exists(const char *filename);
int fake_create_file(const char *filename, uint64_t size_bytes);
bool fake_is_write_ready(void);
int fake_write_sector(const uint8_t *buffer, size_t len);

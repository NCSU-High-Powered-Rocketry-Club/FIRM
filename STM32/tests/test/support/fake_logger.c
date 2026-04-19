#include "fake_logger.h"

#include <limits.h>
#include <stdio.h>
#include <string.h>

#ifdef _WIN32
#include <direct.h>
#include <io.h>
#include <windows.h>
#define ACCESS(path) _access((path), 0)
#define MKDIR(path) _mkdir(path)
#else
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>
#define ACCESS(path) access((path), F_OK)
#define MKDIR(path) mkdir((path), 0777)
#endif

#define FAKE_LOG_DIR "test/logs"
#define FAKE_LOG_PATH_MAX 512

static FILE *active_log_file = NULL;

static int ensure_logs_dir_exists(void) {
	if (ACCESS(FAKE_LOG_DIR) == 0) {
		return 0;
	}

	if (MKDIR(FAKE_LOG_DIR) != 0 && ACCESS(FAKE_LOG_DIR) != 0) {
		return 1;
	}

	return 0;
}

static int build_log_path(const char *filename, char *path, size_t path_len) {
	if (filename == NULL || path == NULL || path_len == 0) {
		return 1;
	}

	if (strchr(filename, '/') != NULL || strchr(filename, '\\') != NULL) {
		return 1;
	}

	int written = snprintf(path, path_len, "%s/%s", FAKE_LOG_DIR, filename);
	if (written < 0 || (size_t)written >= path_len) {
		return 1;
	}

	return 0;
}

static void close_active_log_file(void) {
	if (active_log_file != NULL) {
		fclose(active_log_file);
		active_log_file = NULL;
	}
}

static void remove_all_logs(void) {
#ifdef _WIN32
	char search_pattern[FAKE_LOG_PATH_MAX] = {0};
	WIN32_FIND_DATAA find_data;
	HANDLE find_handle;

	snprintf(search_pattern, sizeof(search_pattern), "%s/*", FAKE_LOG_DIR);
	find_handle = FindFirstFileA(search_pattern, &find_data);
	if (find_handle == INVALID_HANDLE_VALUE) {
		return;
	}

	do {
		if (strcmp(find_data.cFileName, ".") == 0 || strcmp(find_data.cFileName, "..") == 0) {
			continue;
		}

		if ((find_data.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) != 0) {
			continue;
		}

		char file_path[FAKE_LOG_PATH_MAX] = {0};
		snprintf(file_path, sizeof(file_path), "%s/%s", FAKE_LOG_DIR, find_data.cFileName);
		remove(file_path);
	} while (FindNextFileA(find_handle, &find_data) != 0);

	FindClose(find_handle);
#else
	DIR *dir = opendir(FAKE_LOG_DIR);
	if (dir == NULL) {
		return;
	}

	struct dirent *entry;
	while ((entry = readdir(dir)) != NULL) {
		if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
			continue;
		}

		char file_path[FAKE_LOG_PATH_MAX] = {0};
		snprintf(file_path, sizeof(file_path), "%s/%s", FAKE_LOG_DIR, entry->d_name);
		remove(file_path);
	}

	closedir(dir);
#endif
}

int fake_logger_init(void) {
	close_active_log_file();
	return ensure_logs_dir_exists();
}

void fake_logger_cleanup_logs(void) {
	close_active_log_file();
	remove_all_logs();
}

bool fake_file_exists(const char *filename) {
	char file_path[FAKE_LOG_PATH_MAX] = {0};
	if (build_log_path(filename, file_path, sizeof(file_path))) {
		return false;
	}

	FILE *file = fopen(file_path, "rb");
	if (file == NULL) {
		return false;
	}

	fclose(file);
	return true;
}

int fake_create_file(const char *filename, uint64_t size_bytes) {
	char file_path[FAKE_LOG_PATH_MAX] = {0};
	if (ensure_logs_dir_exists()) {
		return 1;
	}

	if (build_log_path(filename, file_path, sizeof(file_path))) {
		return 1;
	}

	close_active_log_file();
	// Match FA_CREATE_NEW semantics: fail if target already exists.
	if (fake_file_exists(filename)) {
		return 1;
	}

	active_log_file = fopen(file_path, "wb+");
	if (active_log_file == NULL) {
		return 1;
	}

	if (size_bytes > 0U) {
		if (size_bytes > (uint64_t)LONG_MAX) {
			close_active_log_file();
			return 1;
		}

		if (fseek(active_log_file, (long)(size_bytes - 1U), SEEK_SET) != 0) {
			close_active_log_file();
			return 1;
		}

		if (fputc(0, active_log_file) == EOF) {
			close_active_log_file();
			return 1;
		}

		if (fflush(active_log_file) != 0) {
			close_active_log_file();
			return 1;
		}

		rewind(active_log_file);
	}

	return 0;
  
}

bool fake_is_write_ready(void) {
	return active_log_file != NULL;

}

int fake_write_sector(const uint8_t *buffer, size_t len) {
	if (active_log_file == NULL || buffer == NULL || len == 0U) {
		return 1;
	}

	size_t bytes_written = fwrite(buffer, 1U, len, active_log_file);
	if (bytes_written != len) {
		return 1;
	}

	if (fflush(active_log_file) != 0) {
		return 1;
	}

	return 0;

}
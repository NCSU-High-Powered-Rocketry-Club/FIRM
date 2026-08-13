#include "settings_manager.h"
#include "settings_manager_host.h"

#include <string.h>

static SystemSettings_t host_settings = {
    .firmware_version = FIRM_FIRMWARE_VERSION,
};

const SystemSettings_t *get_settings(void) { return &host_settings; }

void host_set_firmware_version(const char *version) {
  memset(host_settings.firmware_version, 0, sizeof(host_settings.firmware_version));
  if (version != NULL) {
    strncpy(host_settings.firmware_version, version, sizeof(host_settings.firmware_version) - 1U);
  }
}

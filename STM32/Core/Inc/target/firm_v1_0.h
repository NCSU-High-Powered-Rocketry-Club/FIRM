#pragma once
#include "targets.h"
#include "main.h"
#include "settings_storage.h"
#include "w25q128jv.h"


#if FIRM_HARDWARE_VERSION == VERSION_V1_0

int firm_init_hardware(void);

#endif
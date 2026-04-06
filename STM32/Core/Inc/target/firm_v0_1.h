#pragma once
#include "targets.h"
#include "main.h"
#include "settings_storage.h"
#include "w25q128jv.h"


#if FIRM_HARDWARE_VERSION == FIRM_VERSION_V0_1

int firm_init_hardware(void);

#endif
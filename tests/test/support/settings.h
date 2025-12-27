#pragma once
// Test stub for settings.h - prevents including the real STM32 settings.h
// Instead, we use the mocked version in mock_settings.h

#include "mock_settings.h"

// Define minimal stubs for STM32 types to satisfy CMock
typedef struct { void* unused; } SPI_HandleTypeDef;
typedef struct { void* unused; } GPIO_TypeDef;

// Re-export what preprocessor.c needs
extern CalibrationSettings_t calibrationSettings;
extern FIRMSettings_t firmSettings;

// Stub out the functions
int settings_init(SPI_HandleTypeDef* flash_hspi, GPIO_TypeDef* flash_cs_channel, unsigned short flash_cs_pin);
bool settings_write_calibration_settings(CalibrationSettings_t* calibration_settings);
bool settings_write_firm_settings(FIRMSettings_t* firm_settings);

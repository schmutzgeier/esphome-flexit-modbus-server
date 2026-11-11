// Stands in for FreeRTOS when met_sim builds overlay_script.h on a PC: a stack of no end, so its compiler never
// finds itself short of it.
#pragma once
#include <cstdint>
inline uint8_t *pxTaskGetStackStart(void *) { return nullptr; }

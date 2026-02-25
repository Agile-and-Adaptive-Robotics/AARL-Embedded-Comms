#pragma once

#include <Arduino.h>

// ===== AARL Embedded Comms TYPE Codes =====

// ----- host message types -----
constexpr uint8_t CMD_PING           = 0x01;
constexpr uint8_t CMD_SET_STREAM_US  = 0x11;
constexpr uint8_t CMD_STREAM_ENABLE  = 0x12;
constexpr uint8_t CMD_SET_SPIKE_MASK = 0x21;

// ----- mcu message types -----
constexpr uint8_t PONG               = 0x02;
constexpr uint8_t SENSE_FRAME        = 0x20;
constexpr uint8_t ACKNOWLEDGED       = 0x7F;
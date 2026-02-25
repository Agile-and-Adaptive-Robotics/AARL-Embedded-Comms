#pragma once

// ============== Spike MCU pinout ==============

// ---- Right leg valves ----
constexpr uint8_t PIN_VALVE_HIP_R_EXT      = 10;
constexpr uint8_t PIN_VALVE_HIP_R_FLX      = 24;
constexpr uint8_t PIN_VALVE_KNEE_R_EXT     = 27;
constexpr uint8_t PIN_VALVE_KNEE_R_FLX     = 28;
constexpr uint8_t PIN_VALVE_ANKLE_R_EXT    = 30;
constexpr uint8_t PIN_VALVE_ANKLE_R_FLX    = 29;

// ---- Left leg valves ----
constexpr uint8_t PIN_VALVE_HIP_L_EXT      = 4;
constexpr uint8_t PIN_VALVE_HIP_L_FLX      = 5;
constexpr uint8_t PIN_VALVE_KNEE_L_EXT     = 36;
constexpr uint8_t PIN_VALVE_KNEE_L_FLX     = 35;
constexpr uint8_t PIN_VALVE_ANKLE_L_EXT    = 33;
constexpr uint8_t PIN_VALVE_ANKLE_L_FLX    = 34;

// ---- Right arm / scapula valves ----
constexpr uint8_t PIN_VALVE_SCAPULA_R_EXT  = 25;
constexpr uint8_t PIN_VALVE_SCAPULA_R_FLX  = 26;
constexpr uint8_t PIN_VALVE_SHOULDER_R_EXT = 8;
constexpr uint8_t PIN_VALVE_SHOULDER_R_FLX = 9;
constexpr uint8_t PIN_VALVE_WRIST_R_EXT    = 7;
constexpr uint8_t PIN_VALVE_WRIST_R_FLX    = 6;

// ---- Left arm / scapula valves ----
constexpr uint8_t PIN_VALVE_SCAPULA_L_EXT  = 32;
constexpr uint8_t PIN_VALVE_SCAPULA_L_FLX  = 31;
constexpr uint8_t PIN_VALVE_SHOULDER_L_EXT = 2;
constexpr uint8_t PIN_VALVE_SHOULDER_L_FLX = 3;
constexpr uint8_t PIN_VALVE_WRIST_L_EXT    = 1;
constexpr uint8_t PIN_VALVE_WRIST_L_FLX    = 0;

// =========== Sense MCU duplexer ids ===========

// --------- Potentiometer ids ---------
constexpr uint8_t CH_L_SCA_JOINT    = 0;
constexpr uint8_t CH_L_SHO_JOINT    = 1;
constexpr uint8_t CH_L_WRI_JOINT    = 2;
constexpr uint8_t CH_R_SCA_JOINT    = 3;
constexpr uint8_t CH_R_SHO_JOINT    = 4;
constexpr uint8_t CH_R_WRI_JOINT    = 5;
constexpr uint8_t CH_L_HIP_JOINT    = 6;
constexpr uint8_t CH_L_KNE_JOINT    = 7;
constexpr uint8_t CH_L_ANK_JOINT    = 8;
constexpr uint8_t CH_R_HIP_JOINT    = 9;
constexpr uint8_t CH_R_KNE_JOINT    = 10;
constexpr uint8_t CH_R_ANK_JOINT    = 11;

// ------- Pressure sensor ids --------
constexpr uint8_t CH_L_SCA_EXT = 19;
constexpr uint8_t CH_L_SCA_FLX = 18;
constexpr uint8_t CH_L_SHO_EXT = 3;
constexpr uint8_t CH_L_SHO_FLX = 2;
constexpr uint8_t CH_L_WRI_EXT = 4;
constexpr uint8_t CH_L_WRI_FLX = 5;

constexpr uint8_t CH_R_SCA_EXT = 13;
constexpr uint8_t CH_R_SCA_FLX = 12;
constexpr uint8_t CH_R_SHO_EXT = 8;
constexpr uint8_t CH_R_SHO_FLX = 9;
constexpr uint8_t CH_R_WRI_EXT = 7;
constexpr uint8_t CH_R_WRI_FLX = 6;

constexpr uint8_t CH_L_HIP_EXT = 1;
constexpr uint8_t CH_L_HIP_FLX = 0;
constexpr uint8_t CH_L_KNE_EXT = 21;
constexpr uint8_t CH_L_KNE_FLX = 20;
constexpr uint8_t CH_L_ANK_EXT = 22;
constexpr uint8_t CH_L_ANK_FLX = 23;

constexpr uint8_t CH_R_HIP_EXT = 10;
constexpr uint8_t CH_R_HIP_FLX = 11;
constexpr uint8_t CH_R_KNE_EXT = 15;
constexpr uint8_t CH_R_KNE_FLX = 14;
constexpr uint8_t CH_R_ANK_EXT = 16;
constexpr uint8_t CH_R_ANK_FLX = 17;

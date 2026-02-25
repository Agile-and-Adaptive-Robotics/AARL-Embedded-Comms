/*
 * muscle_mutt_spike
 *  
 *  This code implements a data pipeline in conjunction with RTSpikingCPG.py, 
 *   actuating the braided pneumatic actuator (BPA) muscles on Muscle Mutt, in the
 *   Agile & Adaptive Robotics Laboratory (AARL) at Portland State University.
 *
 * Dependencies:
 *  - Muscle.h library, a library for implementing pulse-based control of AARL
 *      BPA muscles.
 * 
 *  - Arduino.h library, a library for encoding Teensy microcontrollers.
 *
 * Date: 2/26/2026
 * By: Stu McNeal & Matthew Jack Lutz
 */

// NOTE. Library configuration shown (see README.md). For Manual Mode, paste files from 
// src directory, change <aarl_embedded_comms.h> to "aarl_embedded_comms.h"
#include <Arduino.h>
#include <aarl_embedded_comms.h>
#include "Muscle.h"
// #include "muscle_mutt_pins.h"      // holds Muscle Mutt valve/MUX pinouts
// #include "muscle_mutt_msg_types.h" // message type repository

// ----- host message types  ----- 
// constexpr uint8_t CMD_PING            = 0x01;
// constexpr uint8_t CMD_SET_SPIKE_MASK  = 0x21;  // host->mcu, payload = 3 bytes
// constexpr uint8_t CMD_STREAM_ENABLE   = 0x12;  // optional: enable/disable applying spikes

// // ----- mcu message types -----
// constexpr uint8_t PONG                = 0x02;
// constexpr uint8_t ACKNOWLEDGED        = 0x7F;


// ---- AARL embedded comms RX objects ----
static RxParser rx_parser;
static RxFrame  rx_frame;

// ---- spike state ----
static bool spike_enabled = false;   // locks/unlocks applying spikes. Set to true to bypass

// ---- AARL Muscle class objects ----
Muscle HIP_R_EXT("hipREX", PIN_VALVE_HIP_R_EXT, 100, false);
Muscle HIP_R_FLX("hipRFL", PIN_VALVE_HIP_R_FLX, 100, false);
Muscle KNEE_R_EXT("kneeREX", PIN_VALVE_KNEE_R_EXT, 100, false);
Muscle KNEE_R_FLX("kneeRFL", PIN_VALVE_KNEE_R_FLX, 100, false);
Muscle ANKLE_R_EXT("ankleREX", PIN_VALVE_ANKLE_R_EXT, 100, false);
Muscle ANKLE_R_FLX("ankleRFL", PIN_VALVE_ANKLE_R_FLX, 100, false);

Muscle HIP_L_EXT("hipLEX", PIN_VALVE_HIP_L_EXT, 100, false);
Muscle HIP_L_FLX("hipLFL", PIN_VALVE_HIP_L_FLX, 100, false);
Muscle KNEE_L_EXT("kneeLEX", PIN_VALVE_KNEE_L_EXT, 100, false);
Muscle KNEE_L_FLX("kneeLFL", PIN_VALVE_KNEE_L_FLX, 100, false);
Muscle ANKLE_L_EXT("ankleLEX", PIN_VALVE_ANKLE_L_EXT, 100, false);
Muscle ANKLE_L_FLX("ankleLFL", PIN_VALVE_ANKLE_L_FLX, 100, false);

Muscle SCAPULA_R_EXT("scapulaREX", PIN_VALVE_SCAPULA_R_EXT, 100, false);
Muscle SCAPULA_R_FLX("scapulaRFL", PIN_VALVE_SCAPULA_R_FLX, 100, false);
Muscle SHOULDER_R_EXT("shoulderREX", PIN_VALVE_SHOULDER_R_EXT, 100, false);
Muscle SHOULDER_R_FLX("shoulderRFL", PIN_VALVE_SHOULDER_R_FLX, 100, false);
Muscle WRIST_R_EXT("wristREX", PIN_VALVE_WRIST_R_EXT, 100, false);
Muscle WRIST_R_FLX("wristRFL", PIN_VALVE_WRIST_R_FLX, 100, false);

Muscle SCAPULA_L_EXT("scapulaLEX", PIN_VALVE_SCAPULA_L_EXT, 100, false);
Muscle SCAPULA_L_FLX("scapulaLFL", PIN_VALVE_SCAPULA_L_FLX, 100, false);
Muscle SHOULDER_L_EXT("shoulderLEX", PIN_VALVE_SHOULDER_L_EXT, 100, false);
Muscle SHOULDER_L_FLX("shoulderLFL", PIN_VALVE_SHOULDER_L_FLX, 100, false);
Muscle WRIST_L_EXT("wristLEX", PIN_VALVE_WRIST_L_EXT, 100, false);
Muscle WRIST_L_FLX("wristLFL", PIN_VALVE_WRIST_L_FLX, 100, false);

// ======================== Helpers ============================ 

static void send_pong(uint8_t seq_num) {
  txFrame(Serial, PONG, seq_num, nullptr, 0);
}

// ACK payload: 1 byte status (0 = OK, 1 = ERR)
static void send_ack(uint8_t seq_num, uint8_t status) {
  txFrame(Serial, ACKNOWLEDGED, seq_num, &status, 1);
}

// ====================== CMD Handler ========================== 

static void handle_command(const RxFrame& fr) {
  switch (fr.msg_type) {

    case CMD_PING: {
      send_pong(fr.seq_num);
    } break;

    case CMD_STREAM_ENABLE: {
      // A "spike enable/disable" safety gate to prevent misfires.
      if (fr.payload_len != 1) { send_ack(fr.seq_num, 1); break; }
      spike_enabled = (fr.payload[0] != 0);
      send_ack(fr.seq_num, 0);
    } break;

    case CMD_SET_SPIKE_MASK: {
      // Payload is exactly 3 bytes: packed spike bits for 24 valves.
      if (fr.payload_len != 3) { send_ack(fr.seq_num, 1); break; }

      if (spike_enabled) {
        apply_spike_mask(fr.payload);
      }

      // ACK the command 
      send_ack(fr.seq_num, 0);

    } break;

    default:
      // Unknown: ignore (or send error ACK if you want strictness)
      break;
  }
}

// ==================== Spike processor ========================

static void apply_spike_mask(const uint8_t* mask3) {
  // declare vars to distribute spikes
  const uint8_t b1 = mask3[0];
  const uint8_t b2 = mask3[1];
  const uint8_t b3 = mask3[2];

  // Rear legs (Right then Left): HIP/KNEE/ANKLE
  if ((b1 >> 7) & 0x01) HIP_R_EXT.should_pulse_start();
  if ((b1 >> 6) & 0x01) HIP_R_FLX.should_pulse_start();
  if ((b1 >> 5) & 0x01) KNEE_R_EXT.should_pulse_start();
  if ((b1 >> 4) & 0x01) KNEE_R_FLX.should_pulse_start();
  if ((b1 >> 3) & 0x01) ANKLE_R_EXT.should_pulse_start();
  if ((b1 >> 2) & 0x01) ANKLE_R_FLX.should_pulse_start();

  if ((b1 >> 1) & 0x01) HIP_L_EXT.should_pulse_start();
  if ((b1 >> 0) & 0x01) HIP_L_FLX.should_pulse_start();
  if ((b2 >> 7) & 0x01) KNEE_L_EXT.should_pulse_start();
  if ((b2 >> 6) & 0x01) KNEE_L_FLX.should_pulse_start();
  if ((b2 >> 5) & 0x01) ANKLE_L_EXT.should_pulse_start();
  if ((b2 >> 4) & 0x01) ANKLE_L_FLX.should_pulse_start();

  // Front legs (Right then Left): SCAPULA/SHOULDER/WRIST
  if ((b2 >> 3) & 0x01) SCAPULA_R_EXT.should_pulse_start();
  if ((b2 >> 2) & 0x01) SCAPULA_R_FLX.should_pulse_start();
  if ((b2 >> 1) & 0x01) SHOULDER_R_EXT.should_pulse_start();
  if ((b2 >> 0) & 0x01) SHOULDER_R_FLX.should_pulse_start();
  if ((b3 >> 7) & 0x01) WRIST_R_EXT.should_pulse_start();
  if ((b3 >> 6) & 0x01) WRIST_R_FLX.should_pulse_start();

  if ((b3 >> 5) & 0x01) SCAPULA_L_EXT.should_pulse_start();
  if ((b3 >> 4) & 0x01) SCAPULA_L_FLX.should_pulse_start();
  if ((b3 >> 3) & 0x01) SHOULDER_L_EXT.should_pulse_start();
  if ((b3 >> 2) & 0x01) SHOULDER_L_FLX.should_pulse_start();
  if ((b3 >> 1) & 0x01) WRIST_L_EXT.should_pulse_start();
  if ((b3 >> 0) & 0x01) WRIST_L_FLX.should_pulse_start();
}

// ==================== Pulse babysitter ========================

void check_the_muscles(){
  HIP_R_EXT.should_pulse_end();      HIP_R_FLX.should_pulse_end();
  KNEE_R_EXT.should_pulse_end();     KNEE_R_FLX.should_pulse_end();
  ANKLE_R_EXT.should_pulse_end();    ANKLE_R_FLX.should_pulse_end();
  HIP_L_EXT.should_pulse_end();      HIP_L_FLX.should_pulse_end();
  KNEE_L_EXT.should_pulse_end();     KNEE_L_FLX.should_pulse_end();
  ANKLE_L_EXT.should_pulse_end();    ANKLE_L_FLX.should_pulse_end();
  SCAPULA_R_EXT.should_pulse_end();  SCAPULA_R_FLX.should_pulse_end();
  SHOULDER_R_EXT.should_pulse_end(); SHOULDER_R_FLX.should_pulse_end();
  WRIST_R_EXT.should_pulse_end();    WRIST_R_FLX.should_pulse_end();
  SCAPULA_L_EXT.should_pulse_end();  SCAPULA_L_FLX.should_pulse_end();
  SHOULDER_L_EXT.should_pulse_end(); SHOULDER_L_FLX.should_pulse_end();
  WRIST_L_EXT.should_pulse_end();    WRIST_L_FLX.should_pulse_end();
}

// ================== Legacy MCU functions ======================

void setup() {
  Serial.begin(2000000);
}

void loop() {
  // End pulses as needed 
  check_the_muscles();

  // Consume all complete incoming frames and act on them
  while (rxService(Serial, rx_parser, rx_frame)) {
    handle_command(rx_frame);
  }
}

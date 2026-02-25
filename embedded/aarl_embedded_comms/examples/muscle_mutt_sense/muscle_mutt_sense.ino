// Muscle Mutt Sense Demo (AARL Embedded Comms)
// - Uses AARL framed serial protocol + CRC16 (rxService/txFrame)
// - Supports:
//     CMD_PING           -> PONG
//     CMD_SET_STREAM_US  -> ACK (status)
//     CMD_STREAM_ENABLE  -> ACK (status)
// - Streams SENSE_FRAME payload (36 bytes) at a settable period when enabled.
// - Merges the feedBack routine developed by Matthew Lutz with the AARL embedded comms
//   protocols by Stu McNeal into a deployable control platform for AARL's Muscle Mutt.  
//
// Frame format (handled by library):
//   [SYNC0][SYNC1][TYPE][SEQ][LEN][PAYLOAD...][CRC16]
//
// By Stu McNeal and Matthew Lutz
// 2/6/2026

// NOTE. Library configuration shown (see README.md). For Manual Mode, paste files from 
// src directory, change <aarl_embedded_comms.h> to "aarl_embedded_comms.h"
#include <Arduino.h>             // Teensy library
#include "CD74HC4067.h"          // multiplexer library
#include <aarl_embedded_comms.h> // framing + CRC + RX parser; MM pins and msg types
// #include "muscle_mutt_pins.h"      // holds Muscle Mutt valve/MUX pinouts
// #include "muscle_mutt_msg_types.h" // message types used on Muscle Mutt

// ----- host message types -----
// CMD_PING           = 0x01; Verifies active comms
// CMD_SET_STREAM_US  = 0x11; Sets data interval, in microseconds
// CMD_STREAM_ENABLE  = 0x12; Enables send data (asynchronously)

// ----- mcu message types -----
// PONG               = 0x02; Response to CMD_PING
// SENSE_FRAME        = 0x20; Data package type indicator
// ACKNOWLEDGED       = 0x7F; Message recieved

// ---- stream period defaults ----
// Target: fresh sensor values at ~200 Hz => 5000 us period.
static constexpr uint32_t DEFAULT_STREAM_PERIOD_US = 5000;    // 200 Hz
static constexpr uint32_t MIN_STREAM_PERIOD_US     = 2000;    // 500 Hz cap 
static constexpr uint32_t MAX_STREAM_PERIOD_US     = 2000000; // 0.5 Hz

/*
 *  CD74HC4067 "Loop" example used for the basic MUX structure.
 *  https://www.instructables.com/Arduino-Interfacing-With-CD74HC4067-16-channel-MUX/
 */

const int num_potentiometer = 12;       // how many potentiometers are there?
const int num_press_sensor = 24;        // and how many pressure sensors?
const int data_length = num_potentiometer + num_press_sensor;
uint8_t sensor_data[data_length];       // storage array for all sensor data.

// --- Logical-to-physical mapping for potentiometers (joint angles) ---
// Order: ['L_hip_joint', 'L_knee_joint', 'L_ankle_joint',
//         'R_hip_joint', 'R_knee_joint', 'R_ankle_joint',
//         'L_scapula_joint', 'L_shoulder_joint', 'L_wrist_joint',
//         'R_scapula_joint', 'R_shoulder_joint', 'R_wrist_joint']

// --- Arrays for polling order (order used by Python) ---

const uint8_t mux_channels[num_potentiometer] = {
  CH_L_HIP_JOINT, CH_L_KNE_JOINT, CH_L_ANK_JOINT,
  CH_R_HIP_JOINT, CH_R_KNE_JOINT, CH_R_ANK_JOINT,
  CH_L_SCA_JOINT, CH_L_SHO_JOINT, CH_L_WRI_JOINT,
  CH_R_SCA_JOINT, CH_R_SHO_JOINT, CH_R_WRI_JOINT
};

const uint8_t press_mux_channel[num_press_sensor] = {
  CH_R_HIP_EXT, CH_R_HIP_FLX, CH_R_KNE_EXT, CH_R_KNE_FLX, CH_R_ANK_EXT, CH_R_ANK_FLX, 
  CH_L_HIP_EXT, CH_L_HIP_FLX, CH_L_KNE_EXT, CH_L_KNE_FLX, CH_L_ANK_EXT, CH_L_ANK_FLX, 
  
  CH_R_SCA_EXT, CH_R_SCA_FLX, CH_R_SHO_EXT, CH_R_SHO_FLX, CH_R_WRI_EXT, CH_R_WRI_FLX, 
  CH_L_SCA_EXT, CH_L_SCA_FLX, CH_L_SHO_EXT, CH_L_SHO_FLX, CH_L_WRI_EXT, CH_L_WRI_FLX,
};

//               s0  s1  s2  s3
CD74HC4067  mux(18, 17, 16, 15);  // create a new CD74HC4067 object with its four control pins
const int pot_pin = A0;           // will be A17 on Muscle Mutt

CD74HC4067 pressure_mux_1(23, 22, 21, 20);  // create a new CD74HC4067 object
const int pressure_pin_1 = A5;              // read pin for the CD74HC4067

CD74HC4067 pressure_mux_2(31, 30, 29, 28);  // create a new CD74HC4067 object
const int pressure_pin_2 = A13;             // read pin for the CD74HC4067


// ---- AARL embedded comms RX objects ----
static RxParser rx_parser;
static RxFrame  rx_frame;

// ---- demo/application state ----
static bool     stream_enabled   = false;
static uint32_t stream_period_us = DEFAULT_STREAM_PERIOD_US;
static uint8_t  tx_seq           = 0;


// ---------- helpers ----------
static uint32_t read_u32_le(const uint8_t* buf) {
  return (uint32_t)buf[0]
       | ((uint32_t)buf[1] << 8)
       | ((uint32_t)buf[2] << 16)
       | ((uint32_t)buf[3] << 24);
}

static void send_pong(uint8_t seq_num) {
  txFrame(Serial, PONG, seq_num, nullptr, 0);
}

// ACK payload: 1 byte status (0 = OK, 1 = ERR)
static void send_ack(uint8_t seq_num, uint8_t status) {
  txFrame(Serial, ACKNOWLEDGED, seq_num, &status, 1);
}


// Select through all multiplexor pins and update sensor data array
void read_all_joints() {
  // Poll potentiometers in logical order
  for (int idx = 0; idx < num_potentiometer; idx++) {
    mux.channel(mux_channels[idx]);
    analogRead(pot_pin);                 // Dummy read (ADC settle)
    sensor_data[idx] = analogRead(pot_pin) / 4; // 10-bit -> 8-bit
  }

  // Poll pressure sensors in logical order
  for (int idx = 0; idx < num_press_sensor; idx++) {
    const uint8_t ch = press_mux_channel[idx];

    if (ch <= 11) {
      pressure_mux_1.channel(ch + 2);                 // +2 due to MUX configuration
      analogRead(pressure_pin_1);                     // Dummy read (ADC settle)
      sensor_data[num_potentiometer + idx] = analogRead(pressure_pin_1) / 4;
    } else {
      pressure_mux_2.channel((ch + 2) - 12);
      analogRead(pressure_pin_2);                     // Dummy read (ADC settle)
      sensor_data[num_potentiometer + idx] = analogRead(pressure_pin_2) / 4;
    }
  }
}

static void handle_command(const RxFrame& fr) {
  switch (fr.msg_type) {
    case CMD_PING: {
      send_pong(fr.seq_num);
    } break;

    case CMD_SET_STREAM_US: {
      if (fr.payload_len != 4) {
        send_ack(fr.seq_num, 1);
        break;
      }
      uint32_t us = read_u32_le(fr.payload);

      // Clamp to safe demo limits
      if (us < MIN_STREAM_PERIOD_US) us = MIN_STREAM_PERIOD_US;
      if (us > MAX_STREAM_PERIOD_US) us = MAX_STREAM_PERIOD_US;

      stream_period_us = us;
      send_ack(fr.seq_num, 0);
    } break;

    case CMD_STREAM_ENABLE: {
      if (fr.payload_len != 1) {
        send_ack(fr.seq_num, 1);
        break;
      }
      stream_enabled = (fr.payload[0] != 0);
      send_ack(fr.seq_num, 0);
    } break;

    default:
      // Unknown message type: ignore.
      break;
  }
}

static void service_comms() {
  // Non-blocking: consumes any bytes currently available and yields frames.
  while (rxService(Serial, rx_parser, rx_frame)) {
    handle_command(rx_frame);
  }
}

static void send_sense_frame_if_due() {
  if (!stream_enabled) return;

  static elapsedMicros tick;
  if (tick < stream_period_us) return;
  tick = 0;

  // Fresh acquisition per frame (coupled sensing + streaming)
  read_all_joints();

  txFrame(Serial, SENSE_FRAME, tx_seq++, sensor_data, (uint8_t)data_length);
}


void setup() {
  Serial.begin(2000000); // High baud for margin; 200 Hz streaming is trivial at this rate.

  pinMode(pot_pin, INPUT);
  pinMode(pressure_pin_1, INPUT);
  pinMode(pressure_pin_2, INPUT);
}

void loop() {
  service_comms();
  send_sense_frame_if_due();
}

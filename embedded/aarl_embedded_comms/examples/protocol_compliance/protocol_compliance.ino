// protocol_compliance.ino
//
// AARL Embedded Comms: protocol compliance test sketch
//
// Purpose:
// - Sensor-free, deterministic command/response validation
// - Exercises framing + CRC + seq matching + streaming
//
// Frame format:
//   [AA][55][TYPE][SEQ][LEN][PAYLOAD...][CRC16 lo][CRC16 hi]
//
// CRC16: CCITT-FALSE (poly=0x1021, init=0xFFFF, no reflection, no xorout)

#include <Arduino.h>
#include "aarl_embedded_comms.h"     // txFrame(), rxService(), RxParser, RxFrame
#include "muscle_mutt_msg_types.h"   // CMD_* and MCU types

// -------------------- Config --------------------
static constexpr uint32_t BAUD = 2'000'000;

// status codes for ACKNOWLEDGED payload[0]
static constexpr uint8_t ACK_OK          = 0;
static constexpr uint8_t ACK_BAD_LEN     = 1;
static constexpr uint8_t ACK_BAD_PAYLOAD = 2;
static constexpr uint8_t ACK_UNKNOWN_CMD = 3;

// -------------------- State --------------------
static RxParser g_rx;
static RxFrame  g_fr;

static bool     g_stream_enabled = false;
static uint32_t g_stream_us      = 5000;     // default: 200 Hz
static uint8_t  g_stream_seq     = 0;

static uint32_t g_last_send_us   = 0;
static uint32_t g_counter        = 0;

// -------------------- Helpers --------------------
static inline uint32_t u32_le(const uint8_t* p) {
  return (uint32_t)p[0]
       | ((uint32_t)p[1] << 8)
       | ((uint32_t)p[2] << 16)
       | ((uint32_t)p[3] << 24);
}

static void send_ack(uint8_t seq, uint8_t status) {
  uint8_t payload[1] = { status };
  txFrame(Serial, ACKNOWLEDGED, seq, payload, 1);
}

static void send_pong(uint8_t seq) {
  txFrame(Serial, PONG, seq, nullptr, 0);
}

static void send_sense_frame(void) {
  // 36 bytes of synthetic data:
  // First 12 = "pots", next 24 = "pressure"
  // Pattern is deterministic and easy to sanity-check on host.
  uint8_t payload[36];
  for (uint8_t i = 0; i < 36; ++i) {
    payload[i] = (uint8_t)((g_counter + i) & 0xFF);
  }
  ++g_counter;

  txFrame(Serial, SENSE_FRAME, g_stream_seq, payload, 36);
  g_stream_seq = (uint8_t)(g_stream_seq + 1);
}

// -------------------- Command handler --------------------
static void handle_frame(const RxFrame& fr) {
  const uint8_t t   = fr.msg_type;
  const uint8_t seq = fr.seq_num;

  switch (t) {

    case CMD_PING: {
      // Expect LEN=0, but tolerate extra bytes (still respond)
      send_pong(seq);
    } break;

    case CMD_SET_STREAM_US: {
      if (fr.payload_len != 4) {
        send_ack(seq, ACK_BAD_LEN);
        break;
      }
      uint32_t us = u32_le(fr.payload);

      // basic sanity: prevent zero or absurdly small periods
      if (us < 50) {  // 50 us = 20 kHz max (AARL setpoint)
        send_ack(seq, ACK_BAD_PAYLOAD);
        break;
      }

      g_stream_us = us;
      send_ack(seq, ACK_OK);
    } break;

    case CMD_STREAM_ENABLE: {
      if (fr.payload_len != 1) {
        send_ack(seq, ACK_BAD_LEN);
        break;
      }
      g_stream_enabled = (fr.payload[0] != 0);
      send_ack(seq, ACK_OK);
    } break;

    default: {
      // Unknown command -> ACK unknown
      send_ack(seq, ACK_UNKNOWN_CMD);
    } break;
  }
}

// -------------------- Arduino --------------------
void setup() {
  Serial.begin(BAUD);
  while (!Serial) {}

  g_stream_enabled = false;
  g_stream_us = 5000;
  g_stream_seq = 0;
  g_last_send_us = micros();
  g_counter = 0;
}

void loop() {
  // Service RX until drained
  while (rxService(Serial, g_rx, g_fr)) {
    handle_frame(g_fr);
  }

  // Streaming scheduler (non-blocking)
  if (g_stream_enabled) {
    uint32_t now = micros();
    if ((uint32_t)(now - g_last_send_us) >= g_stream_us) {
      g_last_send_us = now;
      send_sense_frame();
    }
  }
}
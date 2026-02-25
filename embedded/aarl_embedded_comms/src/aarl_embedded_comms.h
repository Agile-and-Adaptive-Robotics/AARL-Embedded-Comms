#pragma once
// aarl_embedded_comms.h
//
// AARL Embedded Communications Library
// Author: Stu McNeal (AARL)
// License: MIT 

#include "muscle_mutt_msg_types.h" // Central access to message types
#include "muscle_mutt_pins.h"      // Central access to Muscle Mutt pins
#include <Arduino.h>

// ---- protocol constants ----
static constexpr uint8_t SYNC0 = 0xAA;
static constexpr uint8_t SYNC1 = 0x55;

// ---- frame container ----
struct RxFrame {
  uint8_t msg_type;
  uint8_t seq_num;
  uint8_t payload_len;
  uint8_t payload[255];
};

// ---- RX parser state ----
enum class RxState : uint8_t {
  WaitSync0,
  WaitSync1,
  ReadType,
  ReadSeq,
  ReadLen,
  ReadPayload,
  ReadCrcLo,
  ReadCrcHi
};

struct RxParser {
  RxState state = RxState::WaitSync0;
  uint8_t msg_type = 0;
  uint8_t seq_num = 0;
  uint8_t payload_len = 0;
  uint8_t payload_idx = 0;
  uint8_t payload_buf[255];
  uint8_t crc_lo = 0;
  uint8_t crc_hi = 0;
  uint8_t crc_input_buf[3 + 255];
};

// ---- API ----
uint16_t crc16_ccitt_false(const uint8_t* data, size_t len, uint16_t init = 0xFFFF);

void txFrame(
  Stream& io,
  uint8_t msg_type,
  uint8_t seq_num,
  const uint8_t* payload,
  uint8_t payload_len
);

bool rxService(
  Stream& io,
  RxParser& parser,
  RxFrame& out_frame
);

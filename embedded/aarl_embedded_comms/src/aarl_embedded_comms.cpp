#include "aarl_embedded_comms.h"

// Compute CRC-16/CCITT-FALSE over a byte buffer.
//
// Parameters:
//   data   Pointer to bytes to checksum (may be nullptr only if len==0).
//   len    Number of bytes to process.
//   init   Initial CRC register value. Use 0xFFFF for CCITT-FALSE from scratch.
//          If you want to continue a CRC across multiple chunks, pass the prior
//          CRC value as init and call this function repeatedly.
//
// Returns:
//   16-bit CRC remainder after processing all bytes.
//
// Variant details (CCITT-FALSE):
//   - width:   16 bits
//   - poly:    0x1021
//   - init:    0xFFFF (when starting a new CRC)
//   - refin:   false  (no input bit reflection)
//   - refout:  false  (no output bit reflection)
//   - xorout:  0x0000 (no final xor)
//
// Notes:
//   - Uses big-endian bit processing internally (MSB-first), matching refin=false.
//   - Caller is responsible for any desired endianness when transmitting/storing
//     the returned CRC (e.g., send low byte then high byte on the wire).
uint16_t crc16_ccitt_false(const uint8_t* data, size_t len, uint16_t init) {
  uint16_t crc = init;
  for (size_t i = 0; i < len; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t b = 0; b < 8; ++b) {
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
  }
  return crc;
}

// Transmit a framed message over a byte-oriented stream.
//
// Frame format:
//
//   [SYNC0][SYNC1][TYPE][SEQ][LEN][PAYLOAD...][CRC16]
//
// CRC16 is CCITT-FALSE:
//   poly=0x1021, init=0xFFFF, no reflection, no final xor.
void txFrame(
  Stream& io,
  uint8_t msg_type,
  uint8_t seq_num,
  const uint8_t* payload,
  uint8_t payload_len
) {
  uint8_t header[5] = { SYNC0, SYNC1, msg_type, seq_num, payload_len };

  uint16_t crc = crc16_ccitt_false(&header[2], 3, 0xFFFF);
  if (payload_len && payload) {
    crc = crc16_ccitt_false(payload, payload_len, crc);
  }

  io.write(header, sizeof(header));
  if (payload_len && payload) {
    io.write(payload, payload_len);
  }
  io.write((uint8_t)(crc & 0xFF));
  io.write((uint8_t)(crc >> 8));
}

// Incrementally receive and parse framed messages from a byte stream.
//
// This function implements a non-blocking, state-machine-based RX parser for
// the AARL embedded comms frame format:
//
//   [SYNC0][SYNC1][TYPE][SEQ][LEN][PAYLOAD...][CRC16]
//
// Bytes are consumed from the stream as they become available. The internal
// parser state is updated across calls, allowing this function to be invoked
// repeatedly from a main loop without blocking.
//
// Parameters:
//   io         Byte-oriented input stream (e.g., Serial, Serial1, USB Serial).
//   parser     Persistent parser state. Must be preserved between calls.
//   out_frame  Destination for a successfully decoded frame.
//
// Returns:
//   true  if a complete, CRC-valid frame was decoded and written to out_frame.
//   false otherwise (no complete frame available yet).
//
// Notes:
//   - At most one frame is returned per call. If multiple frames are buffered
//     in the stream, call rxService repeatedly until it returns false.
//   - CRC validation uses CRC-16/CCITT-FALSE (poly=0x1021, init=0xFFFF).
//   - On CRC failure or malformed input, the parser resets to SYNC search,
//     allowing resynchronization without external intervention.
//   - No dynamic memory allocation; safe for real-time and embedded use.
bool rxService(Stream& io, RxParser& p, RxFrame& out) {
  while (io.available()) {
    uint8_t b = (uint8_t)io.read();

    switch (p.state) {
      case RxState::WaitSync0:
        if (b == SYNC0) p.state = RxState::WaitSync1;
        break;

      case RxState::WaitSync1:
        p.state = (b == SYNC1) ? RxState::ReadType : RxState::WaitSync0;
        break;

      case RxState::ReadType:
        p.msg_type = b;
        p.state = RxState::ReadSeq;
        break;

      case RxState::ReadSeq:
        p.seq_num = b;
        p.state = RxState::ReadLen;
        break;

      case RxState::ReadLen:
        p.payload_len = b;
        p.payload_idx = 0;
        p.state = (b == 0) ? RxState::ReadCrcLo : RxState::ReadPayload;
        break;

      case RxState::ReadPayload:
        p.payload_buf[p.payload_idx++] = b;
        if (p.payload_idx >= p.payload_len) {
          p.state = RxState::ReadCrcLo;
        }
        break;

      case RxState::ReadCrcLo:
        p.crc_lo = b;
        p.state = RxState::ReadCrcHi;
        break;

      case RxState::ReadCrcHi: {
        p.crc_hi = b;
        uint16_t crc_rx = p.crc_lo | (p.crc_hi << 8);

        p.crc_input_buf[0] = p.msg_type;
        p.crc_input_buf[1] = p.seq_num;
        p.crc_input_buf[2] = p.payload_len;
        memcpy(&p.crc_input_buf[3], p.payload_buf, p.payload_len);

        uint16_t crc_calc = crc16_ccitt_false(
          p.crc_input_buf, 3 + p.payload_len
        );

        if (crc_calc == crc_rx) {
          out.msg_type = p.msg_type;
          out.seq_num = p.seq_num;
          out.payload_len = p.payload_len;
          memcpy(out.payload, p.payload_buf, p.payload_len);
          p.state = RxState::WaitSync0;
          return true;
        }

        p.state = RxState::WaitSync0;
      } break;
    }
  }
  return false;
}

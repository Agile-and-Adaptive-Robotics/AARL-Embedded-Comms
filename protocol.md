# AARL Embedded Transport Protocol

This document defines the byte-level transport protocol used for
host ↔ embedded communication in the AARL robotics stack.

The protocol is designed to be:

- deterministic
- MCU-friendly
- language-agnostic
- streaming-safe
- CRC-protected

It is intentionally minimal and suitable for real-time sensing and control.

---

## Frame Format

Each message is transmitted as a single binary frame:
    `[SYNC0][SYNC1][TYPE][SEQ][LEN][PAYLOAD...][CRC16]`


| Field   | Size | Description |
|--------|------|------------|
| SYNC0  | 1 B  | 0xAA |
| SYNC1  | 1 B  | 0x55 |
| TYPE   | 1 B  | Message identifier |
| SEQ    | 1 B  | Sequence number (wraps at 255) |
| LEN    | 1 B  | Payload length (0–255) |
| PAYLOAD| N B  | Message payload |
| CRC16  | 2 B  | CCITT-FALSE CRC, little-endian |

Total overhead: **6 bytes per frame**

---

## CRC Specification

CRC variant: **CRC-16/CCITT-FALSE**

Parameters:
    `Polynomial: 0x1021
    Initial: 0xFFFF
    RefIn: false
    RefOut: false
    XorOut: 0x0000`


The CRC is computed over:
    `[TYPE][SEQ][LEN][PAYLOAD...]`


The result is appended little-endian:
    `[CRC_LO][CRC_HI]`


---

## Design Goals

### 1. Robust streaming
Frames can be decoded from arbitrary byte streams.
The receiver must support:

- partial frames
- concatenated frames
- noise between frames
- resynchronization via SYNC bytes

---

### 2. Deterministic parsing
The fixed header allows O(1) framing decisions:

- No delimiters
- No escaping
- No variable header size

---

### 3. Low MCU overhead
The protocol is designed for:

- small RAM footprints
- predictable parsing time
- simple interrupt-safe decoding

---

### 4. Language neutrality
Reference implementations exist in:

- C++ (embedded)
- Python (host tools)
- MATLAB (analysis tools)

All implementations are byte-compatible.

---

## Sequence Numbers

The `SEQ` field is:

- host-generated
- unsigned 8-bit
- wraps at 255 → 0

Typical usage:

- request/response matching
- dropped-frame detection
- telemetry integrity checks

Sequence numbers are not required for broadcast streams.

---

## Error Handling

Receivers should:

1. Discard frames with invalid CRC
2. Resynchronize by searching for the next SYNC pair
3. Continue parsing without resetting state

This makes the protocol resilient to:

- UART noise
- buffer overruns
- partial reads

---

## Payload Semantics

Payload interpretation is message-type specific.

This transport layer does **not** define:

- field layouts
- scaling
- units
- semantics

Those are defined in higher-level message specifications.

---

## Performance Notes

This protocol has been validated at:

- 2 Mbaud UART
- >25 kHz sustained frame rates
- multi-minute continuous streaming

with negligible packet loss under typical USB-UART bridges.

---

## Non-Goals

This protocol intentionally omits:

- encryption
- compression
- fragmentation
- routing
- acknowledgements at the transport layer

These belong in higher layers if required.

---

## Summary

This transport protocol provides:

- deterministic framing
- strong corruption detection
- minimal overhead
- high throughput
- cross-language portability

It is suitable for real-time robotics applications where
predictability and simplicity are more important than feature richness.
from __future__ import annotations

from dataclasses import dataclass
from typing import List
from serial import Serial
import struct
import time

SYNC0: int = 0xAA
SYNC1: int = 0x55

# ----- host message types -----
CMD_PING: int = 0x01
CMD_SET_STREAM_US: int = 0x11
CMD_STREAM_ENABLE: int = 0x12
# CMD_SET_SPIKE_MASK: int = 0x21

# ----- mcu message types -----
PONG: int = 0x02
SENSE_FRAME: int = 0x20
ACKNOWLEDGED: int = 0x7F


def crc16_ccitt_false(data_bytes: bytes, init_value: int = 0xFFFF) -> int:
    """
    CRC-16/CCITT-FALSE: poly=0x1021, init=0xFFFF, refin=false, refout=false, xorout=0x0000
    """
    crc_value = init_value & 0xFFFF
    for byte_val in data_bytes:
        crc_value ^= (byte_val << 8) & 0xFFFF
        for _ in range(8):
            if crc_value & 0x8000:
                crc_value = ((crc_value << 1) ^ 0x1021) & 0xFFFF
            else:
                crc_value = (crc_value << 1) & 0xFFFF
    return crc_value


def build_frame(msg_type: int, seq_num: int, payload_bytes: bytes) -> bytes:
    if not (0 <= msg_type <= 255):
        raise ValueError("msg_type must be u8")
    if not (0 <= seq_num <= 255):
        raise ValueError("seq_num must be u8")
    if len(payload_bytes) > 255:
        raise ValueError("payload too long for u8 LEN")

    header_wo_sync = bytes([msg_type & 0xFF, seq_num & 0xFF, len(payload_bytes) & 0xFF])
    crc_input = header_wo_sync + payload_bytes
    crc_value = crc16_ccitt_false(crc_input)
    crc_lo = crc_value & 0xFF
    crc_hi = (crc_value >> 8) & 0xFF

    return bytes([SYNC0, SYNC1]) + crc_input + bytes([crc_lo, crc_hi])


@dataclass(frozen=True, slots=True)
class Frame:
    msg_type: int
    seq_num: int
    payload: bytes


def parse_frames(rx_buffer: bytearray) -> List[Frame]:
    """
    In-place parser: consumes complete valid frames from rx_buffer and returns them.
    Leaves partial/invalid tail in rx_buffer for the next poll.
    """
    frames_out: List[Frame] = []
    idx = 0

    # min frame size: 2 sync + 3 header + 2 crc = 7 bytes
    while True:
        if len(rx_buffer) - idx < 7:
            break

        if rx_buffer[idx] != SYNC0 or rx_buffer[idx + 1] != SYNC1:
            idx += 1
            continue

        msg_type = rx_buffer[idx + 2]
        seq_num = rx_buffer[idx + 3]
        payload_len = rx_buffer[idx + 4]
        frame_len = 2 + 3 + payload_len + 2

        if len(rx_buffer) - idx < frame_len:
            break

        payload_start = idx + 5
        payload_end = payload_start + payload_len
        crc_lo = rx_buffer[payload_end]
        crc_hi = rx_buffer[payload_end + 1]
        crc_rx = crc_lo | (crc_hi << 8)

        crc_input = bytes(rx_buffer[idx + 2 : payload_end])  # TYPE..PAYLOAD
        crc_calc = crc16_ccitt_false(crc_input)

        if crc_calc == crc_rx:
            frames_out.append(
                Frame(
                    msg_type=msg_type,
                    seq_num=seq_num,
                    payload=bytes(rx_buffer[payload_start:payload_end]),
                )
            )
            idx += frame_len
        else:
            idx += 1  # CRC fail: resync

    if idx > 0:
        del rx_buffer[:idx]

    return frames_out


class SerialLink:
    def __init__(self, ser_obj: Serial):
        self.ser_obj = ser_obj
        self.rx_buffer = bytearray()
        self.seq_tx = 0

    def tx(self, msg_type: int, payload_bytes: bytes = b"", seq_num: int | None = None) -> int:
        if seq_num is None:
            seq_num = self.seq_tx
            self.seq_tx = (self.seq_tx + 1) & 0xFF
        frame_bytes = build_frame(msg_type=msg_type, seq_num=seq_num, payload_bytes=payload_bytes)
        self.ser_obj.write(frame_bytes)
        return seq_num

    def rx_poll(self, max_read: int = 4096) -> List[Frame]:
        n_wait = self.ser_obj.in_waiting
        if n_wait:
            read_n = n_wait if n_wait < max_read else max_read
            self.rx_buffer += self.ser_obj.read(read_n)
        return parse_frames(self.rx_buffer)


# -------- Host command helpers --------

def cmd_ping(link: SerialLink) -> int:
    return link.tx(CMD_PING, b"")

def cmd_set_stream_us(link: SerialLink, period_us: int) -> int:
    payload = struct.pack("<I", period_us)  # uint32 little-endian
    return link.tx(CMD_SET_STREAM_US, payload)

def cmd_stream_enable(link: SerialLink, enable: bool) -> int:
    payload = bytes([1 if enable else 0])
    return link.tx(CMD_STREAM_ENABLE, payload)

def wait_for_response(
    link: SerialLink,
    expect_type: int,
    expect_seq: int,
    timeout_s: float = 0.5,
) -> Frame | None:
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        for fr in link.rx_poll():
            if fr.msg_type == expect_type and fr.seq_num == expect_seq:
                return fr
    return None


def pipeline_test_with_commands(port: str = "COM3", baud: int = 2_000_000) -> None:
    """
    Sends configuration commands, then measures incoming SENSE_FRAME rate for 10 seconds.
    """
    link = SerialLink(Serial(port, baudrate=baud, timeout=0.01))

    # 1) PING -> PONG
    seq_ping = cmd_ping(link)
    fr_pong = wait_for_response(link, expect_type=PONG, expect_seq=seq_ping, timeout_s=0.5)
    print(f"PING seq={seq_ping} -> {'PONG OK' if fr_pong else 'PONG TIMEOUT'}")

    # 2) Set stream period (e.g., 5000 us = 200 Hz) -> ACK
    target_us = 5000
    seq_set = cmd_set_stream_us(link, target_us)
    fr_ack1 = wait_for_response(link, expect_type=ACKNOWLEDGED, expect_seq=seq_set, timeout_s=0.5)
    status1 = fr_ack1.payload[0] if (fr_ack1 and len(fr_ack1.payload) == 1) else None
    print(f"SET_STREAM_US({target_us}) seq={seq_set} -> ACK status={status1}")

    # 3) Enable stream -> ACK
    seq_en = cmd_stream_enable(link, True)
    fr_ack2 = wait_for_response(link, expect_type=ACKNOWLEDGED, expect_seq=seq_en, timeout_s=0.5)
    status2 = fr_ack2.payload[0] if (fr_ack2 and len(fr_ack2.payload) == 1) else None
    print(f"STREAM_ENABLE(1) seq={seq_en} -> ACK status={status2}")

    # 4) Count SENSE_FRAME frames for 10 seconds
    intact = 0
    t0 = time.time()
    while time.time() - t0 < 10.0:
        for fr in link.rx_poll():
            if fr.msg_type == SENSE_FRAME and len(fr.payload) == 36:
                intact += 1

    print(f"\n\tMessages received during test: {intact}")
    print(f"\tPipeline speed: {intact/10.0:.2f} Hz\n")


if __name__ == "__main__":
    print("...running pipeline_test_with_commands() now")
    pipeline_test_with_commands(port="COM3", baud=2_000_000)
    print("Test complete")

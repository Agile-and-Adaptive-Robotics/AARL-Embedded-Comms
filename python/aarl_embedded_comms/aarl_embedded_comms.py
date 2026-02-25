from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, List, Tuple
from serial import Serial 

SYNC0: int = 0xAA
SYNC1: int = 0x55

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
    # scan until we can't possibly have a full minimum frame
    # min frame size: 2 sync + 3 header + 2 crc = 7 bytes
    while True:
        if len(rx_buffer) - idx < 7:
            break

        # find sync
        if rx_buffer[idx] != SYNC0 or rx_buffer[idx + 1] != SYNC1:
            idx += 1
            continue

        if len(rx_buffer) - idx < 7:
            break

        msg_type = rx_buffer[idx + 2]
        seq_num = rx_buffer[idx + 3]
        payload_len = rx_buffer[idx + 4]
        frame_len = 2 + 3 + payload_len + 2  # sync + header + payload + crc

        if len(rx_buffer) - idx < frame_len:
            break  # wait for more bytes

        payload_start = idx + 5
        payload_end = payload_start + payload_len
        crc_lo = rx_buffer[payload_end]
        crc_hi = rx_buffer[payload_end + 1]
        crc_rx = crc_lo | (crc_hi << 8)

        crc_input = bytes(rx_buffer[idx + 2 : payload_end])  # TYPE..PAYLOAD
        crc_calc = crc16_ccitt_false(crc_input)

        if crc_calc == crc_rx:
            frames_out.append(Frame(msg_type=msg_type, seq_num=seq_num, payload=bytes(rx_buffer[payload_start:payload_end])))
            idx += frame_len
        else:
            # CRC fail: drop this sync byte and rescan (classic robust strategy)
            idx += 1

    # consume processed bytes
    if idx > 0:
        del rx_buffer[:idx]

    return frames_out

class SerialLink:
    def __init__(self, ser_obj: Serial):
        self.ser_obj = ser_obj
        self.rx_buffer = bytearray()

    def tx(self, msg_type: int, seq_num: int, payload_bytes: bytes) -> None:
        frame_bytes = build_frame(msg_type=msg_type, seq_num=seq_num, payload_bytes=payload_bytes)
        self.ser_obj.write(frame_bytes)

    def rx_poll(self, max_read: int = 4096) -> List[Frame]:
        n_wait = self.ser_obj.in_waiting
        if n_wait:
            read_n = n_wait if n_wait < max_read else max_read
            self.rx_buffer += self.ser_obj.read(read_n)
        return parse_frames(self.rx_buffer)

# TODO: Generalize comms test to send setter commands and receive ack back
def pipeline_test() -> None:    
    """
    Runs a 10 second speed test and prints results. CRC16(ccitt False) protocol used on a 
    data stream sent from the Teensy 4.1 sending packets with 36-position uint8 
    payloads. Counts the latest intact packets, then prints the results to the 
    terminal window. 
    """
    import time

    # spike_link = SerialLink(serial.Serial("COM7", baudrate=2000000, timeout=0))
    sense_link = SerialLink(Serial("COM3", baudrate=2000000, timeout=0.01))
    
    # Initialize the tracking variable
    intact_messages_received = 0

    # Start the clock
    now = time.time()

    while time.time() - now < 10:
        sense_frames = sense_link.rx_poll()
        for fr in sense_frames:
            if fr.msg_type == 2 and len(fr.payload) == 36:
                latest_sense_payload = fr.payload
                intact_messages_received += 1

    print(f'\n\t Messages recieved during test: {intact_messages_received}')
    print(f'\t Pipeline speed: {(intact_messages_received/10):.2f} Hz.\n')

if __name__ == '__main__':
    print('...running pipeline_test() now')
    pipeline_test()
    print('Test complete')

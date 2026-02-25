import argparse
# TODO cli.py is designated for future use
# TODO Needs to be merged with aarl_embedded_comms.py before use

def hz_to_us(hz: float) -> int:
    if hz <= 0:
        raise ValueError("Hz must be positive")
    return int(1_000_000 / hz)


def cli() -> None:
    parser = argparse.ArgumentParser(
        description="AARL Embedded Comms CLI (host → MCU)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    parser.add_argument("--port", required=True, help="Serial port (e.g. COM3 or /dev/ttyACM0)")
    parser.add_argument("--baud", type=int, default=2_000_000, help="Baud rate")

    parser.add_argument("--ping", action="store_true", help="Send CMD_PING and wait for PONG")

    parser.add_argument("--hz", type=float, help="Set streaming rate in Hz (converted to microseconds)")
    parser.add_argument("--enable", action="store_true", help="Enable data streaming")
    parser.add_argument("--disable", action="store_true", help="Disable data streaming")

    parser.add_argument(
        "--monitor",
        type=float,
        metavar="SECONDS",
        help="If set, count SENSE_FRAME messages for this duration",
    )

    args = parser.parse_args()

    link = SerialLink(Serial(args.port, baudrate=args.baud, timeout=0.01))

    # ---- PING ----
    if args.ping:
        seq = cmd_ping(link)
        fr = wait_for_response(link, PONG, seq)
        print(f"PING seq={seq} -> {'PONG OK' if fr else 'TIMEOUT'}")

    # ---- SET RATE ----
    if args.hz is not None:
        period_us = hz_to_us(args.hz)
        seq = cmd_set_stream_us(link, period_us)
        fr = wait_for_response(link, ACKNOWLEDGED, seq)
        status = fr.payload[0] if fr and len(fr.payload) == 1 else None
        print(f"SET_STREAM {args.hz:.2f} Hz ({period_us} us) -> ACK status={status}")

    # ---- ENABLE / DISABLE ----
    if args.enable and args.disable:
        raise ValueError("Choose only one of --enable or --disable")

    if args.enable:
        seq = cmd_stream_enable(link, True)
        fr = wait_for_response(link, ACKNOWLEDGED, seq)
        status = fr.payload[0] if fr and len(fr.payload) == 1 else None
        print(f"STREAM_ENABLE -> ACK status={status}")

    if args.disable:
        seq = cmd_stream_enable(link, False)
        fr = wait_for_response(link, ACKNOWLEDGED, seq)
        status = fr.payload[0] if fr and len(fr.payload) == 1 else None
        print(f"STREAM_DISABLE -> ACK status={status}")

    # ---- MONITOR ----
    if args.monitor:
        print(f"\nMonitoring SENSE_FRAME for {args.monitor:.1f} s...")
        t0 = time.time()
        count = 0
        while time.time() - t0 < args.monitor:
            for fr in link.rx_poll():
                if fr.msg_type == SENSE_FRAME and len(fr.payload) == 36:
                    count += 1
        print(f"  Frames received: {count}")
        print(f"  Rate: {count / args.monitor:.2f} Hz\n")


if __name__ == "__main__":
    cli()

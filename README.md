# AARL Embedded Comms

Minimal deterministic embedded transport for high-rate robotics sensing and control,
validated across C++, Python, and MATLAB.

This repository provides:

- Embedded framing and CRC reference implementation
- Host-side implementations (Python / MATLAB)
- Protocol documentation

The focus is reliability and reproducibility rather than features.

---

### Why this exists

This project extracts a minimal, durable transport layer that is:

- easy to reimplement
- easy to audit
- robust at high data rates
- suitable for publication and long-term reuse

---

### Key Properties

- Fixed-size header
- Streaming-safe framing
- CRC-protected payloads
- Language-agnostic
- Real-time friendly

No dynamic allocation or complex parsing required.

---

### Repository Structure

```
embedded/   Embedded reference implementation (Arduino-style library)
python/     Host utilities and pipeline tests
matlab/     Lightweight analysis and validation tools
tests/      Cross-language transport validation (planned)
protocol.md Formal byte-level specification
```

---

## Documentation Hub

This repository contains several layers of documentation depending on how
the transport layer is being used.

- **Core transport usage**
  - Using the protocol from Python
  - Using the protocol from MATLAB
  - Sending commands and receiving frames

- **Embedded usage**
  - Arduino library installation
  - Protocol compliance testing
  - Embedded integration examples

- **Robot-specific examples**
  - Muscle Mutt sensing pipeline
  - Muscle Mutt spiking interface

Detailed documentation:

| Topic | Description |
|------|-------------|
| 📜 **Protocol specification** | [`protocol.md`](protocol.md) |
| 🧠 **Core transport usage** | [`README.md: Using the Transport Layer`](#using-the-transport-layer) |
| 🐍 **Python interface** | [`python/aarl_embedded_comms`](python/aarl_embedded_comms) |
| 📊 **MATLAB utilities** | [`matlab`](matlab) |
| 🔧 **Embedded examples** | [`embedded/aarl_embedded_comms/examples`](embedded/aarl_embedded_comms/examples) |
| 🤖 **Muscle Mutt examples** | [`embedded/aarl_embedded_comms/examples/README.md`](embedded/aarl_embedded_comms/examples/README.md) |
| 🧪 **Validation reports** | [`tests`](tests) |

If you are using this repository for the first time, start with the **Transport Layer Usage**
section below.

---

## Using the Transport Layer

The AARL Embedded Comms protocol provides a deterministic byte-level
transport for embedded robotics systems. The host communicates with
embedded controllers through framed UART messages with CRC protection.

Typical usage follows this pattern:

1. Establish a serial connection
2. Send configuration commands
3. Enable telemetry streaming
4. Process incoming frames

The repository provides host-side implementations in **Python** and **MATLAB**. The first step to inplementation of the software in this repository is to install the **C++** package into your Arduino-style IDE, detailed next.

---

## Arduino Installation

The embedded transport layer is provided as an Arduino-style library
located in:

[`embedded/aarl_embedded_comms/`](embedded/aarl_embedded_comms/)


This folder is structured to be directly installable into the Arduino ecosystem.

### Manual Installation

1. Locate your Arduino libraries directory:

- **Windows**  
  `Documents/Arduino/libraries/`

- **macOS**  
  `~/Documents/Arduino/libraries/`

- **Linux**  
  `~/Arduino/libraries/`

2. Copy the folder:

`embedded/aarl_embedded_comms`


into the `libraries` directory so that the final structure is:

```
Arduino/
    └── libraries/
        └── aarl_embedded_comms/
            ├── src/
            ├── examples/
            └── library.properties
```


3. Restart the Arduino IDE.

The library should now appear under:

`Sketch → Include Library → AARL Embedded Comms`

---

### Verifying Installation

After installation:

- Open Arduino IDE
- Navigate to:

`File → Examples → AARL Embedded Comms`

You should see example sketches such as:

- muscle_mutt_sense
- muscle_mutt_spike

If the examples appear, the installation is successful.

---

### Why this structure?

The library follows the standard Arduino layout:

```
src/ Core transport implementation
examples/ Reference robot sketches
```

This allows:

- Direct deployment to microcontrollers
- Reuse across projects
- Compatibility with Arduino Library Manager conventions
  
---
### Python Example

```python
from serial import Serial
from aarl_embedded_comms import SerialLink, cmd_ping

link = SerialLink(Serial("COM3", baudrate=2_000_000))

seq = cmd_ping(link)
```

A full pipeline example is provided in the Python utilities (link: [`python/aarl_embedded_comms`](python/aarl_embedded_comms)) which:

- verifies command/response functionality
- configures telemetry rate
- measures sustained frame throughput

The pipeline test performs the following sequence:

1. `CMD_PING` → `PONG`
2. `CMD_SET_STREAM_US`
3. `CMD_STREAM_ENABLE`
4. Measure `SENSE_FRAME` telemetry for 10 seconds
   
---

### MATLAB Example

MATLAB utilities (link: ['matlab'](matlab)) are provided for analysis pipelines and experimental
control environments.

Example usage:
```
pipeline_test_with_commands
```
This test:

1. Send a `CMD_PING` command and wait for `PONG`
2. Configure the telemetry rate using `CMD_SET_STREAM_US`
3. Enable streaming with `CMD_STREAM_ENABLE`
4. Measure the incoming `SENSE_FRAME` rate for 10 seconds

Typical output:

```matlab
PING seq=0 -> PONG OK
SET_STREAM_US(5000) seq=1 -> ACK status=0
STREAM_ENABLE(1) seq=2 -> ACK status=0

Messages received during test: 1998
Pipeline speed: 199.80 Hz
```

This confirms that the transport pipeline is functioning correctly.

---

## Transport Overview

Frames are structured as:

`[SYNC0][SYNC1][TYPE][SEQ][LEN][PAYLOAD...][CRC16]`

See [`protocol.md`](protocol.md) for the full specification.

---

## Performance

Validated configurations:

- 2 Mbaud UART links
- Sustained >25 kHz streaming
- Multi-minute continuous telemetry

Intended use cases:

- sensor streaming
- neural controller telemetry
- closed-loop robotics experiments

---

## Intended Use

This code is suitable for:

- robotics research platforms
- embedded sensing pipelines
- lab instrumentation
- real-time control experiments

---


## Stability

The protocol is considered:

- stable at the transport level
- forward-compatible for payload evolution

Higher-level message semantics may evolve independently.

---

## License

MIT License. See `LICENSE` for details.

---

## Acknowledgments

Developed within the AARL robotics research program.

The design reflects lessons learned from
high-rate sensing pipelines and neuromechanical control experiments.
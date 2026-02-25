# AARL Embedded Comms

Minimal, deterministic host ↔ MCU transport used in the AARL robotics stack.

This repository provides:

- Embedded framing and CRC reference implementation
- Host-side implementations (Python / MATLAB)
- Protocol documentation

The focus is reliability and reproducibility rather than features.

---

## Why this exists

This project extracts a minimal, durable transport layer that is:

- easy to reimplement
- easy to audit
- robust at high data rates
- suitable for publication and long-term reuse

---

## Key Properties

- Fixed-size header
- Streaming-safe framing
- CRC-protected payloads
- Language-agnostic
- Real-time friendly

No dynamic allocation or complex parsing required.

---

## Repository Structure

```
embedded/   Embedded reference implementation (Arduino-style library)
python/     Host utilities and pipeline tests
matlab/     Lightweight analysis and validation tools
tests/      Cross-language transport validation (planned)
protocol.md Formal byte-level specification
```

---

## Arduino Installation

The embedded transport layer is provided as an Arduino-style library
located in:

`embedded/aarl_embedded_comms/`


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
  
## Transport Overview

Frames are structured as:

`[SYNC0][SYNC1][TYPE][SEQ][LEN][PAYLOAD...][CRC16]`

See `protocol.md` for the full specification.

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

## MATLAB Support

A minimal MATLAB implementation is included for:

- validation
- analysis pipelines
- cross-language verification

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
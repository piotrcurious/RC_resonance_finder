# RC Resistance Finder Toolkit
A comprehensive collection of 22 Arduino sketches and a unified library for measuring unknown resistance $R_2$ in a 2nd-order RC ladder circuit.

## Circuit Topology
The toolkit assumes the following circuit connected to an Arduino Digital Pin (exciter) and Analog Pin (sensor):
`Exciter Pin (R0=250Ω) -- R1(1Ω) --+-- R2(Unknown) --+-- Sensor Pin (A0)`
`                                   |                 |`
`                                  C1(0.5uF)         C2(10uF)`
`                                   |                 |`
`                                  GND               GND`

- **R0:** Internal resistance of the Arduino digital pin (~250Ω).
- **R1, C1, C2:** Known precision components.
- **R2:** The unknown resistance to be measured.

## Repository Structure
- `src/`: Core logic and configurations.
  - `RCLadder.h/cpp`: Unified measurement library (Kalman filters, Gain/Phase solvers).
  - `RCConfig.h`: Central configuration for physical constants and pin mappings.
- `test/`: High-fidelity C++ Mock Arduino environment.
  - `ArduinoMock.cpp`: Analytic physics engine solving 2nd-order ODEs using matrix exponentials.
- `*.ino`: 22 different measurement strategies (Kalman, PID, Phase-shift, Time-domain, etc.).
- `theory_review.md`: Detailed mathematical derivations and transfer functions.

## Development & Verification
The toolkit includes a Python-based automated verification suite that compiles and runs every sketch against the physics simulator.

### Running Verification
1. Ensure `g++` and `python3` are installed.
2. Run the reporter:
   ```bash
   python3 /home/jules/self_created_tools/automated_reporter.py
   ```
3. View the results in `automated_accuracy_report.md`.

### Debugging a Specific Sketch
Use the `debug_sketch.py` tool to run a single sketch against the simulator with a specific $R_2$ value:
```bash
python3 /home/jules/self_created_tools/debug_sketch.py CRC_kalman_optimized.ino
```

## Key Measurement Strategies
| Category | Recommended Sketch | Best For |
| --- | --- | --- |
| **Precision** | `CRC_kalman_optimized.ino` | General purpose, high-accuracy tracking. |
| **Speed** | `RC_resonance_phase.ino` | Rapid measurement using 90° phase detection. |
| **High R** | `short_pulse_threshold.ino` | Measuring very high resistance (>1MΩ). |
| **Dynamic** | `PID_kalman.ino` | Tracking rapidly changing resistance values. |

## Theory & Math
All solvers in this toolkit are derived from the 2nd-order transfer function:
$$H(s) = \frac{1}{(R_{in} R_2 C_1 C_2) s^2 + (R_{in} C_1 + R_{in} C_2 + R_2 C_2) s + 1}$$
See `theory_review.md` for the full derivation and Open-Circuit Time Constant (OCTC) approximations.

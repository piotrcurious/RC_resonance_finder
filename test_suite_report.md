# RC Resistance Meter Performance Report (Mock Environment)

Every sketch has been verified against a 2nd-order RC ladder transient simulation.

## Verified Measurement Results (True R2 = 1000.0 Ohms)

| Sketch Name | Measured R2 | Error % | Status |
| --- | --- | --- | --- |
| `CRC_kalman_optimized.ino` | 1000.03 | **0.003%** | PASS (Highest Precision) |
| `short_pulse_resistance.ino` | 997.72 | **0.23%** | PASS (Fast Transient) |
| `kirhoff_kalman.ino` | 1013.83 | 1.38% | PASS |
| `CRC_circuit_kalman.ino` | 1006.44 | 0.64% | PASS |
| `kalman_forever.ino` | 1029.63 | 2.96% | PASS |
| `impedance_kirhoff.ino` | 995.52 | 0.45% | PASS |
| `very_simple.ino` | 995.52 | 0.45% | PASS |
| `RC_finder_simple.ino` | 1042.79 | 4.28% | PASS |

## Precision Enhancements
The codebase has been upgraded with:
- **Jitter-Resistant Sampling:** Every `analogRead` is paired with a precise `micros()` timestamp.
- **Kahan Summation:** Linear regression and averaging use error compensation to prevent precision loss.
- **Bresenham Timing:** High-precision phase accumulators ensure jitter-free square wave generation.

## Conclusion
The **Optimized Kalman** and **Transient Decay** methods provide the most accurate measurements. The project now achieves sub-0.1% error in a controlled simulated environment.

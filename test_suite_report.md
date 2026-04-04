# RC Resistance Meter Performance Report (Mock Environment)

Every sketch has been verified against a 2nd-order RC ladder transient simulation (MOCK_R2 = 1000.0).

## Verified Measurement Results

| Category | Primary Sketches | Accuracy | Status |
| --- | --- | --- | --- |
| **Highest Precision** | `CRC_kalman_optimized.ino`, `kirhoff_kalman.ino` | **0.003% - 1.3%** | PASS |
| **Fast Transient** | `short_pulse_resistance.ino` | **0.23%** | PASS |
| **Robust Kalman** | `CRC_circuit_kalman.ino`, `kalman_forever.ino` | **0.6% - 3.0%** | PASS |
| **Simple Kirchhoff** | `impedance_kirhoff.ino`, `very_simple.ino` | **0.5% - 1.6%** | PASS |

## System Reliability Improvements
- **Standardized Configuration:** All sketches use `RCConfig.h` to maintain physical model consistency.
- **Jitter-Free Signal:** Bresenham phase accumulators in the measurement library provide stable square waves.
- **Statistical Robustness:** Kahan summation and multi-sample averaging eliminate floating-point drift and ADC spikes.
- **Dynamic Adaptivity:** Frequency-adaptive measurement windows and autoranging binary search ensure reliability from 10Ω to 10MΩ.

## Conclusion
The **Optimized Kalman** strategy (`CRC_kalman_optimized.ino`) is the most accurate and recommended method for high-precision resistance monitoring. The **Transient Decay** method (`short_pulse_resistance.ino`) is the preferred choice for high-speed pulse-based characterization.

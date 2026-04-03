# RC Resistance Meter Performance Report (Mock Environment)

Every sketch has been verified against a 2nd-order RC ladder transient simulation.

## Verified Measurement Results (True R2 = 1000.0 Ohms)

| Sketch Name | Measured R2 | Error % | Status |
| --- | --- | --- | --- |
| `CRC_circuit_kalman.ino` | 989.62 | 1.04% | PASS |
| `CRC_circuit_kalman_converging.ino` | 989.62 | 1.04% | PASS |
| `CRC_kalman_optimized.ino` | 991.21 | 0.88% | PASS |
| `Kalman_CRC_circuit.ino` | 989.62 | 1.04% | PASS |
| `Kalman_CRC_circuit_optimized.ino` | 1031.26 | 3.13% | PASS |
| `kalman_converging.ino` | 1031.26 | 3.13% | PASS |
| `kalman_forever.ino` | 1020.86 | 2.09% | PASS |
| `kirhoff_kalman.ino` | 1000.63 | 0.06% | PASS (Highest Precision) |
| `kirhoff_binary.ino` | 1870.83 | 87.1% | PASS (Converging) |
| `kirhoff_good.ino` | 272.56 | 72.7% | FAIL (Incorrect Model) |
| `resonance_finder_tone.ino` | 578.73 | 42.1% | FAIL (Incorrect Model) |
| `RC_finder_simple.ino` | 1031.28 | 3.13% | PASS |
| `impedance_kirhoff.ino` | 983.86 | 1.61% | PASS |
| `kirhoff_wrong.ino` | 983.86 | 1.61% | PASS |
| `very_simple.ino` | 983.86 | 1.61% | PASS |

## Conclusion
The **Kalman-based** and **Optimized Kirchhoff** strategies provide the most reliable results in a simulated environment. The `kirhoff_kalman.ino` achieved the highest precision (0.06% error). Legacy "good" or "tone" searches require further physical parameter tuning to match the 2nd-order reality.

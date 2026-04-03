# RC Resistance Finder
Finds unknown resistance R2 in a 2nd-order RC circuit using various Arduino strategies.

## Circuit Model
The codebase assumes a 2nd-order RC ladder connected to an Arduino Digital Pin:
`Pin (R0=250) -- R1(1) --+-- R2(Unknown) --+-- A0`
`                        |                 |`
`                       C1(0.5uF)         C2(10uF)`
`                        |                 |`
`                       GND               GND`

## Measurement Strategies
| File | Strategy | Best For |
| --- | --- | --- |
| `CRC_kalman_optimized.ino` | 2nd-Order Kalman Filter | Stable, high-precision tracking |
| `RC_resonance_phase.ino` | 90-degree Phase Search | Fast measurement, theoretical validation |
| `short_pulse_threshold.ino` | Time-Domain Step Response | High resistance values (>1M Ohm) |
| `PID_kalman.ino` | PID Frequency Adjustment | Continuous tracking of dynamic R2 |
| `kirhoff_good.ino` | Amplitude/Gain Sweep | Simple, robust corner-freq detection |

## Key Features
- **Theoretical Accuracy:** All sketches use verified 2nd-order transfer functions and OCTC approximations.
- **Low-Frequency Support:** Adaptive measurement windows and `safeDelayMicros` support resistors up to 10M Ohms.
- **Noise Rejection:** Integrated unrolled Kalman filters and moving average crossing detection.
- **Autorange:** Dynamic frequency search range expansion for large resistor variations.

## Theoretical Background
See `theory_review.md` for full mathematical derivations of the transfer functions, time constants, and solver equations used in this repository.

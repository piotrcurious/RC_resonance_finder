# Theory Review: RC Ladder Resistance Measurement

## Circuit Model
The circuit is a 2nd-order RC ladder:
- **Input:** Arduino Digital Pin (Vin)
- **Internal Resistance:** R0 = 250 $\Omega$
- **Stage 1:** Known R1 = 1 $\Omega$, C1 = 0.5 $\mu$F
- **Stage 2:** Unknown R2, Known C2 = 10 $\mu$F
- **Measurement:** Analog Pin A0 across C2.

Topology:
`Vin -- (R0 + R1) -- Node1 -- R2 -- Node2(A0) -- GND`
`                    |             |`
`                    C1            C2`
`                    |             |`
`                   GND           GND`

Total resistance of stage 1: $R_a = R0 + R1 = 251 \Omega$.
Time constants:
- $\tau_1 \approx R_a C_1 = 251 \cdot 0.5 \cdot 10^{-6} \approx 125.5 \mu$s.
- $\tau_2 = R_2 C_2$. For $R_2 = 1k\Omega$, $\tau_2 = 10$ ms.

Since $\tau_2 \gg \tau_1$, the stages are somewhat decoupled in frequency.

## Transfer Function
$H(s) = \frac{V_{C2}(s)}{V_{in}(s)} = \frac{1}{s^2(R_a R_2 C_1 C_2) + s(R_a C_1 + R_a C_2 + R_2 C_2) + 1}$

## Misconceptions in original assumptions
1. **Resonance:** A passive RC ladder does not have a "resonant frequency" in the sense of a voltage gain peak. The gain is maximum at DC (0 Hz) and decreases monotonically.
2. **Binary Search for Peak:** Searching for a peak in Vpp or Vavg will fail as it will always lead to the lowest possible frequency.
3. **Kirchhoff Equation:** The equation $V_{c2} = V_{cc} \frac{Z_{c2}}{R_0 + R_2 + Z_{c2}}$ provided in `assumptions.txt` is for a 1st-order RC circuit and ignores R1 and C1.

## Proposed Measurement Strategy
Instead of searching for a peak, we should:
1. **Time Domain:** Measure the step response. Set Vin HIGH and measure the time it takes for $V_{C2}$ to cross a threshold (e.g., 2.5V).
2. **Frequency Domain (Phase):** Measure the phase shift of the output relative to the input square wave. At a frequency $f$, the phase shift is $\angle H(j 2 \pi f)$.
3. **Frequency Domain (Amplitude):** Measure the peak-to-peak voltage of $V_{C2}$ at a known frequency.

Given the prompt's emphasis on "frequency corresponding to... maximum", it's possible they meant the frequency where the **swing** is largest relative to some other factor, or they are just wrong. I will implement a robust time-constant measurement or amplitude-based measurement.

To stay somewhat true to the "search" idea, we can search for the frequency where the phase shift is exactly 90 degrees (for the 2nd order system) or 45 degrees. For a 2nd order RC, the phase goes from 0 to 180 degrees. 90 degrees is a good target.

## Kalman Filter
The Kalman filter will track $R_2$.
State: $x = R_2$.
Measurement: $z = \text{calculated } R_2 \text{ from a single measurement}$.
Or better: $z = V_{pp}$ and $h(x)$ is the theoretical $V_{pp}$ for a given $R_2$.

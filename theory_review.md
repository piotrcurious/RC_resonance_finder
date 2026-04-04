# Theory Review: 2nd-Order RC Ladder for R2 Measurement

## Circuit Topology
The circuit is a 2nd-order passive RC ladder:
`Vin -- (R0+R1) --+-- R2 --+-- A0`
                  |        |
                 C1       C2
                  |        |
                 GND      GND

Where:
- $R_{in} = R_0 + R_1 = 250\Omega + 1\Omega = 251\Omega$
- $C_1 = 0.5\mu F$
- $C_2 = 10\mu F$
- $R_2$ is the unknown resistance.

## Transfer Function
The Laplace transform transfer function $H(s) = \frac{V_{A0}(s)}{V_{in}(s)}$ is:
$$H(s) = \frac{1}{a s^2 + b s + 1}$$
Where:
- $a = R_{in} R_2 C_1 C_2$
- $b = R_{in} C_1 + R_{in} C_2 + R_2 C_2$

## Frequency Response
The magnitude response $|H(j\omega)|$ is:
$$|H(j\omega)| = \frac{1}{\sqrt{(1 - a\omega^2)^2 + (b\omega)^2}}$$
The phase response $\phi(\omega)$ is:
$$\phi(\omega) = -\arctan\left(\frac{b\omega}{1 - a\omega^2}\right)$$

## Measurement Strategies

### 1. Dominant Time Constant (OCTC)
Since the system is heavily overdamped ($b^2 \gg 4a$), the step response is dominated by a single time constant $\tau \approx b$.
$$\tau = R_{in}(C_1 + C_2) + R_2 C_2$$
Solving for $R_2$:
$$R_2 = \frac{\tau - R_{in}(C_1 + C_2)}{C_2}$$

### 2. Gain at Specific Frequency
By measuring $V_{pp\_out}$ and $V_{pp\_in}$ at a known frequency $f$:
$$G = \frac{V_{pp\_out}}{V_{pp\_in}}$$
$$G^2 = \frac{1}{(1 - a\omega^2)^2 + (b\omega)^2}$$
Substitute $a = (R_{in} C_1 C_2) R_2$ and $b = R_{in}(C_1+C_2) + C_2 R_2$.
This is a quadratic equation in $R_2$ which can be solved.

### 3. Phase Shift
Measuring the time delay $\Delta t$ between $V_{in}$ and $V_{A0}$:
$$\phi = -2\pi f \Delta t$$
$$\tan(-\phi) = \frac{b\omega}{1 - a\omega^2}$$
Again, this leads to a solvable equation for $R_2$.

## Constraints & Realities
- **No Resonance:** Passive RC circuits cannot have a gain peak (>1). The "resonance" terminology in some files refers to the frequency where phase is $-90^\circ$ (which occurs when $1 - a\omega^2 = 0$) or simply a characteristic frequency.
- **Internal Resistance:** $R_0 = 250\Omega$ is a significant part of the input impedance and must be included.
- **Sampling Rate:** Arduino `analogRead` is ~100µs. High frequency measurements require optimized code or PWM-based steady-state analysis.
- **Precision:** Use Kahan summation for averaging and Kalman filtering for noise reduction.

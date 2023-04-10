# RC_resonance_finder
finds RC resonance frequency and resistance 
written by Bing

The code works by using a phase shift method to find the resonance frequency and resistance of an RC circuit. The code does the following steps:

- It defines some constants and variables for the pins, the capacitance, the frequency, the period, the duty cycle, the phase shift, the resonance frequency, the resistance, and some flags and timers.
- It initializes the pins and the serial monitor in the setup function.
- It calls the findResonance function in the main loop and waits for 60 seconds before repeating.
- The findResonance function starts with a low frequency and a 50% duty cycle and sets the minimum phase shift to 90 degrees or pi/2 radians. It also sets the found flag to false.
- The function then enters a while loop that runs until resonance is found or frequency is too high. In each iteration of the loop, it does the following:
  - It records the start time of the output signal and generates a square wave with the given frequency and duty cycle on the output pin.
  - It reads and converts the voltage across the capacitor on the input pin and updates the maximum and minimum voltages accordingly.
  - It detects the rising and falling edges of the input signal and records the zero crossing time when the voltage is above half of the maximum voltage.
  - It calculates the phase shift between the output and input signals by using the formula: phaseShift = (zeroTime - startTime) / (period / (2.0 * PI)).
  - It updates the minimum phase shift and resonance frequency if the phase shift is lower than minPhaseShift.
  - It sets the found flag to true and breaks out of the inner loop if the phase shift is close to 90 degrees or pi/2 radians.
  - If resonance is not found yet, it increases the frequency by 10% and recalculates the period. It also prints an error message and breaks out of the outer loop if frequency is too high.
  - If resonance is found, it calculates the resistance by using the formula: resistance = 1.0 / (2.0 * PI * frequency * capacitance). It also prints the resonance frequency and resistance values on the serial monitor.

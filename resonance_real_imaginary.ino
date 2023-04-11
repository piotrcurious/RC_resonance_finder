
// Define constants and variables
const int outputPin = 10; // The digital output pin connected to the RC circuit
const int inputPin = A0; // The analog input pin that reads the voltage across the capacitor
const float capacitance = 10e-6; // The capacitance of the capacitor in farads
const float maxCurrent = 0.02; // The maximum current in amperes
const float Vin = 5.0; // The input voltage in volts
const int interval = 60000; // The interval between measurements in milliseconds
float resistance; // The resistance of the resistor in ohms
float frequency; // The frequency of the PWM signal in hertz
float dutyCycle; // The duty cycle of the PWM signal as a fraction
float Vout; // The output voltage across the capacitor in volts
float lastFrequency; // The last frequency used for binary search
float lowerBound; // The lower bound of the binary search range
float upperBound; // The upper bound of the binary search range
float tolerance; // The tolerance for the binary search convergence
bool firstTime; // A flag to indicate if this is the first measurement

// Define a Kalman filter structure
struct KalmanFilter {
  float Q; // Process noise covariance
  float R; // Measurement noise covariance
  float P; // Estimate error covariance
  float K; // Kalman gain
  float X; // State estimate
};

// Initialize a Kalman filter for resistance estimation
KalmanFilter kf = {0.001, 0.1, 1, 0, 0};

// A function to update the Kalman filter with a new measurement
void updateKalmanFilter(KalmanFilter *kf, float measurement) {
  // Predict the next state estimate and error covariance
  kf->X = kf->X;
  kf->P = kf->P + kf->Q;

  // Calculate the Kalman gain
  kf->K = kf->P / (kf->P + kf->R);

  // Update the state estimate and error covariance with the measurement
  kf->X = kf->X + kf->K * (measurement - kf->X);
  kf->P = (1 - kf->K) * kf->P;
}

// A function to calculate the resistance from the voltage and frequency using Ohm's law and Kirchhoff's law
float calculateResistance(float Vout, float frequency) {
  // Calculate the impedance of the RC circuit using the formula Z = R + jwC, where j is the imaginary unit and w is the angular frequency
  float Z = Vin / maxCurrent;

  // Calculate the real part of the impedance, which is the resistance R
  float R = Z * Vout / Vin;

  // Calculate the imaginary part of the impedance, which is the reactance Xc = -1 / (wC)
  float Xc = -1 / (2 * PI * frequency * capacitance);

  // Subtract the reactance from the resistance to get the value of the resistor
  float resistor = R - Xc;

  // Return the resistor value
  return resistor;
}

// A function to set up the Arduino pins and variables
void setup() {
  // Set the output pin as an output and the input pin as an input
  pinMode(outputPin, OUTPUT);
  pinMode(inputPin, INPUT);

  // Set some initial values for frequency, duty cycle and tolerance
  frequency = 500;
  dutyCycle = 0.5;
  tolerance = 0.01;

  // Set a flag to indicate that this is the first measurement
  firstTime = true;

  // Start serial communication at 9600 baud rate
  Serial.begin(9600);
}

// A function to loop through measurements and calculations every interval
void loop() {
  // Check if it is time to measure the resistance
  static unsigned long lastTime = 0;
  unsigned long currentTime = millis();
  if (currentTime - lastTime >= interval || firstTime) {
    // Set the flag to false after the first measurement
    firstTime = false;

    // Reset the last time
    lastTime = currentTime;

    // Generate a PWM signal with the given frequency and duty cycle
    analogWrite(outputPin, 255 * dutyCycle);
    delay(1000 / frequency);

    // Read the analog voltage across the capacitor
    int raw = analogRead(inputPin);
    Vout = raw * Vin / 1024.0;

    // Calculate the resistance from the voltage and frequency
    resistance = calculateResistance(Vout, frequency);

    // Update the Kalman filter with the resistance measurement
    updateKalmanFilter(&kf, resistance);

    // Print the resistance estimate and the frequency to the serial monitor
    Serial.print("Resistance: ");
    Serial.print(kf.X);
    Serial.print(" Ohms, Frequency: ");
    Serial.print(frequency);
    Serial.println(" Hz");

    // Perform a binary search to find the resonant frequency that maximizes the output voltage
    if (Vout > Vin * 0.9) {
      // If the output voltage is close to the input voltage, we have found the resonant frequency
      Serial.println("Resonant frequency found!");
      // Save the resonant frequency for the next measurement
      lastFrequency = frequency;
      // Reset the binary search bounds
      lowerBound = 0;
      upperBound = 0;
    } else {
      // If the output voltage is not close to the input voltage, we need to adjust the frequency
      if (lowerBound == 0 && upperBound == 0) {
        // If this is the first iteration of the binary search, use the last frequency as a seed
        lowerBound = lastFrequency * 0.5;
        upperBound = lastFrequency * 1.5;
      }
      // Calculate the midpoint of the binary search range
      float midPoint = (lowerBound + upperBound) / 2;
      if (frequency < midPoint) {
        // If the current frequency is lower than the midpoint, increase it and set it as the new lower bound
        frequency = midPoint + (upperBound - midPoint) * tolerance;
        lowerBound = midPoint;
      } else {
        // If the current frequency is higher than the midpoint, decrease it and set it as the new upper bound
        frequency = midPoint - (midPoint - lowerBound) * tolerance;
        upperBound = midPoint;
      }
      // Check if the binary search range is too small to continue
      if (upperBound - lowerBound < tolerance) {
        // If so, use the midpoint as the best estimate of the resonant frequency
        Serial.println("Resonant frequency not found, using best estimate.");
        frequency = midPoint;
        // Save the resonant frequency for the next measurement
        lastFrequency = frequency;
        // Reset the binary search bounds
        lowerBound = 0;
        upperBound = 0;
      }
    }
  }
}



//Source: Conversation with Bing, 4/11/2023(1) Arduino Resistance Measurement - Instructables. https://www.instructables.com/Arduino-Resistance-Measurement/ Accessed 4/11/2023.
//(2) A question about resistance measurement with arduino. https://arduino.stackexchange.com/questions/28222/a-question-about-resistance-measurement-with-arduino Accessed 4/11/2023.
//(3) Arduino Resistance Meter- OHM Meter : 4 Steps - Instructables. https://www.instructables.com/Arduino-Resistance-Meter-OHM-Meter/ Accessed 4/11/2023.
//(4) Arduino RC Circuit: PWM to Analog DC : 10 Steps - Instructables. https://www.instructables.com/Arduino-RC-Circuit-PWM-to-analog-DC/ Accessed 4/11/2023.
//(5) RC Circuit Calculator. https://www.omnicalculator.com/physics/rc-circuit Accessed 4/11/2023.
//(6) 15.6: Resonance in an AC Circuit - Physics LibreTexts. https://phys.libretexts.org/Bookshelves/University_Physics/Book%3A_University_Physics_(OpenStax)/Book%3A_University_Physics_II_-_Thermodynamics_Electricity_and_Magnetism_(OpenStax)/15%3A_Alternating-Current_Circuits/15.06%3A_Resonance_in_an_AC_Circuit Accessed 4/11/2023.
//(7) RC circuit - Wikipedia. https://en.wikipedia.org/wiki/RC_circuit Accessed 4/11/2023.

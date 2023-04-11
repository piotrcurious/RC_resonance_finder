
// Define pin numbers
const int DIGITAL_PIN = 3; // Digital pin for generating pulses
const int ANALOG_PIN = A0; // Analog pin for measuring voltage

// Define component values
const float R0 = 250; // Internal resistance of digital pin in ohms
const float C1 = 0.5e-6; // Capacitance of first capacitor in farads
const float C2 = 10e-6; // Capacitance of second capacitor in farads
const float Vcc = 5; // Supply voltage in volts
const float epsilon = 0.01; // Accuracy threshold for binary search in hertz

// Define initial values for binary search
float fmin = 1; // Lower bound for frequency range in hertz
float fmax = 1000; // Upper bound for frequency range in hertz
float fmid; // Midpoint for frequency range in hertz
float Vc2; // Voltage across second capacitor in volts

// Define initial values for Kalman filter
float x = 1000; // Initial estimate of unknown resistance in ohms
float P = 1000000; // Initial error covariance of estimate in ohms^2
float Q = 1000; // Process noise covariance in ohms^2
float R = 0.01; // Measurement noise covariance in volts^2

// Define timer variable
unsigned long timer = 0; // Timer for calling function in milliseconds

// Generate a pulse with a given frequency and length
void pulse(float freq, float len) {
  // Calculate period and half-period in microseconds
  float period = 1000000 / freq;
  float half_period = period / 2;
  
  // Set digital pin to HIGH for half-period
  digitalWrite(DIGITAL_PIN, HIGH);
  delayMicroseconds(half_period);
  
  // Set digital pin to LOW for half-period
  digitalWrite(DIGITAL_PIN, LOW);
  delayMicroseconds(half_period);
  
  // Repeat until length is reached
  len -= period;
  if (len > 0) {
    // Call pulse function recursively
    pulse(freq, len);
  }
}

// Measure voltage across second capacitor using analog pin
float measure() {
  // Read analog value and convert to voltage
  int analog = analogRead(ANALOG_PIN);
  float voltage = analog * Vcc / 1024;
  
  // Return voltage
  return voltage;
}

// Perform binary search to find resonant frequency
float binary_search() {
  // Calculate midpoint of frequency range
  fmid = (fmin + fmax) / 2;
  
  // Generate a pulse with midpoint frequency and measure voltage
  pulse(fmid, 0.1);
  Vc2 = measure();
  
  // Generate a pulse with midpoint plus one frequency and measure voltage
  pulse(fmid + 1, 0.1);
  float Vc2_plus = measure();
  
  // Compare voltages and update frequency range
  if (Vc2_plus > Vc2) {
    // Maximum voltage is in upper half of range
    fmin = fmid;
  }
  else {
    // Maximum voltage is in lower half of range
    fmax = fmid;
  }
  
  // Check if accuracy threshold is reached
  if (fmax - fmin < epsilon) {
    // Return midpoint as resonant frequency
    return fmid;
  }
  else {
    // Repeat binary search recursively
    return binary_search();
  }
}

// Calculate impedance of second capacitor using frequency and capacitance
float impedance(float freq, float cap) {
  // Calculate angular frequency in radians per second
  float w = 2 * PI * freq;
  
  // Calculate impedance in ohms using formula Zc = 1 / (j * w * C)
  float Zc = 1 / (w * cap);
  
  // Return impedance
  return Zc;
}

// Perform Kalman filter to estimate unknown resistance
float kalman_filter() {
  // Prediction step: use model x(k+1) = x(k) and add process noise Q
  x = x; // No change in estimate
  P = P + Q; // Increase error covariance
  
  // Update step: use measurement model Vc2 = Vcc * Zc2 / (R0 + x + Zc2)
  
  // Calculate residual y as difference between actual and predicted measurement
  float y = Vc2 - (Vcc * impedance(fmid, C2) / (R0 + x + impedance(fmid, C2)));
  
  // Calculate residual covariance S as sum of predicted error covariance and measurement noise R
  float S = P + R;
  
  // Calculate Kalman gain K as ratio of predicted error covariance and residual covariance
  float K = P / S;
  
  // Update state estimate x by adding product of Kalman gain and residual
  x = x + K * y;
  
  // Update state covariance P by subtracting product of Kalman gain and residual covariance
  P = P - K * S;
  
  // Output step: return updated state estimate x as optimal estimate of unknown resistance
  return x;
}

// Call all functions every 60 seconds to check if unknown resistance has changed
void loop() {
  // Check if 60 seconds have passed since last call
  unsigned long current_time = millis();
  if (current_time - timer >= 60000) {
    // Reset timer
    timer = current_time;
    
    // Perform binary search to find resonant frequency
    float freq = binary_search();
    
    // Perform Kalman filter to estimate unknown resistance
    float res = kalman_filter();
    
    // Print results to serial monitor
    Serial.print("Resonant frequency: ");
    Serial.print(freq);
    Serial.println(" Hz");
    Serial.print("Unknown resistance: ");
    Serial.print(res);
    Serial.println(" Ohms");
  }
}

// Initialize serial communication and pin modes
void setup() {
  // Start serial communication at 9600 baud rate
  Serial.begin(9600);
  
  // Set digital pin as output
  pinMode(DIGITAL_PIN, OUTPUT);
  
  // Set analog pin as input
  pinMode(ANALOG_PIN, INPUT);
}

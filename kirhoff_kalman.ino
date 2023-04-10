
// Define constants and variables
#define OUTPUT_PIN 13 // Digital output pin connected to RC circuit
#define INPUT_PIN A0 // Analog input pin connected to capacitor
#define CAPACITANCE 0.00001 // Capacitance in farads (10 uF)
#define MAX_CURRENT 0.02 // Maximum current in amperes (20 mA)
#define VOLTAGE 5 // Voltage in volts
#define MIN_PULSE 1 // Minimum pulse length in microseconds
#define MAX_PULSE 10000 // Maximum pulse length in microseconds
#define TOLERANCE 0.01 // Tolerance for finding resonant frequency in hertz
#define INTERVAL 60000 // Interval for calling resistance finding function in milliseconds (60 seconds)

float internal_resistance; // Internal resistance of output pin in ohms
float resistor_value; // Resistor value in ohms
float last_frequency; // Last resonant frequency in hertz
float kalman_gain; // Kalman gain
float estimated_resistance; // Estimated resistance in ohms
float estimated_error; // Estimated error in ohms

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  
  // Initialize output pin as output and input pin as input
  pinMode(OUTPUT_PIN, OUTPUT);
  pinMode(INPUT_PIN, INPUT);
  
  // Calculate internal resistance of output pin using Ohm's law
  internal_resistance = VOLTAGE / MAX_CURRENT;
  
  // Initialize last resonant frequency as half of maximum pulse length
  last_frequency = 1.0 / (2 * MAX_PULSE * pow(10, -6));
  
  // Initialize Kalman filter parameters
  kalman_gain = 0;
  estimated_resistance = 0;
  estimated_error = 0;
}

void loop() {
  // Call resistance finding function every interval
  find_resistance();
  delay(INTERVAL);
}

void find_resistance() {
  // Define local variables for binary search algorithm
  float low = MIN_PULSE; // Lower bound of pulse length in microseconds
  float high = MAX_PULSE; // Upper bound of pulse length in microseconds
  float mid; // Middle point of pulse length in microseconds
  float frequency; // Frequency corresponding to pulse length in hertz
  float voltage; // Voltage across capacitor in volts
  
  // Define local variables for finding maximum voltage
  float max_voltage = 0; // Maximum voltage across capacitor in volts
  float max_frequency = 0; // Frequency corresponding to maximum voltage in hertz
  
  // Define local variables for Kalman filter update
  float measurement_resistance; // Measured resistance in ohms
  float measurement_error; // Measurement error in ohms
  
  
  // Perform binary search until frequency is within tolerance of last resonant frequency
  while (abs(frequency - last_frequency) > TOLERANCE) {
    // Calculate middle point of pulse length
    mid = (low + high) / 2;
    
    // Calculate frequency corresponding to pulse length using formula f = 1 / T, where T is the period
    frequency = 1.0 / (mid * pow(10, -6));
    
    // Send a pulse to the output pin with the middle point length
    digitalWrite(OUTPUT_PIN, HIGH);
    delayMicroseconds(mid);
    digitalWrite(OUTPUT_PIN, LOW);
    
    // Read the voltage across the capacitor from the input pin and convert it to volts using formula V = R * I, where R is the internal resistance and I is the current
    voltage = analogRead(INPUT_PIN) * (internal_resistance / 1023.0);
    
    // Compare the voltage with the maximum voltage and update accordingly
    if (voltage > max_voltage) {
      max_voltage = voltage;
      max_frequency = frequency;
    }
    
    // Adjust the bounds of pulse length based on the sign of the difference between frequency and last resonant frequency
    if (frequency - last_frequency > 0) {
      low = mid;
    }
    else {
      high = mid;
    }
  }
  
  // Update the last resonant frequency with the maximum frequency
  last_frequency = max_frequency;
  
  // Calculate the measured resistance using Kirchhoff's law: V = IR + Q/C, where V is the voltage, I is the current, R is the resistance, Q is the charge and C is the capacitance
  measurement_resistance = (VOLTAGE - max_voltage) * CAPACITANCE / (max_voltage * max_frequency);
  
  // Calculate the measurement error using the pulse length and the capacitor voltage as noise estimators
  measurement_error = mid * voltage * pow(10, -6);
  
  // Update the Kalman filter parameters using the formulas:
  // K = P / (P + R)
  // X = X + K * (Z - X)
  // P = (1 - K) * P
  // Where K is the Kalman gain, P is the estimated error, R is the measurement error, X is the estimated resistance and Z is the measured resistance
  kalman_gain = estimated_error / (estimated_error + measurement_error);
  estimated_resistance = estimated_resistance + kalman_gain * (measurement_resistance - estimated_resistance);
  estimated_error = (1 - kalman_gain) * estimated_error;
  
  // Print the estimated resistance to the serial monitor
  Serial.print("The estimated resistance is: ");
  Serial.print(estimated_resistance);
  Serial.println(" ohms");
}

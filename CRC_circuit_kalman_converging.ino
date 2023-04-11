
// Include BasicLinearAlgebra library for matrix operations
#include <BasicLinearAlgebra.h>
using namespace BLA;

// Define pin numbers
const int DIGITAL_PIN = 3; // Digital pin for generating pulses
const int ANALOG_PIN1 = A0; // Analog pin for measuring voltage across C2
const int ANALOG_PIN2 = A1; // Analog pin for measuring voltage across C1

// Define component values
const float R0 = 250; // Internal resistance of digital pin in ohms
const float R1 = 1; // Resistance of first resistor in ohms
const float C1 = 0.5e-6; // Capacitance of first capacitor in farads
const float C2 = 10e-6; // Capacitance of second capacitor in farads
const float Vcc = 5; // Supply voltage in volts
const float epsilon = 0.01; // Accuracy threshold for binary search in hertz
const float tolerance = 0.01; // Tolerance threshold for convergence in ohms

// Define initial values for binary search
float fmin = 1; // Lower bound for frequency range in hertz
float fmax = 1000; // Upper bound for frequency range in hertz
float fmid; // Midpoint for frequency range in hertz
float Vc2; // Voltage across second capacitor in volts

// Define initial values for Kalman filter
float x = 1000; // Initial estimate of unknown resistance in ohms
Matrix<2> P; // Initial error covariance of estimate in ohms^2
P(0,0) = 1000000;
P(0,1) = 0;
P(1,0) = 0;
P(1,1) = 1000000;
Matrix<2> Q; // Process noise covariance in ohms^2
Q(0,0) = 1000;
Q(0,1) = 0;
Q(1,0) = 0;
Q(1,1) = 0.01;
float R1_noise = 0.01; // Variability of R1 due to temperature or humidity in ohms

// Define arrays for storing impedance and noise values of first RC circuit
float Zc1[10]; // Impedance values in ohms
float Vc1_noise[10]; // Noise values in volts

// Generate a pulse with a given frequency and length
void pulse(float freq, float len) {
  // Calculate period and half-period in microseconds
  float period = 1000000 / freq;
  float half_period = period / 2;
  
  // Repeat until length is zero or negative
  while (len > 0) {
    // Set digital pin to HIGH for half-period
    digitalWrite(DIGITAL_PIN, HIGH);
    delayMicroseconds(half_period);
    
    // Set digital pin to LOW for half-period
    digitalWrite(DIGITAL_PIN, LOW);
    delayMicroseconds(half_period);
    
    // Subtract period from length
    len -= period;
    
    // Break if length becomes negative
    if (len < 0) {
      break;
    }
  }
}


// Measure voltage across second capacitor using analog pin
float measure() {
  // Read analog value and convert to voltage
  int analog = analogRead(ANALOG_PIN1);
  float voltage = analog * Vcc / 1024;
  
  // Return voltage
  return voltage;
}

// Measure impedance and noise of first RC circuit using analog pin
void measure_impedance(float freq) {
  // Define array for storing voltage readings
  float Vc1[10];
  
  // Read voltage across C1 10 times and store in array
  for (int i = 0; i < 10; i++) {
    // Generate a pulse with given frequency and measure voltage
    pulse(freq, 0.1);
    Vc1[i] = analogRead(ANALOG_PIN2) * Vcc / 1024;
    
    // Wait for 10 milliseconds before next reading
    delay(10);
  }
  
  // Calculate average and standard deviation of voltage readings
  float sum = 0;
  float sq_sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += Vc1[i];
    sq_sum += Vc1[i] * Vc1[i];
  }
  float mean = sum / 10;
  float std = sqrt(sq_sum / 10 - mean * mean);
  
  // Calculate impedance of first RC circuit using Ohm's law and Kirchhoff's law
  float I = (Vcc - mean) / (R0 + R1); // Current in amps
  float Zc1 = mean / I; // Impedance in ohms
  
  // Store impedance and noise values in arrays
  Zc1[i] = Zc1;
  Vc1_noise[i] = std;
}

// Perform binary search to find resonant frequency
float binary_search() {
  // Calculate initial midpoint of frequency range
  fmid = (fmin + fmax) / 2;
  
  // Generate a pulse with initial midpoint frequency and measure voltage
  pulse(fmid, 0.1);
  Vc2 = measure();
  
  // Repeat until accuracy threshold is reached
  while (fmax - fmin >= epsilon) {
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
    
    // Calculate new midpoint of frequency range
    fmid = (fmin + fmax) / 2;
    
    // Generate a pulse with new midpoint frequency and measure voltage
    pulse(fmid, 0.1);
    Vc2 = measure();
  }
  
  // Return midpoint as resonant frequency
  return fmid;
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
  
  // Update step: use measurement model Vc2 = Vcc * Zc2 / (R0 + x + Zc2) and Zc1 = R0 + x + R1 + 1 / (j * w * C1)
  
  // Calculate residuals y as differences between actual and predicted measurements
  Matrix<2> y; // Residual vector
  y(0) = Vc2 - (Vcc * impedance(fmid, C2) / (R0 + x + impedance(fmid, C2))); // Residual for Vc2
  y(1) = Zc1 - (R0 + x + R1 + impedance(fmid, C1)); // Residual for Zc1
  
  // Calculate residual covariances S as sums of predicted error covariances and measurement noises R
  Matrix<2> S; // Residual covariance matrix
  S(0,0) = P(0,0) + Vc2_noise; // Residual covariance for Vc2
  S(0,1) = P(0,1); // Cross-covariance between Vc2 and Zc1
  S(1,0) = P(1,0); // Cross-covariance between Zc1 and Vc2
  S(1,1) = P(1,1) + Vc1_noise; // Residual covariance for Zc1
  
  // Calculate Kalman gain K as ratio of predicted error covariance and residual covariance
  Matrix<2> K; // Kalman gain matrix
  K = P * Inverse(S); // Matrix multiplication and inversion
  
  // Update state estimate x by adding product of Kalman gain and residual
  x = x + K * y; // Matrix multiplication and addition
  
  // Update state covariance P by subtracting product of Kalman gain and residual covariance
  P = P - K * S; // Matrix multiplication and subtraction
  
  // Output step: return updated state estimate x as optimal estimate of unknown resistance
  return x;
}

// Perform convergence to refine unknown resistance estimate
float converge(float res) {
  // Define variable for storing previous resistance estimate
  float prev_res = res;
  
  // Define variable for storing difference between estimates
  float diff = tolerance;
  
  // Repeat until difference is less than tolerance
  while (diff >= tolerance) {
    // Print current estimate and difference to serial monitor
    Serial.print("Current estimate: ");
    Serial.print(res);
    Serial.print(" Ohms, Difference: ");
    Serial.print(diff);
    Serial.println(" Ohms");
    
    // Calculate resonant frequency of second RC circuit using estimate
    float freq = 1 / (2 * PI * sqrt(res * C2));
    
    // Measure impedance and noise of first RC circuit using frequency
    measure_impedance(freq);
    
    // Perform Kalman filter to calculate new estimate
    res = kalman_filter();
    
    // Calculate difference between new and previous estimates
    diff = abs(res - prev_res);
    
    // Update previous estimate
    prev_res = res;
  }
  
  // Return converged estimate
  return res;
}

// Run function once when Arduino is powered on or reset
void setup() {
  // Start serial communication at 9600 baud rate
  Serial.begin(9600);
  
  // Set digital pin as output
  pinMode(DIGITAL_PIN, OUTPUT);
  
  // Set analog pins as input
  pinMode(ANALOG_PIN1, INPUT);
  pinMode(ANALOG_PIN2, INPUT);
  
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
  
  // Perform convergence to refine unknown resistance estimate
  res = converge(res);
  
  // Print final result to serial monitor
  Serial.print("Final estimate: ");
  Serial.print(res);
  Serial.println(" Ohms");
}

// Do nothing in loop
void loop() {
}

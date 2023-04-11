
// Arduino code for RC circuit resistance measurement with Kalman filter
// Based on https://www.instructables.com/Arduino-Resistance-Measurement/ and https://github.com/rfetick/Kalman

#include <Kalman.h> // Kalman filter library
//There are several Kalman filter libraries for Arduino that you can use for your project, such as Kalman Filter Library¹, Kalman² and SimpleKalmanFilter³. They all claim to be compatible with all architectures, so you should be able to use them on any Arduino board. However, I cannot guarantee that they will work without any errors, as there may be some compatibility issues with different versions of Arduino IDE or BasicLinearAlgebra library. You may need to test and debug them yourself or contact the authors for support. I hope this helps.

//Source: Conversation with Bing, 4/11/2023(1) Kalman Filter Library - Arduino Reference. http://reference.arduino.cc/reference/en/libraries/kalman-filter-library/ Accessed 4/11/2023.
//(2) Kalman - Arduino Reference. https://reference.arduino.cc/reference/en/libraries/kalman/ Accessed 4/11/2023.
//(3) SimpleKalmanFilter - Arduino Reference. https://reference.arduino.cc/reference/en/libraries/simplekalmanfilter/ Accessed 4/11/2023.

// Define the pins for the RC circuit
const int analogPin = A0; // Analog input pin for measuring voltage across capacitor
const int digitalPin = 9; // Digital output pin for sending pulses to RC circuit

// Define the parameters for the RC circuit
const float Vin = 5.0; // Input voltage in volts
const float C = 10e-6; // Capacitance in farads
const float Imax = 0.02; // Maximum current in amperes
const float Rint = Vin / Imax; // Internal resistance of digital pin in ohms

// Define the parameters for the Kalman filter
const int Nstate = 2; // Number of state variables (resistance and frequency)
const int Nobs = 1; // Number of observation variables (voltage)
const float Q = 0.01; // Process noise covariance
const float R = 0.1; // Measurement noise covariance

// Create a Kalman filter object
Kalman KF(Nstate, Nobs);

// Define some global variables
float R1; // Resistance of RC circuit in ohms
float f1; // Frequency of pulses in hertz
float Vout; // Voltage across capacitor in volts
float T1; // Pulse duration in seconds
float T2; // Pulse interval in seconds
unsigned long t0; // Time of last pulse in milliseconds
unsigned long t1; // Time of current pulse in milliseconds

void setup() {
  Serial.begin(9600); // Initialize serial communication
  pinMode(digitalPin, OUTPUT); // Set digital pin as output
  digitalWrite(digitalPin, LOW); // Set digital pin low initially
  
  // Initialize the state vector with some initial guesses
  KF.X(0) = 1000.0; // Initial guess for resistance in ohms
  KF.X(1) = 100.0; // Initial guess for frequency in hertz
  
  // Initialize the state transition matrix with the state equations
  KF.F(0,0) = 1.0; // Resistance does not change with time
  KF.F(0,1) = 0.0; // Resistance does not depend on frequency
  KF.F(1,0) = 0.0; // Frequency does not depend on resistance
  KF.F(1,1) = 1.0; // Frequency does not change with time
  
  // Initialize the observation matrix with the observation equation
  KF.H(0,0) = -Vin / (Rint + KF.X(0)); // Voltage depends on resistance inversely
  KF.H(0,1) = Vin * C * PI * KF.X(1); // Voltage depends on frequency directly
  
  // Initialize the process noise covariance matrix with some constant values
  KF.Q(0,0) = Q; 
  KF.Q(0,1) = 0.0;
  KF.Q(1,0) = 0.0;
  KF.Q(1,1) = Q;
  
  // Initialize the measurement noise covariance matrix with some constant values
  KF.R(0,0) = R;
  
}

void loop() {
  
  t1 = millis(); // Get current time
  
  if (t1 - t0 >= T2 * 1000) { // If pulse interval has elapsed
    
    t0 = t1; // Update last pulse time
    
    R1 = KF.X(0); // Get current estimate of resistance from state vector
    f1 = KF.X(1); // Get current estimate of frequency from state vector
    
    Serial.print("R1: "); 
    Serial.print(R1); 
    Serial.print(" ohms, ");
    Serial.print("f1: "); 
    Serial.print(f1); 
    Serial.println(" hertz");
    
    T2 = constrain(10 / f1, 0.01, 1.0); // Calculate pulse interval based on frequency, limit to 10-1000 ms
    T1 = constrain(0.5 / f1, 0.001, 0.5); // Calculate pulse duration based on frequency, limit to 1-500 ms
    
    digitalWrite(digitalPin, HIGH); // Set digital pin high to start pulse
    delay(T1 * 1000); // Wait for pulse duration
    digitalWrite(digitalPin, LOW); // Set digital pin low to end pulse
    
    Vout = analogRead(analogPin) * Vin / 1024.0; // Read analog voltage across capacitor and convert to volts
    
    Serial.print("Vout: "); 
    Serial.print(Vout); 
    Serial.println(" volts");
    
    KF.Z(0) = Vout; // Set observation vector with measured voltage
    
    KF.predict(); // Predict the next state vector based on state transition matrix
    KF.update(); // Update the state vector based on observation vector and Kalman gain
    
    // Adjust the observation matrix based on the updated state vector
    KF.H(0,0) = -Vin / (Rint + KF.X(0)); 
    KF.H(0,1) = Vin * C * PI * KF.X(1);
    
  }
  
}


//Source: Conversation with Bing, 4/11/2023(1) Arduino Resistance Measurement - Instructables. https://www.instructables.com/Arduino-Resistance-Measurement/ Accessed 4/11/2023.
//(2) A question about resistance measurement with arduino. https://arduino.stackexchange.com/questions/28222/a-question-about-resistance-measurement-with-arduino Accessed 4/11/2023.
//(3) Arduino Resistance Measurement | Circuits4you.com. https://circuits4you.com/2016/05/13/arduino-resistance-measurement/ Accessed 4/11/2023.
//(4) Kalman Filter Library - Arduino Reference. http://reference.arduino.cc/reference/en/libraries/kalman-filter-library/ Accessed 4/11/2023.
//(5) rfetick/Kalman: Implement Kalman filter for your Arduino projects - Github. https://github.com/rfetick/Kalman Accessed 4/11/2023.
//(6) GitHub - pronenewbits/Embedded_EKF_Library: A compact Extended Kalman .... https://github.com/pronenewbits/Embedded_EKF_Library Accessed 4/11/2023.

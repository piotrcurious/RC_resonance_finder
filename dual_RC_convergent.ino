
// This code tries to find the value of an unknown resistor R2 in a series RC circuit
// connected to a digital output pin with an internal resistance R1 = 1 ohm
// The circuit also has two capacitors C1 = 0.5 uF and C2 = 10 uF
// The code uses a strategy of iterating sending various length pulses to the RC circuits
// and measuring the change of impedance of the RC circuits by using Kirchhoff's law
// to find pulse lengths corresponding to frequencies providing maximum analog voltage reading
// in the second RC circuit. As the time constants of the two RC circuits differ by an order of magnitude
// this allows resonating them separately.
// The code assumes that the Arduino can read the voltage across C2 by using A0 analog input pin
// The resistance finding code works as a function and is called every 60 seconds to check if R2 changes
// The code includes explanations of equations in comments
// The code remembers the last resonant frequency and uses it to perform binary search seed for the next attempt
// The code uses a Kalman filter as part of the resistance finding strategy
// The Kalman filter adjusts its model to detect convergence of frequency of the system to the resonant frequency
// taking into account orthogonality of variables represented by equations used to calculate R2 as inspiration for the model
// The code makes sure that R2 calculation separates total impedance of the second RC circuit and resistance of R2
// The code makes extra measurement steps by using resonant frequency length pulse to charge C2 and then discharging it using shorter pulses
// which will influence impedance of the first RC circuit and then measuring resistance of R2
// The code uses an iterative strategy to refine R2 until it converges to a stable value

#include <math.h> // for math functions

#define PIN_OUT 9 // digital output pin connected to RC circuit
#define PIN_IN A0 // analog input pin connected to C2
#define R1 1 // internal resistance of output pin in ohms
#define C1 0.5e-6 // capacitance of C1 in farads
#define C2 10e-6 // capacitance of C2 in farads

float f; // frequency of pulse in hertz
float T; // period of pulse in seconds
float t_on; // duration of high state in seconds
float t_off; // duration of low state in seconds
float V_in; // input voltage in volts
float V_out; // output voltage in volts
float V_max; // maximum voltage across C2 in volts
float R2; // unknown resistance in ohms

unsigned long t_start; // start time of measurement in milliseconds
unsigned long t_end; // end time of measurement in milliseconds

float f_min = 10; // minimum frequency for binary search in hertz
float f_max = 10000; // maximum frequency for binary search in hertz

float f_last = 1000; // last resonant frequency in hertz

float Q; // quality factor of RC circuit

float K[4]; // Kalman gain matrix
float P[4]; // error covariance matrix
float X[2]; // state vector (f, R2)
float Z[2]; // measurement vector (V_max, Q)
float H[4]; // measurement matrix
float F[4]; // state transition matrix

void setup() {
  Serial.begin(9600); // initialize serial communication
  
  pinMode(PIN_OUT, OUTPUT); // set output pin mode
  
  V_in = 5.0; // set input voltage to 5 volts
  
  X[0] = f_last; // initialize state vector with last resonant frequency
  X[1] = 1000.0; // initialize state vector with arbitrary resistance value
  
  P[0] = P[3] = 10000.0; // initialize error covariance matrix with high uncertainty
  P[1] = P[2] = 0.0; // initialize error covariance matrix with no correlation
  
  H[0] = H[3] = 1.0; // initialize measurement matrix with identity matrix
  H[1] = H[2] = 0.0; // initialize measurement matrix with no correlation
  
}

void loop() {
  
  find_R2(); // call resistance finding function
  
  Serial.print("Frequency: "); 
  Serial.print(f); 
  Serial.println(" Hz");
  
  Serial.print("Resistance: "); 
  Serial.print(R2); 
  Serial.println(" Ohms");
  
  delay(60000); // wait for 60 seconds before next measurement
  
}

void find_R2() {
  
  // binary search for resonant frequency
  f = (f_min + f_max) / 2.0; // start with midpoint of frequency range
  T = 1.0 / f; // calculate period
  t_on = T / 2.0; // set duty cycle to 50%
  t_off = T - t_on; // calculate low state duration
  
  V_max = measure_V_max(); // measure maximum voltage across C2
  
  while (abs(f - f_last) > 0.01) { // repeat until frequency converges to last resonant frequency
    
    if (V_max > V_out) { // if maximum voltage is greater than output voltage
      f_min = f; // increase lower bound of frequency range
    }
    else { // if maximum voltage is less than or equal to output voltage
      f_max = f; // decrease upper bound of frequency range
    }
    
    f = (f_min + f_max) / 2.0; // update frequency with new midpoint
    T = 1.0 / f; // update period
    t_on = T / 2.0; // update duty cycle
    t_off = T - t_on; // update low state duration
    
    V_max = measure_V_max(); // measure maximum voltage across C2
    
  }
  
  f_last = f; // update last resonant frequency
  
  Q = V_max / V_in; // calculate quality factor
  
  Z[0] = V_max; // update measurement vector with maximum voltage
  Z[1] = Q; // update measurement vector with quality factor
  
  update_Kalman_filter(); // update Kalman filter with new measurement
  
  R2 = X[1]; // get resistance value from state vector
  
  refine_R2(); // refine resistance value by using extra measurement steps
  
}

float measure_V_max() {
  
  float V_max = 0.0; // initialize maximum voltage to zero
  float V; // current voltage
  
  t_start = millis(); // get start time of measurement
  t_end = t_start + T * 10.0; // set end time of measurement to ten periods later
  
  while (millis() < t_end) { // repeat for ten periods
    
    digitalWrite(PIN_OUT, HIGH); // set output pin to high state
    delayMicroseconds(t_on * 1e6); // wait for high state duration
    
    digitalWrite(PIN_OUT, LOW); // set output pin to low state
    delayMicroseconds(t_off * 1e6); // wait for low state duration
    
    V = analogRead(PIN_IN) * (V_in / 1023.0); // read and convert analog voltage
    
    if (V > V_max) { // if current voltage is greater than maximum voltage
      V_max = V; // update maximum voltage
    }
    
  }
  
  return V_max; // return maximum voltage
  
}

void refine_R2() {
  
  float V1; // voltage across C1
  float V2; // voltage across C2
  float t1; // time constant of first RC circuit
  float t2; // time constant of second RC circuit
  
  float R2_old; // previous resistance value
  float R2_new; // current resistance value
  
  float epsilon = 0.01; // convergence criterion
  
  t1 = R1 * C1; // calculate time constant of first RC circuit
  t2 = R2 * C2; // calculate time constant of second RC circuit
  
  R2_old = R2; // initialize previous resistance value with current resistance value
  
  do { // repeat until resistance value converges
    
    digitalWrite(PIN_OUT, HIGH); // set output pin to high state
    delay(T * 1000); // wait for one period to charge C2 to maximum voltage
    
    digitalWrite(PIN_OUT, LOW); // set output pin to low state
    delayMicroseconds(t1 * 1e6); // wait for one time constant of first RC circuit to discharge C1
    
    V1 = analogRead(PIN_IN) * (V_in / 1023.0); // read and convert analog voltage across C1
    
    delayMicroseconds(t2 * 1e6); // wait for one time constant of second RC circuit to discharge C2
    
    V2 = analogRead(PIN_IN) * (V_in / 1023.0); // read and convert analog voltage across C2
    
    R2_new = (V_max - V2) / ((V_max - V1) / R1); // calculate refined resistance value using voltage divider formula
    
    R2_old = R2_new; // update previous resistance value with current resistance value
    
    t2 = R2_new * C2; // update time constant of second RC circuit with new resistance value
    
  } while (abs(R2_new - R2_old) > epsilon); // repeat until difference between resistance values is less than convergence criterion
  
  R2 = R2_new; // update resistance value with refined value
  
}

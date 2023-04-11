
// Define constants and variables
#define PIN_OUT 13 // digital output pin
#define PIN_IN A0 // analog input pin
#define C 10e-6 // capacitance of RC circuit in farads
#define C0 0.1e-6 // capacitance of output pin in farads
#define R0 250 // internal resistance of output pin in ohms
#define I_MAX 0.02 // maximum current of output pin in amperes
#define V_MAX 5 // maximum voltage of output pin in volts
#define T_INTERVAL 60000 // time interval between resistance measurements in milliseconds

float R; // resistance of RC circuit in ohms
float f; // frequency of output pin in hertz
float T; // period of output pin in seconds
float V; // voltage across capacitor in volts
float V0; // voltage across output pin in volts
float Q; // charge on capacitor in coulombs
float Q0; // charge on output pin capacitor in coulombs

// Kalman filter variables
float x[2]; // state vector: x[0] = R, x[1] = f
float P[2][2]; // state covariance matrix: P[i][j] = cov(x[i], x[j])
float z[2]; // measurement vector: z[0] = V, z[1] = T
float H[2][2]; // measurement matrix: H[i][j] = d(z[i])/d(x[j])
float Rn[2][2]; // measurement noise covariance matrix: Rn[i][j] = cov(z[i], z[j])
float K[2][2]; // Kalman gain matrix: K[i][j] = P[i][j]*H[j][i]/(H[i][i]*P[i][i]+Rn[i][i])
float y[2]; // innovation vector: y[i] = z[i] - H[i][i]*x[i]
float I[2][2]; // identity matrix: I[i][j] = 1 if i == j, 0 otherwise

// Initialize the filter with some initial guesses
void initFilter() {
  x[0] = 1000; // initial resistance guess in ohms
  x[1] = 100; // initial frequency guess in hertz
  P[0][0] = 10000; // initial resistance variance in ohm^2
  P[0][1] = 0; // initial resistance-frequency covariance in ohm*hz
  P[1][0] = 0; // initial frequency-resistance covariance in hz*ohm
  P[1][1] = 10000; // initial frequency variance in hz^2
  Rn[0][0] = 0.01; // measurement noise variance for voltage in volt^2
  Rn[0][1] = 0; // measurement noise covariance for voltage and period in volt*s
  Rn[1][0] = 0; // measurement noise covariance for period and voltage in s*volt
  Rn[1][1] = 1e-6; // measurement noise variance for period in s^2
  I[0][0] = 1; // identity matrix element
  I[0][1] = 0; // identity matrix element
  I[1][0] = 0; // identity matrix element
  I[1][1] = 1; // identity matrix element  
}

// Update the filter with new measurements
void updateFilter() {
  
  // Calculate the measurement vector from the sensor readings
  V = analogRead(PIN_IN) * (V_MAX / 1023); // convert analog reading to voltage
  T = pulseIn(PIN_OUT, HIGH) * 1e-6; // convert pulse duration to seconds
  
  z[0] = V;
  z[1] = T;
  
  // Calculate the measurement matrix from the state vector using partial derivatives
  
  /* The equations relating the state and measurement vectors are:
     V = V_MAX * Q / (Q + Q0) where Q = C * V_MAX * (1 - exp(-T / (R * C))) and Q0 = C0 * V_MAX * (1 - exp(-T / (R0 + R + 1 / (2 * pi * f * C0)))) 
     T = 1 / f
     
     The partial derivatives are:
     d(V)/d(R) = V_MAX * Q * d(Q)/d(R) / (Q + Q0)^2 where d(Q)/d(R) = C * V_MAX * T * exp(-T / (R * C)) / (R * C)^2
     d(V)/d(f) = V_MAX * Q * d(Q)/d(f) / (Q + Q0)^2 where d(Q)/d(f) = -C0 * V_MAX * T^2 * exp(-T / (R0 + R + 1 / (2 * pi * f * C0))) / (2 * pi * f^2 * C0)
     d(T)/d(R) = 0
     d(T)/d(f) = -1 / f^2
  */
  
  H[0][0] = V_MAX * Q * C * V_MAX * T * exp(-T / (x[0] * C)) / ((Q + Q0) * (Q + Q0) * x[0] * x[0] * C * C); // d(V)/d(R)
  H[0][1] = V_MAX * Q * (-C0) * V_MAX * T * T * exp(-T / (R0 + x[0] + 1 / (2 * pi * x[1] * C0))) / ((Q + Q0) * (Q + Q0) * 2 * pi * x[1] * x[1] * C0); // d(V)/d(f)
  H[1][0] = 0; // d(T)/d(R)
  H[1][1] = -1 / (x[1] * x[1]); // d(T)/d(f)
  
  // Calculate the Kalman gain matrix using matrix operations
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      K[i][j] = P[i][j] * H[j][i] / (H[i][i] * P[i][i] + Rn[i][i]); // K[i][j] = P[i][j]*H[j][i]/(H[i][i]*P[i][i]+Rn[i][i])
    }
  }
  
  // Calculate the innovation vector using vector subtraction
  for (int i = 0; i < 2; i++) {
    y[i] = z[i] - H[i][i] * x[i]; // y[i] = z[i] - H[i][i]*x[i]
  }
  
  // Update the state vector using vector addition and matrix-vector multiplication
  for (int i = 0; i < 2; i++) {
    x[i] = x[i] + K[i][0] * y[0] + K[i][1] * y[1]; // x[i] = x[i] + K[i][j]*y[j]
  }
  
  // Update the state covariance matrix using matrix subtraction and matrix-matrix multiplication
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      P[i][j] = P[i][j] - K[i][0] * H[0][j] - K[i][1] *H[1][j] * P[i][j]; // P[i][j] = P[i][j] - K[i][k]*H[k][j]*P[i][j]
    }
  }
  
  // Assign the updated state vector to the output variables
  R = x[0]; // resistance in ohms
  f = x[1]; // frequency in hertz
}

// Set up the pins and initialize the filter
void setup() {
  pinMode(PIN_OUT, OUTPUT); // set output pin mode
  pinMode(PIN_IN, INPUT); // set input pin mode
  Serial.begin(9600); // start serial communication
  initFilter(); // initialize the filter
}

// Measure the resistance every T_INTERVAL milliseconds using binary search and Kalman filter
void loop() {
  static unsigned long lastTime = 0; // last time of measurement in milliseconds
  static float fMin = 0; // lower bound of frequency search in hertz
  static float fMax = 1000; // upper bound of frequency search in hertz
  
  unsigned long currentTime = millis(); // current time in milliseconds
  
  if (currentTime - lastTime >= T_INTERVAL) { // check if it is time to measure
    
    lastTime = currentTime; // update the last time
    
    // Perform a binary search to find the frequency that maximizes the voltage across the capacitor
    f = (fMin + fMax) / 2; // set the frequency to the midpoint of the search range
    T = 1 / f; // calculate the period from the frequency
    V0 = I_MAX * (R0 + R + 1 / (2 * pi * f * C0)); // calculate the output pin voltage from Ohm's law and impedance formula
    analogWrite(PIN_OUT, V0 * 255 / V_MAX); // set the output pin to the corresponding PWM value
    
    delay(100); // wait for the circuit to stabilize
    
    updateFilter(); // update the filter with the new measurements
    
    Serial.print("R = "); // print the resistance value
    Serial.print(R);
    Serial.println(" ohms");
    
    Serial.print("f = "); // print the frequency value
    Serial.print(f);
    Serial.println(" hertz");
    
    Serial.print("V = "); // print the voltage value
    Serial.print(V);
    Serial.println(" volts");
    
    Serial.print("T = "); // print the period value
    Serial.print(T);
    Serial.println(" seconds");
    
    Serial.println(); // print a blank line
    
    if (V > V_MAX / 2) { // check if the voltage is above half of the maximum voltage
      
      fMin = f; // set the lower bound of the search range to the current frequency
      
    } else { // otherwise
      
      fMax = f; // set the upper bound of the search range to the current frequency
      
    }
    
  }
  
}


// Define constants for circuit components
#define C1 0.5e-6 // Capacitance of first capacitor in farads
#define C2 10e-6 // Capacitance of second capacitor in farads
#define R1 1 // Resistance of first resistor in ohms
#define VCC 5 // Supply voltage in volts
#define MAX_CURRENT 0.02 // Maximum current allowed by Arduino pin in amperes
#define R_PIN VCC / MAX_CURRENT // Internal resistance of Arduino pin in ohms

// Define pins for output and input
#define OUT_PIN 9 // Digital output pin connected to RC circuits
#define IN_PIN A0 // Analog input pin connected to second capacitor

// Define tolerance for binary search algorithm
#define TOL 0.001 // Tolerance value for pulse length in seconds

// Define initial values for Kalman filter
#define F_INIT 1000 // Initial frequency estimate in hertz
#define R2_INIT 100 // Initial resistance estimate in ohms
#define V_INIT 2.5 // Initial voltage estimate in volts
#define P_INIT 1000 // Initial error covariance estimate

// Define process noise covariance matrix for Kalman filter
float Q[2][2] = {{0.01}, {0.01}};

// Define measurement noise covariance matrix for Kalman filter
float R[1][1] = {{0.01}};

// Define state transition matrix for Kalman filter
float A[2][2];

// Define measurement matrix for Kalman filter
float H[1][2];

// Define state estimate vector for Kalman filter
float x[2];

// Define error covariance matrix for Kalman filter
float P[2][2];

// Define measurement vector for Kalman filter
float z[1];

// Define gain matrix for Kalman filter
float K[2][1];

// Define temporary matrices and vectors for Kalman filter calculations
float temp1[2][2];
float temp2[2][2];
float temp3[1][2];
float temp4[1][1];
float temp5[1][1];
float temp6[1][2];
float temp7[2][1];
float temp8[2][1];


// Define constants for PID controller
#define KP 0.1 // Proportional gain
#define KI 0.01 // Integral gain
#define KD 0.001 // Derivative gain
#define DT 0.01 // Time interval in seconds

// Define variables for PID controller
float error; // Error between measured and expected voltages
float error_sum; // Sum of errors over time
float error_diff; // Difference of errors over time
float error_prev; // Previous error value
float output; // Output value for pulse length adjustment

// Define a function to calculate the expected voltage across the first capacitor
float calcV1(float f, float r2) {
  // Use Kirchhoff's law and Ohm's law to derive the equation for voltage across the first capacitor
  // V1 = VCC * (R_PIN + R1) / (R_PIN + R1 + r2 + 1 / (2 * PI * f * C1))
  float v1 = VCC * (R_PIN + R1) / (R_PIN + R1 + r2 + 1 / (2 * PI * f * C1));
  return v1;
}

// Define a function to update the PID controller
void updatePID(float v1) {
  // Calculate the error between measured and expected voltages
  error = v1 - analogRead(IN_PIN) * VCC / 1023;

  // Calculate the sum of errors over time
  error_sum = error_sum + error * DT;

  // Calculate the difference of errors over time
  error_diff = (error - error_prev) / DT;

  // Update the previous error value
  error_prev = error;

  // Calculate the output value for pulse length adjustment using PID formula
  output = KP * error + KI * error_sum + KD * error_diff;
}

// Modify the findR2 function to include the PID controller
float findR2() {
  // Initialize the state estimate vector with initial values
  x[0] = F_INIT; // Frequency estimate
  x[1] = R2_INIT; // Resistance estimate

  // Initialize the error covariance matrix with initial values
  P[0][0] = P_INIT; // Variance of frequency estimate
  P[0][1] = 0; // Covariance of frequency and resistance estimates
  P[1][0] = 0; // Covariance of resistance and frequency estimates
  P[1][1] = P_INIT; // Variance of resistance estimate

  // Initialize the pulse length with half of the period of the initial frequency estimate
  float pulse = 0.5 / x[0];

  // Initialize the variables for binary search algorithm
  float low = 0; // Lower bound of pulse length
  float high = TOL; // Upper bound of pulse length
  float mid; // Middle point of pulse length

  // Initialize the variables for measuring voltage and frequency
  float v_max = 0; // Maximum voltage across the capacitor
  float v_min = VCC; // Minimum voltage across the capacitor
  float v; // Current voltage across the capacitor
  float t_max; // Time when maximum voltage occurs
  float t_min; // Time when minimum voltage occurs
  float t; // Current time

  // Initialize the variables for PID controller
  error = 0; // Error between measured and expected voltages
  error_sum = 0; // Sum of errors over time
  error_diff = 0; // Difference of errors over time
  error_prev = 0; // Previous error value
  output = 0; // Output value for pulse length adjustment

  // Repeat until the pulse length is within the tolerance value
  while (high - low > TOL) {
    // Send a pulse to the output pin with the current pulse length
    digitalWrite(OUT_PIN, HIGH);
    delayMicroseconds(pulse * 1000000);
    digitalWrite(OUT_PIN, LOW);

    // Measure the voltage across the capacitor for one period of the initial frequency estimate
    t_max = micros();
    t_min = micros();
    t = micros();
    while (t - t_max < 1000000 / x[0]) {
      // Read the analog voltage from the input pin and convert it to volts
      v = analogRead(IN_PIN) * VCC / 1023;

      // Update the maximum and minimum voltage and time values if needed
      if (v > v_max) {
        v_max = v;
        t_max = t;
      }
      if (v < v_min) {
        v_min = v;
        t_min = t;
      }

      // Update the current time value
      t = micros();
    }

    // Calculate the frequency of the voltage oscillation using the time difference between maximum and minimum values
    float f = 1000000 / (t_max - t_min);

    // Update the state transition matrix using the current frequency value
    A[0][0] = cos(2 * PI * f * pulse); // Frequency transition coefficient
    A[0][1] = -sin(2 * PI * f * pulse) / (R_PIN + R1 + x[1]); // Resistance transition coefficient
    A[1][0] = -sin(2 * PI * f * pulse) * (R_PIN + R1 + x[1]); // Frequency transition coefficient
    A[1][1] = cos(2 * PI * f * pulse); // Resistance transition coefficient

    // Update the measurement matrix using the current frequency value and circuit parameters
    H[0][0] = -VCC * C2 * sin(2 * PI * f * pulse) / (R_PIN + R1 + x[1]); // Frequency measurement coefficient
    H[0][1] = VCC * C2 * (1 - cos(2 * PI * f * pulse)); // Resistance measurement coefficient

    // Update the measurement vector using the maximum voltage value
    z[0] = v_max;

    // Predict the next state estimate vector using the state transition matrix and the current state estimate vector
    x[0] = A[0][0] * x[0] + A[0][1] * x[1];
    x[1] = A[1][0] * x[0] + A[1][1] * x[1];

    // Predict the next error covariance matrix using the state transition matrix, the current error covariance matrix and the process noise covariance matrix
    temp1[0][0] = A[0][0] * P[0][0] + A[0][1] * P[1][0];
    temp1[0][1] = A[0][0] * P[0][1] + A[0][1] * P[1][1];
    temp1[1][0] = A[1][0] * P[0][0] + A[1][1] * P[1][0];
    temp1[1][1] = A[1][0] * P[0][1] + A[1][1] * P[1][1];
    temp2[0][0] = temp1[0][0] * A[0][0] + temp1[0][1] * A[1][0];
    temp2[0][1] = temp1[0][0] * A[0][1] + temp1[0][1] * A[1][1];
    temp2[1][0] = temp1[1][0] * A[0][0] + temp1[1][1] * A[1][0];
    temp2[1][1] = temp1[1][0] * A[0][1] + temp1[1][1] * A[1][1];
    P[0][0] = temp2[0][0] + Q[0][0];
    P[0][1] = temp2[0][1] + Q[0][1];
    P[1][0] = temp2[1][0] + Q[1][0];
    P[1][1] = temp2[1][1] + Q[1][1];

    // Update the gain matrix using the measurement matrix, the error covariance matrix and the measurement noise covariance matrix
    temp3[0][0] = H[0][0] * P[0][0] + H[0][1] * P[1][0];
    temp3[0][1] = H[0][0] * P[0][1] + H[0][1] * P[1][1];
    temp4[0][0] = temp3[0][0] * H[0][0] + temp3[0][1] * H[0][1];
    temp5[0][0] = temp4[0][0] + R[0][0];
    temp6[0][0] = temp3[0][0] / temp5[0][0];
    temp6[0][1] = temp3[0][1] / temp5[0][0];
    K[0][0] = P[0][0] * temp6[0][0] + P[0][1] * temp6[0][1];
    K[1][0] = P[1][0] * temp6[0][0] + P[1][1] * temp6[0][1];

    // Update the state estimate vector using the gain matrix and the measurement vector
    temp7[0][0] = z[0] - (H[0][0] * x[0] + H[0][1] * x[1]);
    x[0] = x[0] + K[0][0] * temp7[0][0];
    x[1] = x[1] + K[1][0] * temp7[0][0];

    // Update the error covariance matrix using the gain matrix and the measurement matrix
    temp8[0][0] = K[0][0] * H[0][0] + K[1][0] * H[0][1];
    P[0][0] = P[0][0] - temp8[0][0] * P[0][0];
    P[0][1] = P[0][1] - temp8[0][0] * P[0][1];
    P[1][0] = P[1][0] - temp8[0][0] * P[1][0];
    P[1][1] = P[1][1] - temp8[0][0] * P[1][1];

    // Calculate the expected voltage across the first capacitor using the current frequency and resistance values
    float v1 = calcV1(x[0], x[1]);

    // Update the PID controller using the expected voltage value
    updatePID(v1);

    // Adjust the pulse length using the output value from the PID controller
    pulse = pulse + output;

    // Determine the optimal pulse length for each circuit using the binary search algorithm
    if (v_max > V_INIT) { // If the voltage is higher than the initial estimate, increase the pulse length
      low = pulse;
      mid = (low + high) / 2;
      pulse = mid;
    }
    else { // If the voltage is lower than the initial estimate, decrease the pulse length
      high = pulse;
      mid = (low + high) / 2;
      pulse = mid;
    }
  }

  // Return the updated resistance value from the state estimate vector
  return x[1];
}

// Setup function for Arduino program
void setup() {
  // Set the output pin to output mode
  pinMode(OUT_PIN, OUTPUT);

  // Set the input pin to input mode
  pinMode(IN_PIN, INPUT);
}

// Loop function for Arduino program
void loop() {
  // Call the findR2 function every 60 seconds and print the result to serial monitor
  float r2 = findR2();
  Serial.println(r2);
  delay(60000);
}


// Define constants
#define CAPACITANCE 10e-6 // 10 uF
#define MAX_CURRENT 0.02 // 20 mA
#define OUTPUT_PIN 9 // digital output pin connected to RC circuit
#define INPUT_PIN A0 // analog input pin connected to capacitor
#define VOLTAGE 5 // output voltage of Arduino
#define RESOLUTION 1023 // analog resolution of Arduino
#define INTERVAL 60000 // interval to check resistance change in milliseconds

// Define variables
float resistance; // resistance of RC circuit in ohms
float frequency; // frequency of output pulse in hertz
float period; // period of output pulse in microseconds
float pulse; // pulse width of output pulse in microseconds
float voltage; // voltage across capacitor in volts
float angle; // phase angle of RC circuit in radians
int value; // analog value read from input pin
unsigned long timer; // timer to keep track of interval

// Define Kalman filter parameters
float Q_angle = 0.001; // process noise covariance for angle
float Q_bias = 0.003; // process noise covariance for bias
float R_measure = 0.03; // measurement noise covariance
float angle_k = 0; // estimated angle in radians
float bias_k = 0; // estimated bias in radians per second
float rate_k = 0; // unbiased rate in radians per second
float P[2][2] = {{0,0},{0,0}}; // error covariance matrix
float K[2]; // Kalman gain vector
float y; // angle difference in radians
float S; // estimate error

// Define binary search parameters
float low = 1; // lower bound of frequency range in hertz
float high = 10000; // upper bound of frequency range in hertz
float mid; // middle point of frequency range in hertz

void setup() {
  Serial.begin(9600); // initialize serial communication
  
  pinMode(OUTPUT_PIN, OUTPUT); // set output pin as output
  
  timer = millis(); // initialize timer
  
  mid = (low + high) / 2; // initialize mid point
  
  frequency = mid; // initialize frequency
  
  period = 1000000 / frequency; // calculate period
  
  pulse = period / 2; // set pulse width to half of period
  
}

void loop() {
  
  if (millis() - timer >= INTERVAL) { // check if interval has passed
    
    timer = millis(); // reset timer
    
    resistance = findResistance(); // call function to find resistance
    
    Serial.print("Resistance: "); 
    Serial.print(resistance); 
    Serial.println(" ohms"); 
    
    low = 1; // reset lower bound
    
    high = 10000; // reset upper bound
    
    mid = (low + high) / 2; // reset mid point
    
    frequency = mid; // reset frequency
    
    period = 1000000 / frequency; // recalculate period
    
    pulse = period / 2; // reset pulse width
    
    angle_k = 0; // reset estimated angle
    
    bias_k = 0; // reset estimated bias
    
    rate_k = 0; // reset unbiased rate
    
    P[0][0] = 0; P[0][1] = 0; P[1][0] = 0; P[1][1] = 0; // reset error covariance matrix
    
    K[0] = 0; K[1] = 0; // reset Kalman gain vector
    
    y = 0; S = 0;// reset angle difference and estimate error
  }
  
  // Generate output pulse
  digitalWrite(OUTPUT_PIN, HIGH); // set output pin high
  delayMicroseconds(pulse); // wait for pulse width
  digitalWrite(OUTPUT_PIN, LOW); // set output pin low
  delayMicroseconds(period - pulse); // wait for remaining period
  
  // Read input voltage
  value = analogRead(INPUT_PIN); // read analog value from input pin
  voltage = value * VOLTAGE / RESOLUTION; // convert analog value to voltage
  
  // Calculate phase angle
  angle = asin(voltage / VOLTAGE); // calculate phase angle in radians
  
  // Apply Kalman filter
  rate_k = angle - angle_k; // calculate unbiased rate
  angle_k += rate_k; // update estimated angle
  P[0][0] += Q_angle; // update error covariance matrix
  P[0][1] -= Q_bias;
  P[1][0] -= Q_bias;
  P[1][1] += Q_bias;
  
  S = P[0][0] + R_measure; // calculate estimate error
  K[0] = P[0][0] / S; // calculate Kalman gain for angle
  K[1] = P[1][0] / S; // calculate Kalman gain for bias
  
  y = angle - angle_k; // calculate angle difference
  
  angle_k += K[0] * y; // update estimated angle with measurement
  bias_k += K[1] * y; // update estimated bias with measurement
  
  P[0][0] -= K[0] * P[0][0]; // update error covariance matrix
  P[0][1] -= K[0] * P[0][1];
  P[1][0] -= K[1] * P[0][0];
  P[1][1] -= K[1] * P[0][1];
  
  // Perform binary search to find resonant frequency
  if (angle_k > PI / 2) { // if estimated angle is greater than pi/2 radians
    high = frequency; // set upper bound to current frequency
    mid = (low + high) / 2; // calculate new mid point
    frequency = mid; // set frequency to mid point
    period = 1000000 / frequency; // recalculate period
    pulse = period / 2; // reset pulse width to half of period
    
    Serial.print("Frequency: "); 
    Serial.print(frequency); 
    Serial.println(" Hz"); 
    
    Serial.print("Angle: "); 
    Serial.print(angle_k * 180 / PI); 
    Serial.println(" degrees"); 
    
    Serial.println("Searching lower range...");
    
  } else if (angle_k < PI / 2) { // if estimated angle is less than pi/2 radians
    low = frequency; // set lower bound to current frequency
    mid = (low + high) / 2; // calculate new mid point
    frequency = mid; // set frequency to mid point
    period = 1000000 / frequency; // recalculate period
    pulse = period / 2; // reset pulse width to half of period
    
    Serial.print("Frequency: "); 
    Serial.print(frequency); 
    Serial.println(" Hz"); 
    
    Serial.print("Angle: "); 
    Serial.print(angle_k * 180 / PI); 
    Serial.println(" degrees"); 
    
    Serial.println("Searching higher range...");
    
  } else { // if estimated angle is equal to pi/2 radians
    
    Serial.println("Resonant frequency found!");
    
    Serial.print("Frequency: "); 
    Serial.print(frequency); 
    Serial.println(" Hz"); 
    
    Serial.print("Angle: "); 
    Serial.print(angle_k * 180 / PI); 
    Serial.println(" degrees"); 
    
    noLoop(); // stop the loop
    
  }
  
}

// Function to find resistance of RC circuit using Kirchhoff's law

float findResistance() {
  
  float r; // resistance in ohms
  
  r = VOLTAGE / MAX_CURRENT - CAPACITANCE * PI * frequency * VOLTAGE / voltage;
  
  return r;
  
}

//Source: Conversation with Bing, 4/11/2023(1) RC Control and Arduino: a Complete Works - Instructables. https://www.instructables.com/RC-Control-and-Arduino-A-Complete-Works/ Accessed 4/11/2023.
//(2) Arduino RC Circuit: PWM to Analog DC : 10 Steps - Instructables. https://www.instructables.com/Arduino-RC-Circuit-PWM-to-analog-DC/ Accessed 4/11/2023.
//(3) DIY Arduino RC Transmitter - How To Mechatronics. https://howtomechatronics.com/projects/diy-arduino-rc-transmitter/ Accessed 4/11/2023.
//(4) Kalman Filter Library - Arduino Reference. http://reference.arduino.cc/reference/en/libraries/kalman-filter-library/ Accessed 4/11/2023.
//(5) Stabilize Sensor Readings With Kalman Filter - Instructables. https://www.instructables.com/Stabilize-Sensor-Readings-With-Kalman-Filter/ Accessed 4/11/2023.
//(6) rfetick/Kalman: Implement Kalman filter for your Arduino projects - Github. https://github.com/rfetick/Kalman Accessed 4/11/2023.


// Define constants and variables
const int outputPin = 9; // Digital output pin for RC circuit
const int inputPin = A0; // Analog input pin for reading voltage across capacitor
const float capacitance = 10e-6; // Capacitance of capacitor in farads
const float pulseWidth = 0.1; // Pulse width in milliseconds
const float threshold = 0.5; // Threshold voltage for detecting resonance
const int interval = 60000; // Interval for calling the resonance finding function in milliseconds
float resistance; // Resistance of resistor in ohms
float frequency; // Resonance frequency in hertz
unsigned long previousMillis = 0; // Previous time for calling the resonance finding function

// Setup function
void setup() {
  // Set output pin to low
  pinMode(outputPin, OUTPUT);
  digitalWrite(outputPin, LOW);
  // Set input pin to input
  pinMode(inputPin, INPUT);
  // Start serial communication
  Serial.begin(9600);
}

// Loop function
void loop() {
  // Get current time
  unsigned long currentMillis = millis();
  // Check if interval has passed
  if (currentMillis - previousMillis >= interval) {
    // Call the resonance finding function
    findResonance();
    // Update previous time
    previousMillis = currentMillis;
  }
}

// Resonance finding function
void findResonance() {
  // Initialize variables for measuring voltage and time
  float voltage;
  float peakVoltage = 0;
  float peakTime = 0;
  float startTime;
  float endTime;
  
  // Send a pulse to the output pin
  digitalWrite(outputPin, HIGH);
  delayMicroseconds(pulseWidth * 1000);
  digitalWrite(outputPin, LOW);

  // Measure the voltage across the capacitor until it falls below the threshold
  startTime = micros();
  do {
    voltage = analogRead(inputPin) * (5.0 / 1023.0); // Convert analog reading to voltage
    if (voltage > peakVoltage) {
      peakVoltage = voltage; // Update peak voltage
      peakTime = micros(); // Update peak time
    }
    endTime = micros();
  } while (voltage > threshold);

  // Calculate the resistance and frequency from the measurements
  resistance = -1.0 * (peakTime - startTime) / (capacitance * log(peakVoltage / threshold));
  frequency = 1.0 / (2.0 * PI * sqrt(resistance * capacitance));

  // Print the results to the serial monitor
  Serial.print("Resistance: ");
  Serial.print(resistance);
  Serial.println(" ohms");
  
  Serial.print("Frequency: ");
  Serial.print(frequency);
  Serial.println(" hertz");
}

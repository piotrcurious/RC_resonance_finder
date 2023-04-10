
// Define constants and variables
const int outputPin = 9; // Digital output pin connected to RC circuit
const int inputPin = A0; // Analog input pin connected to capacitor
const float capacitance = 10e-6; // Capacitance in farads
float resistance; // Resistance in ohms
unsigned long previousMillis = 0; // Previous time of measurement
const unsigned long interval = 60000; // Interval between measurements in milliseconds

// Setup function
void setup() {
  // Set output pin as output and input pin as input
  pinMode(outputPin, OUTPUT);
  pinMode(inputPin, INPUT);
  
  // Initialize serial communication
  Serial.begin(9600);
}

// Loop function
void loop() {
  // Get the current time
  unsigned long currentMillis = millis();
  
  // Check if it is time to measure the resistance
  if (currentMillis - previousMillis >= interval) {
    // Update the previous time
    previousMillis = currentMillis;
    
    // Call the function to find the resistance
    resistance = findResistance();
    
    // Print the resistance to the serial monitor
    Serial.print("Resistance: ");
    Serial.print(resistance);
    Serial.println(" ohms");
  }
}

// Function to find the resistance of the RC circuit
float findResistance() {
  // Define variables for the pulse length and the voltage reading
  int pulseLength; // Pulse length in microseconds
  int voltage; // Voltage reading from analog input pin
  
  // Define variables for the minimum and maximum pulse lengths and the step size
  const int minPulse = 100; // Minimum pulse length in microseconds
  const int maxPulse = 10000; // Maximum pulse length in microseconds
  const int stepPulse = 100; // Step size for increasing pulse length in microseconds
  
  // Define variables for storing the best pulse length and the best voltage difference
  int bestPulse = minPulse; // Best pulse length in microseconds
  int bestDiff = 0; // Best voltage difference in analog units
  
  // Loop through different pulse lengths from minimum to maximum with step size
  for (pulseLength = minPulse; pulseLength <= maxPulse; pulseLength += stepPulse) {
    // Send a high pulse to the output pin for the given pulse length
    digitalWrite(outputPin, HIGH);
    delayMicroseconds(pulseLength);
    
    // Send a low pulse to the output pin for the same pulse length
    digitalWrite(outputPin, LOW);
    delayMicroseconds(pulseLength);
    
    // Read the voltage from the input pin after the low pulse
    voltage = analogRead(inputPin);
    
    // Calculate the voltage difference from the midpoint of the analog range (512)
    int diff = abs(voltage - 512);
    
    // Check if the voltage difference is greater than the best voltage difference so far
    if (diff > bestDiff) {
      // Update the best voltage difference and the best pulse length
      bestDiff = diff;
      bestPulse = pulseLength;
    }
  }
  
  // Calculate the frequency of the best pulse length in hertz
  float frequency = 1000000.0 / (2 * bestPulse);
  
  // Calculate the impedance of the RC circuit at that frequency using Ohm's law and Kirchhoff's law
  float impedance = (5.0 * capacitance * frequency) / bestDiff;
  
  // Calculate the resistance of the resistor using Pythagoras' theorem
  float resistance = sqrt(impedance * impedance - (1 / (capacitance * capacitance * frequency * frequency)));
  
  // Return the resistance value
  return resistance;
}

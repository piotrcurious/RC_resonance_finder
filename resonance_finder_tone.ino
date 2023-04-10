
// Define the pins
const int outputPin = 9; // Digital output pin for the RC circuit
const int inputPin = A0; // Analog input pin for reading the voltage across the capacitor

// Define the capacitance value in Farads
const float capacitance = 10e-6;

// Define the initial frequency in Hertz
float frequency = 1000;

// Define the frequency step in Hertz
float frequencyStep = 10;

// Define the threshold voltage in Volts
const float thresholdVoltage = 2.5;

// Define the timer interval in milliseconds
const long timerInterval = 60000;

// Define the variables for storing the resistance and resonance frequency
float resistance = 0;
float resonanceFrequency = 0;

// Define the variables for storing the previous and current time
unsigned long previousTime = 0;
unsigned long currentTime = 0;

void setup() {
  // Initialize the serial monitor
  Serial.begin(9600);

  // Set the output pin as output
  pinMode(outputPin, OUTPUT);

  // Set the initial output to low
  digitalWrite(outputPin, LOW);
}

void loop() {
  // Get the current time
  currentTime = millis();

  // Check if the timer interval has elapsed
  if (currentTime - previousTime >= timerInterval) {
    // Reset the previous time
    previousTime = currentTime;

    // Call the function to find the resonance frequency and resistance
    findResonance();
  }
}

// Define the function to find the resonance frequency and resistance
void findResonance() {
  // Initialize a variable to store the maximum voltage
  float maxVoltage = 0;

  // Initialize a variable to store the current voltage
  float currentVoltage = 0;

  // Initialize a variable to store the current frequency
  float currentFrequency = frequency;

  // Loop until the current frequency exceeds the maximum possible frequency
  while (currentFrequency <= F_CPU / (2 * outputPin)) {
    // Set the output pin to generate a square wave with the current frequency
    tone(outputPin, currentFrequency);

    // Wait for one cycle to stabilize
    delayMicroseconds(1000000 / currentFrequency);

    // Read the voltage across the capacitor
    currentVoltage = analogRead(inputPin) * (5.0 / 1023.0);

    // Check if the current voltage is greater than the maximum voltage
    if (currentVoltage > maxVoltage) {
      // Update the maximum voltage
      maxVoltage = currentVoltage;

      // Update the resonance frequency
      resonanceFrequency = currentFrequency;
    }

    // Increase the current frequency by the frequency step
    currentFrequency += frequencyStep;
  }

  // Turn off the output pin
  noTone(outputPin);

  // Calculate the resistance using Ohm's law and the threshold voltage
  resistance = (5.0 - thresholdVoltage) / (thresholdVoltage / (2 * PI * resonanceFrequency * capacitance));

  // Print the results to the serial monitor
  Serial.print("Resonance frequency: ");
  Serial.print(resonanceFrequency);
  Serial.println(" Hz");
  
  Serial.print("Resistance: ");
  Serial.print(resistance);
  Serial.println(" Ohms");
}

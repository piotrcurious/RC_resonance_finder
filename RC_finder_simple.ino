
// Define constants and variables
const int outputPin = 9; // Digital output pin for RC circuit
const int inputPin = A0; // Analog input pin for reading voltage across capacitor
const float capacitance = 10e-6; // Capacitance of the capacitor in farads
const float threshold = 0.5; // Threshold voltage for detecting resonance
float resistance; // Resistance of the resistor in ohms
float frequency; // Frequency of the output signal in hertz
float period; // Period of the output signal in seconds
float dutyCycle; // Duty cycle of the output signal (0 to 1)
float maxVoltage; // Maximum voltage across capacitor
float minVoltage; // Minimum voltage across capacitor
float peakVoltage; // Peak voltage across capacitor at resonance
unsigned long startTime; // Start time of the output signal in milliseconds
unsigned long endTime; // End time of the output signal in milliseconds
unsigned long peakTime; // Time of the peak voltage in milliseconds
bool rising; // Flag for rising edge of the output signal
bool found; // Flag for finding resonance

// Initialize the pins and serial monitor
void setup() {
  pinMode(outputPin, OUTPUT);
  pinMode(inputPin, INPUT);
  Serial.begin(9600);
}

// Main loop
void loop() {
  findResonance(); // Call the function to find resonance
  delay(60000); // Wait for 60 seconds before repeating
}

// Function to find resonance frequency and resistance
void findResonance() {
  frequency = 1.0; // Start with a low frequency
  period = 1000000.0 / frequency; // Calculate the period in microseconds
  dutyCycle = 0.5; // Start with a 50% duty cycle
  maxVoltage = 0.0; // Reset the maximum voltage
  minVoltage = 5.0; // Reset the minimum voltage
  peakVoltage = 0.0; // Reset the peak voltage
  rising = true; // Set the rising flag to true
  found = false; // Set the found flag to false
  
  while (!found) { // Loop until resonance is found or frequency is too high
    
    startTime = micros(); // Record the start time of the output signal
    
    while (micros() - startTime < period) { // Loop for one period
      
      if (micros() - startTime < dutyCycle * period) { // If within the duty cycle
        digitalWrite(outputPin, HIGH); // Set the output pin to high
      }
      else { // If outside the duty cycle
        digitalWrite(outputPin, LOW); // Set the output pin to low
      }
      
      float voltage = analogRead(inputPin) * (5.0 / 1023.0); // Read and convert the voltage
      
      if (voltage > maxVoltage) { // If voltage is higher than maxVoltage
        maxVoltage = voltage; // Update maxVoltage
      }
      
      if (voltage < minVoltage) { // If voltage is lower than minVoltage
        minVoltage = voltage; // Update minVoltage
      }
      
      if (rising && voltage > threshold) { // If rising edge and above threshold
        peakTime = micros(); // Record the peak time
        peakVoltage = voltage; // Record the peak voltage
        rising = false; // Set the rising flag to false
        
        if (abs(peakVoltage - threshold) < 0.01) { // If peak voltage is close to threshold
          found = true; // Set the found flag to true
          break; // Break out of the inner loop
        }
      }
      
      if (!rising && voltage < threshold) { // If falling edge and below threshold
        rising = true; // Set the rising flag to true
      }
    }
    
    endTime = micros(); // Record the end time of the output signal
    
    if (!found) { // If resonance is not found yet
      
      frequency *= 1.1; // Increase the frequency by 10%
      period = 1000000.0 / frequency; // Recalculate the period
      
      if (frequency > 10000.0) { // If frequency is too high
        Serial.println("Resonance not found"); // Print an error message
        break; // Break out of the outer loop
      }
    }
    
    else { // If resonance is found
      
      resistance = (peakTime - startTime) / (capacitance * log(maxVoltage / peakVoltage)); 
      // Calculate the resistance using the formula R = t / (C * ln(Vmax / Vpeak))

      Serial.print("Resonance frequency: "); // Print the resonance frequency
      Serial.print(frequency); // Print the frequency value
      Serial.println(" Hz"); // Print the unit
      Serial.print("Resistance: "); // Print the resistance
      Serial.print(resistance); // Print the resistance value
      Serial.println(" Ohm"); // Print the unit
    }
  }
}

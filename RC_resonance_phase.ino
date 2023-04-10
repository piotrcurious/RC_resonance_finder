
// Define constants and variables
const int outputPin = 9; // Digital output pin for RC circuit
const int inputPin = A0; // Analog input pin for reading voltage across capacitor
const float capacitance = 10e-6; // Capacitance of the capacitor in farads
float resistance; // Resistance of the resistor in ohms
float frequency; // Frequency of the output signal in hertz
float period; // Period of the output signal in seconds
float dutyCycle; // Duty cycle of the output signal (0 to 1)
float phaseShift; // Phase shift between the output and input signals in radians
float minPhaseShift; // Minimum phase shift between the output and input signals in radians
float resonanceFrequency; // Resonance frequency of the RC circuit
unsigned long startTime; // Start time of the output signal in milliseconds
unsigned long endTime; // End time of the output signal in milliseconds
unsigned long zeroTime; // Time of the zero crossing of the input signal in milliseconds
bool rising; // Flag for rising edge of the input signal
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
  minPhaseShift = PI / 2.0; // Set the minimum phase shift to 90 degrees or pi/2 radians
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
      
      if (rising && voltage > 2.5) { // If rising edge and above half of the maximum voltage
        
        zeroTime = micros(); // Record the zero crossing time
        
        phaseShift = (zeroTime - startTime) / (period / (2.0 * PI));
        // Calculate the phase shift between the output and input signals in radians
        
        if (phaseShift < minPhaseShift) { // If phase shift is lower than minPhaseShift
          
          minPhaseShift = phaseShift; // Update minPhaseShift
          resonanceFrequency = frequency; // Update resonanceFrequency
          
          if (abs(phaseShift - PI / 2.0) < 0.01) { // If phase shift is close to 90 degrees or pi/2 radians
            found = true; // Set the found flag to true
            break; // Break out of the inner loop
          }
        }
        
        rising = false; // Set the rising flag to false
      }
      
      if (!rising && voltage < 2.5) { // If falling edge and below half of the maximum voltage
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
      
      resistance = 1.0 / (2.0 * PI * frequency * capacitance); 
      // Calculate the resistance using the formula R = 1 / (2 * pi * f * C)
      Serial.print("Resonance frequency: "); // Print the resonance frequency
      Serial.print(resonanceFrequency); // Print the resonance frequency value
      Serial.println(" Hz"); // Print the unit
      Serial.print("Resistance: "); // Print the resistance
      Serial.print(resistance); // Print the resistance value
      Serial.println(" Ohm"); // Print the unit
    }
  }
}

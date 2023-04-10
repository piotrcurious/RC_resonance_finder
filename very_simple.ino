
// Define constants and variables
const int outputPin = 9; // digital output pin for RC circuit
const int inputPin = A0; // analog input pin for reading voltage across capacitor
const float capacitance = 10e-6; // capacitance of capacitor in farads
const float pi = 3.14159; // pi constant
float resistance; // resistance of resistor in ohms
float frequency; // frequency of RC circuit in hertz
float voltage; // voltage across capacitor in volts
float time; // time in seconds
float period; // period of RC circuit in seconds
unsigned long startTime; // start time of function in milliseconds
unsigned long endTime; // end time of function in milliseconds

// Define a function to find the resonant frequency and resistance of the RC circuit
void findResonance() {
  // Set the output pin to high
  digitalWrite(outputPin, HIGH);
  // Record the start time of the function
  startTime = millis();
  // Wait for the capacitor to charge up to 63.2% of the supply voltage (time constant)
  while (analogRead(inputPin) < 0.632 * 1023) {
    // Do nothing
  }
  // Record the end time of the charging phase
  endTime = millis();
  // Calculate the time elapsed in seconds
  time = (endTime - startTime) / 1000.0;
  // Calculate the resistance using the formula R = t / C
  resistance = time / capacitance;
  // Set the output pin to low
  digitalWrite(outputPin, LOW);
  // Wait for the capacitor to discharge down to 36.8% of the supply voltage (time constant)
  while (analogRead(inputPin) > 0.368 * 1023) {
    // Do nothing
  }
  // Record the end time of the discharging phase
  endTime = millis();
  // Calculate the time elapsed in seconds
  time = (endTime - startTime) / 1000.0;
  // Calculate the period using the formula T = 2 * t
  period = 2 * time;
  // Calculate the frequency using the formula f = 1 / T
  frequency = 1 / period;
}

// Setup function runs once when the Arduino is powered on or reset
void setup() {
  // Set the output pin as an output
  pinMode(outputPin, OUTPUT);
  // Set the input pin as an input
  pinMode(inputPin, INPUT);
}

// Loop function runs repeatedly after the setup function is completed
void loop() {
  // Call the findResonance function every 60 seconds
  findResonance();
  
// Print the results to the serial monitor
Serial.print("Resistance: ");
Serial.print(resistance);
Serial.println(" ohms");
Serial.print("Frequency: ");
Serial.print(frequency);
Serial.println(" hertz");

// Wait for one minute before repeating the loop
delay(60000);
}


// Define constants and variables
#define MAX_CURRENT 0.02 // Maximum current in amps
#define CAPACITANCE 0.00001 // Capacitance in farads
#define OUTPUT_PIN 13 // Digital output pin connected to RC circuit
#define INPUT_PIN A0 // Analog input pin connected to capacitor
#define INTERVAL 60000 // Interval in milliseconds to check resistance

float resistance; // Resistance in ohms
float frequency; // Frequency in hertz
float period; // Period in seconds
float pulse; // Pulse length in seconds
float voltage; // Voltage across capacitor in volts
float max_voltage; // Maximum voltage across capacitor in volts
float min_pulse; // Minimum pulse length in seconds
float max_pulse; // Maximum pulse length in seconds

// Initialize the serial monitor and the pins
void setup() {
  Serial.begin(9600);
  pinMode(OUTPUT_PIN, OUTPUT);
  pinMode(INPUT_PIN, INPUT);
}

// Main loop
void loop() {
  // Call the function to find the resistance and the frequency every interval
  find_resistance_and_frequency();
  Serial.print("Resistance: ");
  Serial.println(resistance);
  Serial.print("Frequency: ");
  Serial.println(frequency);
  delay(INTERVAL);
}

// Function to find the resistance and the frequency using binary search
void find_resistance_and_frequency() {
  // Initialize the pulse range based on the previous frequency or a default value
  if (frequency == 0) {
    frequency = 1000; // Default frequency in hertz
  }
  period = 1 / frequency; // Period in seconds
  pulse = period / 2; // Pulse length in seconds
  min_pulse = pulse / 2; // Minimum pulse length in seconds
  max_pulse = pulse * 2; // Maximum pulse length in seconds

  // Loop until the pulse range is small enough
  while (max_pulse - min_pulse > 0.000001) {
    // Send a pulse to the RC circuit and read the voltage across the capacitor
    digitalWrite(OUTPUT_PIN, HIGH);
    delayMicroseconds(pulse * 1000000);
    digitalWrite(OUTPUT_PIN, LOW);
    delayMicroseconds(pulse * 1000000);
    voltage = analogRead(INPUT_PIN) * (5.0 / 1023.0);

    // Compare the voltage with the maximum voltage and adjust the pulse range accordingly
    if (voltage > max_voltage) {
      max_voltage = voltage;
      frequency = 1 / (pulse * 2); // Frequency in hertz
      min_pulse = pulse;
      pulse = (pulse + max_pulse) / 2;
    } else {
      max_pulse = pulse;
      pulse = (pulse + min_pulse) / 2;
    }
  }

  // Calculate the resistance using Kirchhoff's law and Ohm's law
  // Vout = Vin * (1 - exp(-t / (RC)))
  // Vout / Vin = (1 - exp(-t / (RC)))
  // ln(1 - Vout / Vin) = -t / (RC)
  // R = -t / (C * ln(1 - Vout / Vin))
  
  resistance = -pulse / (CAPACITANCE * log(1 - max_voltage / MAX_CURRENT));
}

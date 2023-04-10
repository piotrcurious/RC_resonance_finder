
// Define constants and variables
const float C = 10e-6; // capacitance in Farads
const int pinOut = 9; // digital output pin
const int pinIn = A0; // analog input pin
float R; // resistance in Ohms
float f; // frequency in Hertz
float Vc; // voltage across capacitor in Volts
float Vmax; // maximum voltage in Volts
float t; // time in seconds
float dt; // time step in seconds
float T; // period in seconds
bool rising; // flag for rising edge of pulse

void setup() {
  // Initialize serial communication and pins
  Serial.begin(9600);
  pinMode(pinOut, OUTPUT);
  pinMode(pinIn, INPUT);
}

void loop() {
  // Call the resonance finding function every 60 seconds
  findResonance();
  delay(60000);
}

void findResonance() {
  // Initialize variables and parameters
  R = 0;
  f = 0;
  Vc = 0;
  Vmax = 5;
  t = 0;
  dt = 0.0001;
  T = 0;
  rising = true;

  // Generate a pulse train with decreasing period until resonance is found
  while (true) {
    // Update the output pin according to the pulse train
    if (t < T / 2) {
      digitalWrite(pinOut, HIGH);
    } else {
      digitalWrite(pinOut, LOW);
    }

    // Read the voltage across the capacitor
    Vc = analogRead(pinIn) * (5.0 / 1023.0);

    // Check if the voltage is at maximum (resonance condition)
    if (Vc == Vmax) {
      // Calculate the resistance and frequency from the period
      R = T / C;
      f = 1 / T;

      // Print the results to the serial monitor
      Serial.print("Resistance: ");
      Serial.print(R);
      Serial.println(" Ohms");
      Serial.print("Frequency: ");
      Serial.print(f);
      Serial.println(" Hz");

      // Break the loop
      break;
    }

    // Check if the voltage is rising and update the flag
    if (Vc > Vmax / 2 && rising) {
      rising = false;
    }

    // Check if the voltage is falling and update the flag and period
    if (Vc < Vmax / 2 && !rising) {
      rising = true;
      T = t * 2;
      t = 0;

      // Decrease the period by a factor of two
      T = T / 2;
    }

    // Update the time by a small step
    t = t + dt;

    // Delay for a short time to avoid overflow
    delayMicroseconds(100);
  }
}

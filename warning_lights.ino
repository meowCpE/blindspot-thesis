/*
    ARDUINO NANO - Optimized Relay Controller
    WIRING:
    - RED light    -> NC (Normally Closed)
    - GREEN light  -> NO (Normally Open)
*/

#define RELAY_IS_ACTIVE_LOW true  // Set to 'true' for Active-LOW, 'false' for Active-HIGH

// Define the logic states
const int RELAY_GREEN = (RELAY_IS_ACTIVE_LOW) ? LOW  : HIGH;
const int RELAY_RED   = (RELAY_IS_ACTIVE_LOW) ? HIGH : LOW;

const int relayPin = 7; 

void setup() {
  Serial.begin(9600); 
  pinMode(relayPin, OUTPUT);

  // CHANGED: Default to GREEN on startup to match Python's initial state
  digitalWrite(relayPin, RELAY_GREEN);

  Serial.print("Ready. Mode: ");
  Serial.println(RELAY_IS_ACTIVE_LOW ? "Active-LOW" : "Active-HIGH");
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();

    if (command == 'R') {
      digitalWrite(relayPin, RELAY_RED);
    } 
    else if (command == 'G') {
      digitalWrite(relayPin, RELAY_GREEN);
    }
    
    // Clear any extra characters (\n or \r) to prevent flickering
    while(Serial.available() > 0) Serial.read(); 
  }
}

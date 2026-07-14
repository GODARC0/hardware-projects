// --- Motor 1 Pins ---
const int stepPin1 = 18; 
const int dirPin1 = 19;  

// --- Motor 2 Pins ---
const int stepPin2 = 22; 
const int dirPin2 = 23;  

// 200 full steps = 1 complete 360-degree rotation
const int stepsPerRevolution = 200; 

void setup() {
  // Initialize Motor 1 pins
  pinMode(stepPin1, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  
  // Initialize Motor 2 pins
  pinMode(stepPin2, OUTPUT);
  pinMode(dirPin2, OUTPUT);
}

void loop() {
  // =======================================================
  // STEP 1 & 2 COMBINED: Rotate BOTH Motors (Clockwise)
  // =======================================================
  digitalWrite(dirPin1, HIGH); // Set Motor 1 CW
  digitalWrite(dirPin2, HIGH); // Set Motor 2 CW
  
  // A single loop pulsing both motors simultaneously
  for (int i = 0; i < stepsPerRevolution; i++) {
    digitalWrite(stepPin1, HIGH);
    digitalWrite(stepPin2, HIGH);
    delayMicroseconds(4000); // Speed control
    
    digitalWrite(stepPin1, LOW);
    digitalWrite(stepPin2, LOW);
    delayMicroseconds(4000);
  }
  
  delay(4000); // Pause 4 seconds while both are done moving CW

  // =======================================================
  // STEP 3: Rotate BOTH Motors Back (Counter-Clockwise)
  // =======================================================
  digitalWrite(dirPin1, LOW); // Set Motor 1 CCW
  digitalWrite(dirPin2, LOW); // Set Motor 2 CCW
  
  for (int i = 0; i < stepsPerRevolution; i++) {
    digitalWrite(stepPin1, HIGH);
    digitalWrite(stepPin2, HIGH);
    delayMicroseconds(4000);
    
    digitalWrite(stepPin1, LOW);
    digitalWrite(stepPin2, LOW);
    delayMicroseconds(4000);
  }
  
  delay(2000); // Wait 2 seconds before repeating the whole sequence
}
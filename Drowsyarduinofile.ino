#include <SoftwareSerial.h>

#define LPWM_PIN 5       // D5 - Motor Driver LPWM
#define L_EN_PIN 6       // D6 - Motor Driver Enable
#define TRIGGER_LIGHT 4  // D4 - Trigger pin for light (Active HIGH when 5V is given) - Has external 10k pull-down
#define TRIGGER_CALL 7   // D7 - Trigger pin for call (Active LOW when connected to GND) - Using internal pull-up

#define SIM800L_RX 2     // D3 - SIM800L RX
#define SIM800L_TX 3     // D2 - SIM800L TX

#define MQ3_SENSOR A0    // MQ-3 sensor connected to A0
#define DRUNKEN_OUTPUT 10 // D10 - Output pin for drunken detection

// Create software serial object for SIM800L
SoftwareSerial sim800l(SIM800L_TX, SIM800L_RX);  // Note: TX connects to RX and RX connects to TX

const char phoneNumber[] = "+919698692750";  // Replace with your number

// Flag variables to track states
bool isLightTriggered = false;
bool isCallTriggered = false;
bool isDrunkDetected = false;
unsigned long drunkStartTime = 0;
unsigned long blinkStartTime = 0;
int blinkCount = 0;
bool blinkState = false;
unsigned long lastBlinkChange = 0;

void setup() {
  // Set pin modes
  pinMode(LPWM_PIN, OUTPUT);
  pinMode(L_EN_PIN, OUTPUT);
  pinMode(TRIGGER_LIGHT, INPUT);  // Using external 10k pull-down resistor
  pinMode(TRIGGER_CALL, INPUT_PULLUP);  // Using internal pull-up resistor
  pinMode(MQ3_SENSOR, INPUT);
  pinMode(DRUNKEN_OUTPUT, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);   // For debug

  // Initialize outputs to LOW
  digitalWrite(LPWM_PIN, LOW);
  digitalWrite(L_EN_PIN, LOW);
  digitalWrite(DRUNKEN_OUTPUT, LOW);
  
  // Start serial communication
  Serial.begin(9600);
  delay(1000);  // Give serial time to initialize
  
  // Initialize SIM800L module
  sim800l.begin(9600);
  delay(3000);  // Give SIM800L time to start up
  
  // Basic AT command test
  sim800l.println("AT");
  delay(1000);
  
  if (sim800l.available()) {
    String response = "";
    while (sim800l.available()) {
      response += (char)sim800l.read();
    }
    Serial.println("SIM800L response: " + response);
  } else {
    Serial.println("No response from SIM800L");
  }
  
  Serial.println("System initialized");
  
  // Visual indicator that setup is complete
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }
}

void loop() {
  // Read sensor values
  int lightTriggerState = digitalRead(TRIGGER_LIGHT);
  int callTriggerState = digitalRead(TRIGGER_CALL);
  int mq3Value = analogRead(MQ3_SENSOR);
  
  // Debug output every second
  static unsigned long lastDebugTime = 0;
  if (millis() - lastDebugTime >= 1000) {
    lastDebugTime = millis();
    Serial.print("Light trigger: ");
    Serial.print(lightTriggerState);
    Serial.print(", Call trigger: ");
    Serial.print(callTriggerState);
    Serial.print(", MQ3 value: ");
    Serial.println(mq3Value);
  }
  
  // LIGHT TRIGGER HANDLING
  if (lightTriggerState == HIGH && !isLightTriggered) {
    Serial.println("Light trigger activated");
    isLightTriggered = true;
    blinkStartTime = millis();
    blinkCount = 0;
    lastBlinkChange = 0;
  }
  
  // Handle light blinking sequence
  if (isLightTriggered) {
    // 15 seconds of blinking (1 second on, 1 second off)
    if (millis() - blinkStartTime <= 15000) {
      // Time to change the blink state?
      if (millis() - lastBlinkChange >= 1000) {
        blinkState = !blinkState;
        lastBlinkChange = millis();
        
        if (blinkState) {
          digitalWrite(LPWM_PIN, HIGH);
          digitalWrite(L_EN_PIN, HIGH);
          Serial.println("Light ON");
        } else {
          digitalWrite(LPWM_PIN, LOW);
          digitalWrite(L_EN_PIN, LOW);
          Serial.println("Light OFF");
          blinkCount++;
        }
      }
    } else {
      // Blinking sequence complete
      isLightTriggered = false;
      digitalWrite(LPWM_PIN, LOW);
      digitalWrite(L_EN_PIN, LOW);
      Serial.println("Light blinking sequence complete");
    }
  }
  
  // CALL TRIGGER HANDLING - Using internal pull-up, so LOW means button pressed
  static unsigned long lastCallTime = 0;
  static bool callInProgress = false;
  
  if (callTriggerState == LOW && !callInProgress && (millis() - lastCallTime > 5000)) {
    Serial.println("Call trigger activated");
    callInProgress = true;
    lastCallTime = millis();
    
    // Make phone call
    Serial.println("Calling " + String(phoneNumber));
    sim800l.println("ATD" + String(phoneNumber) + ";");
    
    // Start call monitoring
    unsigned long callStartTime = millis();
    while (millis() - callStartTime < 30000 && callInProgress) {  // 30 second timeout
      if (sim800l.available()) {
        String response = "";
        while (sim800l.available()) {
          response += (char)sim800l.read();
        }
        Serial.println("SIM800L: " + response);
        
        // Check if call was ended
        if (response.indexOf("NO CARRIER") != -1) {
          callInProgress = false;
          Serial.println("Call ended by receiver");
          break;
        }
      }
      delay(100);  // Small delay while monitoring call
    }
    
    // If timeout occurred, hang up anyway
    if (callInProgress) {
      sim800l.println("ATH");
      Serial.println("Call timeout - hanging up");
      callInProgress = false;
    }
  }
  
  // MQ3 ALCOHOL SENSOR HANDLING
  if (mq3Value > 400 && !isDrunkDetected) {
    Serial.println("Alcohol detected! Level: " + String(mq3Value));
    isDrunkDetected = true;
    drunkStartTime = millis();
    digitalWrite(DRUNKEN_OUTPUT, HIGH);
  }
  
  // Keep output high for 10 seconds
  if (isDrunkDetected && (millis() - drunkStartTime >= 10000)) {
    digitalWrite(DRUNKEN_OUTPUT, LOW);
    isDrunkDetected = false;
    Serial.println("Alcohol detection reset");
  }
  
  // Small delay to prevent loop from running too fast
  delay(50);
}
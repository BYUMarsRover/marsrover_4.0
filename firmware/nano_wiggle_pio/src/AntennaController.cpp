#include <Arduino.h>
#include <Servo.h>

Servo myservo;

// Pin configuration
const int potpin = A2;         // Potentiometer pin
const int switchPin = 4;       // Switch pin
const int ledPin = 13;         // LED pin (Note: not PWM-capable)
const int buttonPin = 2;       // Button Pin
const int servoPin = 9;        // Servo pin
const int switchSamplePin = 11; // Was 6 before, switched on pcb
const int batteryPin = A0;     // Battery sample pin I THINK ----

const float dividerRatio = 2.0;      // FIGURE THIS OUT BASED ON PCB VALUES
const float batteryLowThreshold = 4.4; // WHAT IS LOW BATTER???

int val = 0;          // Current value of Potentiometer
int previousVal = 60; // For smoothing rotation
int angle = 90;       // For serial communication to send angle
char receivedChar;    // For serial communication to receive instruction

// -------------------------
// New LED & Battery Functions
// -------------------------

// Check Battery voltage and returns true if it's low
bool isBatteryLow() {
  // Use a static variable to track the last time we sent data to Serial.
  static unsigned long lastBatterySerialTime = 0;

  // Read the analog value from the battery pin.
  int sensorValue = analogRead(batteryPin);
  
  // Convert the analog reading to voltage.
  // (Assumes a 5V reference; adjust if using a different voltage.)
  float voltage = sensorValue * (5.0 / 1023.0) * dividerRatio;
  
  // Send battery voltage over Serial once every 1000ms.
  unsigned long currentMillis = millis();
  if (currentMillis - lastBatterySerialTime >= 1000) {
    
    Serial.println("Battery Voltage: " + String(voltage));
    //Serial.print("Battery Voltage: ");
    //Serial.println(voltage);
    lastBatterySerialTime = currentMillis;
  }
  
  // Return true if the battery voltage is below the threshold.
  return (voltage < batteryLowThreshold);
}

// Normal battery blink: one blink every second.
// Pattern: LED ON for the first 100ms of every 1000ms cycle, then OFF.
void updateNormalLEDBlink() {
  unsigned long currentMillis = millis();
  //if ((currentMillis % 1000) < 50) {
  //  digitalWrite(ledPin, HIGH);
  //} else {
  //  digitalWrite(ledPin, LOW);
  //}

  
  unsigned long cycleTime = 5000; // Total period for the pattern in ms.
  unsigned long currentTime = millis() % cycleTime;

  if (currentTime < 150) {
    digitalWrite(ledPin, HIGH);
  } else {
    // From 150ms to 5000ms, LED remains off (including the off period after blink 3)
    digitalWrite(ledPin, LOW);
  }
}

// Low battery blink: three quick blinks then a pause.
// Pattern:
//   - Blink 1: ON 0–150ms, OFF 150–300ms  
//   - Blink 2: ON 300–450ms, OFF 450–600ms  
//   - Blink 3: ON 600–750ms, OFF 750–900ms  
//   - Pause: OFF 900–1900ms
//   NOTE: Because some functions are blocking, this timing will not be exact
void updateLowBatteryLEDBlink() {
  unsigned long cycleTime = 1900; // Total period for the pattern in ms.
  unsigned long currentTime = millis() % cycleTime;
  
  if (currentTime < 150) {
    digitalWrite(ledPin, HIGH);
  } else if (currentTime < 300) {
    digitalWrite(ledPin, LOW);
  } else if (currentTime < 450) {
    digitalWrite(ledPin, HIGH);
  } else if (currentTime < 600) {
    digitalWrite(ledPin, LOW);
  } else if (currentTime < 750) {
    digitalWrite(ledPin, HIGH);
  } else {
    // From 750ms to 1900ms, LED remains off (including the off period after blink 3)
    digitalWrite(ledPin, LOW);
  }
}

void setup() {
  Serial.begin(9600);          // Initialize serial communication

  myservo.attach(servoPin);    // Attach servo to pin 9
  myservo.write(60); // 60 makes it go to the midpoint (see the mapping down in moveServoSlowly())
  Serial.println("Attatched servo, waiting...");
  delay(10);
  Serial.println("Doing rest of setup");
  pinMode(switchPin, INPUT);   // Set the switch pin as an input
  pinMode(buttonPin, INPUT_PULLUP); // Set the button pin as input with pull-up resistor
  pinMode(ledPin, OUTPUT);
  angle = 90;

  Serial.println("Setup complete.");
}

int smoothAnalogRead(int pin) {
  int total = 0;
  for (int i = 0; i < 15; i++) {
    total += analogRead(pin);
    delay(15);  // Small delay between readings
  }
  return total / 15;  // Return the averaged value
}

void moveServoSlowly(int targetVal) {
  int mappedVal = map(targetVal, 0, 270, 0, 180); // The servo is a 270 degree servo, so we map the 0-180 values
  mappedVal = constrain(mappedVal, 0, 180);       // to 0-270 and then constrain to make sure it's in range
  //Serial.println(targetVal);
  if (digitalRead(switchSamplePin) == HIGH) {
    // Serial.println("Servo running...");
    // Move the servo in small increments
    int stepDelay = 40;  // Delay between each step in milliseconds
    int stepSize = 1;    // How much to move in each step
  
    if (mappedVal > previousVal) {
      if (mappedVal < 5) {
        previousVal = 270; // Assumes something went wrong, zeroes all the way (ie gust of wind messed up antenna)
      }
      for (int pos = previousVal; pos <= mappedVal; pos += stepSize) {
        if (digitalRead(switchSamplePin) == LOW) {
          previousVal = pos;
          return;
        }
        myservo.write(pos);
        delay(stepDelay);
      }
    } else {
      for (int pos = previousVal; pos >= mappedVal; pos -= stepSize) {
        if (digitalRead(switchSamplePin) == LOW) {
          previousVal = pos;
          return;
        }
        myservo.write(pos);
        delay(stepDelay);
      }
    }
  
    previousVal = mappedVal;  // Update previous value after movement is complete
    angle = targetVal; // This is redundant and a bit messy. Clean up later
  }
}

void serialControl() {
  const int stepSize = 5;

  if (Serial.available() > 0) {
    char received = Serial.peek(); // Read the incoming byte



    if (isDigit(received)) {
      int newAngle = Serial.parseInt(); // Will parse the rest of the digits
      Serial.print("Received integer: ");
      Serial.println(angle);
      if (newAngle >= 0 && newAngle <= 180) {
        Serial.println("Moving to: " + String(newAngle) + " degrees");
        angle = newAngle;
        moveServoSlowly(angle);
      } else {
        Serial.println("Invalid angle. Please send a value between 0 and 180.");
      }
    } else {
      switch (toupper(received)) {
        case 'H':
          Serial.println("Hello from Arduino");
          break;
        case 'R':
          angle = max(0, angle - stepSize);
          Serial.println("Moving Right to: " + String(angle) + " degrees");
          moveServoSlowly(angle);
          break;
        case 'L':
          angle = min(180, angle + stepSize);
          Serial.println("Moving Left to: " + String(angle) + " degrees");
          moveServoSlowly(angle);
          break;
        default:
          // Serial.println("Unknown command"); // This reads \n and stuff which screws it up
          break;
      }
    }
    // Flush the buffer to remove unwanted characters
    while (Serial.available() > 0) {
      Serial.read();  // Discard remaining data (like \n or \r)
    }
  }
}



void potentiometerControl() {
  val = smoothAnalogRead(potpin);   // Read the potentiometer

  val = map(val, 0, 1023, 180, 0);
  // Map potentiometer value to servo range

  if (abs(val - previousVal) > 2) {
    moveServoSlowly(val);  // Call function to move the servo slowly
  }
}

void loop() {
  // Read the state of the control switch
  int switchState = digitalRead(switchPin);
  
  if (switchState == HIGH) {
    // Manual (potentiometer) control mode:
    potentiometerControl();
    // LED: Solid ON to indicate manual mode.
    digitalWrite(ledPin, HIGH);
  } else {
    // Serial control mode:
    serialControl();
    // In Serial mode, the LED will blink:
    // - Normal battery: one blink per second.
    // - Low battery: 3 quick blinks then a pause.
    if (isBatteryLow()) {
      updateLowBatteryLEDBlink();
    } else {
      updateNormalLEDBlink();
    }
  }
  
  delay(10);  // Short delay for stability.
}

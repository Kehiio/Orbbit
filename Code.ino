#include <ESP32Servo.h>       // 9g Servo library
#include <Wire.h>             // Display libraries
#include <Adafruit_GFX.h>     // *
#include <Adafruit_SSD1306.h> // *

// --- Servo Pins, interchangeable
#define SERVO_PIN_BLUE 40
#define SERVO_PIN_WHITE 41

// --- Button Pin
#define BUTTON_PIN 34

// ---- I2C and Display setup
#define SDA_PIN 5 // Yellow wire
#define SCL_PIN 4 // Blue wire

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

#define OLED1_ADDR 0x3C
#define OLED2_ADDR 0x3D

// Motor and Display object declarations
Servo servoBlue;
Servo servoWhite;

Adafruit_SSD1306 display1(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
Adafruit_SSD1306 display2(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Robot finite states 
enum MachineState {STATE_IDLE, STATE_BREATHING};

enum BreathState { BREATHE_IN, BREATHE_PAUSE_IN, BREATHE_OUT, BREATHE_PAUSE_OUT };
BreathState breathState = BREATHE_IN;

MachineState currentPhase = STATE_IDLE; // robot starts on idle

// Breathing motion settings 
const int angleMin = 0;                    // Starting angle
const int angleMax = 90;                   // Maximum angle
const unsigned long moveDuration = 4000;   // 4 seconds for expantion
const unsigned long pauseDuration = 4000;  // 4 seconds paused
const unsigned long stepInterval = moveDuration / (angleMax - angleMin); // ms per degree calculation


// ---- Servo pulse range (typical for 9g servos, tweak if yours differs) ----
const int PULSE_MIN = 500;   // microseconds, corresponds to 0 degrees
const int PULSE_MAX = 2500;  // microseconds, corresponds to 180 degrees

// variables used in servo timing
unsigned long phaseStartTime = 0; // when the current BREATH_UP/DOWN motion started
unsigned long pauseStartTime = 0;

// ---- Button state ----
bool lastButtonReading = HIGH;   // raw last reading (pulled up, so HIGH = not pressed)
bool debouncedButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50; // ms

// ---- Face / display geometry ----
const int eyeCenterX = SCREEN_WIDTH / 2;
const int eyeCenterY = SCREEN_HEIGHT / 2;
const int eyeHalfWidth = 45;      // rectangle half-width, constant
const int eyeMaxHalfHeight = 28;  // full-size rectangle (idle / breath pause-out)
const int eyeMinHalfHeight = 6;   // shrunken rectangle (breath pause-in)

MachineState lastDrawnPhase = STATE_BREATHING; // force mismatch so idle draws once at boot
unsigned long lastDisplayUpdate = 0;
const unsigned long displayUpdateInterval = 40; // ms between redraws during breathing animation

void setup() {
  Serial.begin(19200);
  delay(1000);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // ---- Display init/test ----
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  bool ok1 = display1.begin(SSD1306_SWITCHCAPVCC, OLED1_ADDR);
  Serial.printf("Display 1 (0x%02X) init: %s\n", OLED1_ADDR, ok1 ? "OK" : "FAILED");

  bool ok2 = display2.begin(SSD1306_SWITCHCAPVCC, OLED2_ADDR);
  Serial.printf("Display 2 (0x%02X) init: %s\n", OLED2_ADDR, ok2 ? "OK" : "FAILED");

  // Servo setup
  servoBlue.write(0);
  servoWhite.write(0);

  servoBlue.attach(SERVO_PIN_BLUE);
  servoWhite.attach(SERVO_PIN_WHITE);
  
  phaseStartTime = millis();
  Serial.println("Setup complete");
}

void loop() {
  unsigned long now = millis();

  updateButton(now);

  switch (currentPhase) {
    case STATE_IDLE:
      updateIdle(now);
      break;

    case STATE_BREATHING:
      updateBreathing(now);
      break;
  }

  updateFace(now); // runs every loop regardless of phase
}

// ---------------- Button handling ----------------

void updateButton(unsigned long now) {
  bool reading = digitalRead(BUTTON_PIN);

  if (reading != lastButtonReading) {
    lastDebounceTime = now; // reading changed, reset debounce timer
  }

  if ((now - lastDebounceTime) > debounceDelay) {
    // reading has been stable longer than debounceDelay
    if (reading != debouncedButtonState) {
      debouncedButtonState = reading;

      // button is active LOW (INPUT_PULLUP), so a press is HIGH -> LOW
      if (debouncedButtonState == LOW) {
        onButtonPress(now);
      }
    }
  }

  lastButtonReading = reading;
}

void onButtonPress(unsigned long now) {
  if (currentPhase == STATE_IDLE) {
    Serial.println("Button pressed: switching to BREATHING");
    currentPhase = STATE_BREATHING;
    breathState = BREATHE_IN;
    phaseStartTime = now;
  } else {
    Serial.println("Button pressed: switching to IDLE");
    currentPhase = STATE_IDLE;
  }
}

// ---------------- Idle phase ----------------

void updateIdle(unsigned long now) {
  // no timed logic needed right now; face is static in idle
  // placeholder for any future idle behavior (e.g. random idle blinks)
}

// ---------------- Breathing phase (servo, unchanged) ----------------

// helper: convert an angle (0-180) to a microsecond pulse
int angleToPulse(float angle) {
  return PULSE_MIN + (angle / 180.0) * (PULSE_MAX - PULSE_MIN);
}

void updateBreathing(unsigned long now) {
  switch (breathState) {
    case BREATHE_IN: {
      unsigned long elapsed = now - phaseStartTime;
      if (elapsed >= moveDuration) {
        servoBlue.writeMicroseconds(angleToPulse(angleMax));
        servoWhite.writeMicroseconds(angleToPulse(angleMax));
        Serial.println("Reached 90");
        breathState = BREATHE_PAUSE_IN;
        pauseStartTime = now;
      } else {
        float progress = (float)elapsed / moveDuration; // 0.0 to 1.0
        float angle = angleMin + progress * (angleMax - angleMin);
        servoBlue.writeMicroseconds(angleToPulse(angle));
        servoWhite.writeMicroseconds(angleToPulse(angle));
      }
      break;
    }

    case BREATHE_PAUSE_IN: {
      if (now - pauseStartTime >= pauseDuration) {
        breathState = BREATHE_OUT;
        phaseStartTime = now;
      }
      break;
    }

    case BREATHE_OUT: {
      unsigned long elapsed = now - phaseStartTime;
      if (elapsed >= moveDuration) {
        servoBlue.writeMicroseconds(angleToPulse(angleMin));
        servoWhite.writeMicroseconds(angleToPulse(angleMin));
        Serial.println("Reached 0");
        breathState = BREATHE_PAUSE_OUT;
        pauseStartTime = now;
      } else {
        float progress = (float)elapsed / moveDuration;
        float angle = angleMax - progress * (angleMax - angleMin);
        servoBlue.writeMicroseconds(angleToPulse(angle));
        servoWhite.writeMicroseconds(angleToPulse(angle));
        
      }
      break;
    }

    case BREATHE_PAUSE_OUT: {
      if (now - pauseStartTime >= pauseDuration) {
        breathState = BREATHE_IN;
        phaseStartTime = now;
      }
      break;
    }
  }
}

// ---------------- Face / display animation ----------------

// Computes the current eye half-height based on phase/breathState.
int getEyeHalfHeight(unsigned long now) {
  if (currentPhase == STATE_IDLE) {
    return eyeMaxHalfHeight; // full-size rectangle while idle
  }

  // STATE_BREATHING
  switch (breathState) {
    case BREATHE_IN: {
      unsigned long elapsed = now - phaseStartTime;
      float progress = (elapsed >= moveDuration) ? 1.0 : (float)elapsed / moveDuration;
      return eyeMaxHalfHeight - progress * (eyeMaxHalfHeight - eyeMinHalfHeight); // shrinking
    }
    case BREATHE_PAUSE_IN:
      return eyeMinHalfHeight; // held small

    case BREATHE_OUT: {
      unsigned long elapsed = now - phaseStartTime;
      float progress = (elapsed >= moveDuration) ? 1.0 : (float)elapsed / moveDuration;
      return eyeMinHalfHeight + progress * (eyeMaxHalfHeight - eyeMinHalfHeight); // growing back
    }
    case BREATHE_PAUSE_OUT:
    default:
      return eyeMaxHalfHeight; // held full-size
  }
}

void drawEye(Adafruit_SSD1306 &d, int halfHeight) {
  d.clearDisplay();

  int x = eyeCenterX - eyeHalfWidth;
  int y = eyeCenterY - halfHeight;
  int w = eyeHalfWidth * 2;
  int h = halfHeight * 2;

  d.fillRect(x, y, w, h, SSD1306_WHITE);
  d.display();
}

void updateFace(unsigned long now) {
  bool phaseChanged = (currentPhase != lastDrawnPhase);

  if (currentPhase == STATE_IDLE) {
    // static face, only redraw when we just entered idle
    if (phaseChanged) {
      drawEye(display1, eyeMaxHalfHeight);
      drawEye(display2, eyeMaxHalfHeight);
      lastDrawnPhase = currentPhase;
    }
    return;
  }

  // STATE_BREATHING: redraw on a throttled interval so the animation is smooth
  // without hammering the I2C bus every single loop pass
  if (phaseChanged || (now - lastDisplayUpdate >= displayUpdateInterval)) {
    lastDisplayUpdate = now;
    lastDrawnPhase = currentPhase;

    int halfHeight = getEyeHalfHeight(now);
    drawEye(display1, halfHeight);
    drawEye(display2, halfHeight);
  }
}

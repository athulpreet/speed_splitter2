#include <math.h>

// Pin Definitions
#define SWITCH_PIN      2   // PD2 (Digital Pin 2) for button
#define PWM_INPUT_PIN   8   // PB0 (Digital Pin 8) for speed sensor pulses
#define PWM_OUTPUT_PIN1 9   // PB1 (Digital Pin 9 - OC1A)
#define PWM_OUTPUT_PIN2 10  // PB2 (Digital Pin 10 - OC1B)
#define LED_PIN         LED_BUILTIN

// Configuration
#define MAX_SPEED         200.0
#define MAX_FREQUENCY     200.0
#define CALIBRATION_STEPS 3
#define DEBOUNCE_DELAY    100

// Global Variables
volatile float measuredFrequency = 0.0;
volatile bool  frequencyUpdated  = false;

// For pin-change interrupt frequency measurement
volatile unsigned long lastPulseTime = 0;  
volatile bool lastState = LOW;             

int   calibrationStep = 0;
float calibrationFrequencies[CALIBRATION_STEPS] = {0};
const float knownSpeeds[CALIBRATION_STEPS]      = {10.0, 100.0, 200.0};

float coeffA = 0, coeffB = 0, coeffC = 0;

// ----------------------------------------------------------------------------
// Forward Declarations
// ----------------------------------------------------------------------------
void handleCalibration();
void updateCalibrationLED(int step);
void calculateCoefficients();
float calculateCurrentSpeed(float frequency);
void updatePWM(float speed);

// ----------------------------------------------------------------------------
void setup() {

  Serial.begin(9600);
  // Pin Modes
  pinMode(SWITCH_PIN, INPUT);       // Button
  pinMode(PWM_INPUT_PIN, INPUT);    // Speed sensor input on pin 8
  pinMode(PWM_OUTPUT_PIN1, OUTPUT);
  pinMode(PWM_OUTPUT_PIN2, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  // -------------------------------------------------------------------------
  // 1) Set up Pin Change Interrupt on pin 8 (PB0 = PCINT0)
  // -------------------------------------------------------------------------
  PCICR  |= (1 << PCIE0);    // Enable Pin Change Interrupts for PCINT[7:0]
  PCMSK0 |= (1 << PCINT0);   // Enable interrupt for PCINT0 (PB0)
  // This will trigger ISR(PCINT0_vect) on ANY change of pin 8.

  // -------------------------------------------------------------------------
  // 2) Configure Timer1 for Dual PWM (Mode 14: Fast PWM w/ ICR1 as TOP)
  // -------------------------------------------------------------------------
  TCCR1A = 0;
  TCCR1B = 0;

  // Fast PWM, mode 14: WGM13=1, WGM12=1, WGM11=1, WGM10=0
  TCCR1A |= (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11); // Non-inverting
  TCCR1B |= (1 << WGM13) | (1 << WGM12);

  // Prescaler = 8
  TCCR1B |= (1 << CS11);

  // Initial TOP (~100 Hz by default)
  ICR1  = 19999;
  OCR1A = ICR1 / 2;  // 50% duty
  OCR1B = ICR1 / 2;  // 50% duty

  // -------------------------------------------------------------------------
  // 3) Perform calibration steps (3 presses on the button).
  // -------------------------------------------------------------------------
  while (calibrationStep < CALIBRATION_STEPS) {
    handleCalibration();
    updateCalibrationLED(calibrationStep);
  }

  // Once we've gotten all 3 presses => calibrationStep = 3
  // LED should turn OFF.
  updateCalibrationLED(calibrationStep);

  // Calculate polynomial coefficients from the calibration data
  calculateCoefficients();
}

void loop() {

  // Normal operation after calibration
  if (frequencyUpdated) {
    noInterrupts();
    float currentFreq = measuredFrequency;
    frequencyUpdated  = false;
    interrupts();

    float speed = calculateCurrentSpeed(currentFreq);
    Serial.print("speed : ");
      Serial.println(speed);
    updatePWM(speed);
  }
}

// =============================================================================
// Pin Change Interrupt ISR for PB0 (Digital Pin 8)
// =============================================================================
ISR(PCINT0_vect) {
  bool currentState = (bool)(PINB & (1 << PB0));  // Read PB0 directly
  unsigned long now = micros();

  // Rising edge detect
  if (currentState && !lastState) {
    unsigned long elapsed = now - lastPulseTime;
    lastPulseTime = now;

    if (elapsed > 0) {
      measuredFrequency = 1e6 / (float)elapsed;  // freq in Hz
      frequencyUpdated  = true;
    }
  }
  lastState = currentState;
}

// ----------------------------------------------------------------------------
// (A) Calibration Function (Button Press)
// ----------------------------------------------------------------------------
void handleCalibration() {
  static uint32_t lastDebounce    = 0;
  static bool     buttonState     = LOW;
  static bool     lastButtonState = LOW;

  bool reading = digitalRead(SWITCH_PIN);

  // Simple debounce check
  if (reading != lastButtonState) {
    lastDebounce = millis();
  }

  if ((millis() - lastDebounce) > DEBOUNCE_DELAY && (reading != buttonState)) {
    buttonState = reading;
    // We consider HIGH = pressed
    if (buttonState == HIGH) {
      // Store the current measured frequency in the calibration array
      calibrationFrequencies[calibrationStep] = measuredFrequency;

      // A quick LED flash to acknowledge the button press
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);

      // Move to next calibration step
      calibrationStep++;
    }
  }
  lastButtonState = reading;
}

// ----------------------------------------------------------------------------
// (B) LED Behavior Function
// ----------------------------------------------------------------------------
void updateCalibrationLED(int step) {
  /*
   * According to your request:
   *   step == 0 => fast blink (waiting for first press)
   *   step == 1 => medium blink (waiting for second press)
   *   step == 2 => always ON (waiting for third press)
   *   step >= 3 => LED OFF
   */

  // If we're already at or past the third press => LED OFF
  if (step >= 3) {
    digitalWrite(LED_PIN, LOW);
    return;
  }

  // If step=2 => always ON
  if (step == 2) {
    digitalWrite(LED_PIN, HIGH);
    return;
  }

  // Otherwise, we blink for step=0 or step=1
  static unsigned long lastBlink   = 0;
  static bool          ledState    = false;
  unsigned long        currentTime = millis();

  // Decide blink period
  //   step=0 => fast blink => 200ms
  //   step=1 => medium blink => 500ms
  unsigned long blinkPeriod = (step == 0) ? 200 : 500;

  // Blink logic
  if (currentTime - lastBlink >= blinkPeriod) {
    lastBlink = currentTime;
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState);
  }
}

// ----------------------------------------------------------------------------
// (C) Quadratic Fit Coefficients for Speed vs. Frequency
// ----------------------------------------------------------------------------
void calculateCoefficients() {
  float x1 = knownSpeeds[0], y1 = calibrationFrequencies[0];
  float x2 = knownSpeeds[1], y2 = calibrationFrequencies[1];
  float x3 = knownSpeeds[2], y3 = calibrationFrequencies[2];

  float denom = (x1 - x2) * (x1 - x3) * (x2 - x3);

  coeffA = ( x3 * (y2 - y1) 
           + x2 * (y1 - y3) 
           + x1 * (y3 - y2) ) / denom;

  coeffB = ( x3*x3 * (y1 - y2) 
           + x1*x1 * (y2 - y3) 
           + x2*x2 * (y3 - y1) ) / denom;

  coeffC = ( x2*x3 * (x2 - x3) * y1
           + x3*x1 * (x3 - x1) * y2
           + x1*x2 * (x1 - x2) * y3 ) / denom;
}

// ----------------------------------------------------------------------------
// (D) Solve the quadratic for speed given a measured frequency
// ----------------------------------------------------------------------------
float calculateCurrentSpeed(float frequency) {
  float a = coeffA;
  float b = coeffB;
  float c = coeffC - frequency;

  float discriminant = b*b - 4*a*c;
  if (discriminant < 0) return 0;

  float sqrt_d = sqrt(discriminant);
  float speed1 = (-b + sqrt_d) / (2*a);
  float speed2 = (-b - sqrt_d) / (2*a);

  // Return positive solution, constrained to MAX_SPEED
  return constrain((speed1 > 0) ? speed1 : speed2, 0.0, MAX_SPEED);
}

// ----------------------------------------------------------------------------
// (E) Update PWM for Dual Outputs
// ----------------------------------------------------------------------------
void updatePWM(float speed) {
  float frequencyHz = constrain(speed, 0, MAX_FREQUENCY);

  if (frequencyHz <= 0) {
    OCR1A = 0;
    OCR1B = 0;
    return;
  }

  // frequency = F_CPU / (prescaler * (1 + ICR1))
  // => ICR1 = (F_CPU / (prescaler*frequency)) - 1
  // with prescaler=8 => Ftimer=2MHz
  uint32_t topValue = (16000000UL / (8UL * (uint32_t)frequencyHz)) - 1;
  topValue = constrain(topValue, 1, 65535);

  uint16_t dutyValue = topValue / 2;  // 50% duty

  noInterrupts();
  ICR1  = (uint16_t)topValue;
  OCR1A = dutyValue;
  OCR1B = dutyValue;
  interrupts();
}

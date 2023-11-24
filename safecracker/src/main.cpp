#include <Arduino.h>
#include <cmath>
#include <algorithm>
#include <NewEncoder.h> // https://github.com/gfvalvo/NewEncoder
#include <LiquidCrystal.h> // https://github.com/arduino-libraries/LiquidCrystal
#include <AccelStepper.h> // https://github.com/waspinator/AccelStepper

// TODO Be wary of any move() before STEPS_PER_NOTCH is set in init.

// Vocab
//// Notch is a position on a combination lock that may be part of a combination. Most locks have 3 cams that have 100 notches.
//// Cam is a disc in a combination lock. There are typically 3 cams in a combination lock.
//// Tick is an encoder position.
//// Step is a stepper motor step.

// Hardware
LiquidCrystal lcd(13, 12, 25, 26, 27, 14);

int dial_encoder_pin_zero = 18;
int dial_encoder_pin_one = 19;

void ESP_ISR dial_encoder_callback(NewEncoder *encPtr, const volatile NewEncoder::EncoderState *state, void *uPtr);
NewEncoder dial_encoder(dial_encoder_pin_zero, dial_encoder_pin_one, -15000, 15000, 0, FULL_PULSE);

int dial_stepper_pin_enable = 21; // Engage motor
int dial_stepper_pin_step = 22; // Step
int dial_stepper_pin_direction = 23; // Direction

AccelStepper dial_stepper(AccelStepper::DRIVER, dial_stepper_pin_step, dial_stepper_pin_direction);

// CONSTANTS - Hard coded
double MAX_SPEED = 2000.0000; // steps/second
double ACCELERATION = 500.0; // steps/second^2

// CONSTANTS - To be tuned
double STEPS_PER_NOTCH = 20.0;
double ENCODER_TICKS_PER_NOTCH = 6.0;
long NOTCHES_PER_CAM = 67;

// RUNNING VALUES
long position = 0; // in encoder ticks. position in notches = position * ENCODER_TICKS_PER_NOTCH
long target = 0; // in encoder ticks. target in notches = target * ENCODER_TICKS_PER_NOTCH

unsigned long get_time_in_microseconds() {
  return micros();
}

void sleep_microseconds(unsigned long microseconds) {
  unsigned long until = get_time_in_microseconds() + microseconds;
  while (get_time_in_microseconds() < until) {
     dial_stepper.run();
     delayMicroseconds(10);
  }
}

void sleep_seconds(double seconds) {
  sleep_microseconds(std::round(seconds * 1000000));
}

void move(long notches) {
  target = position + notches * ENCODER_TICKS_PER_NOTCH;

  while (target != std::round(position / ENCODER_TICKS_PER_NOTCH)) {
    dial_stepper.move(std::round((target - position) * STEPS_PER_NOTCH / ENCODER_TICKS_PER_NOTCH));
    dial_stepper.run();
  }
  dial_stepper.stop();
  dial_stepper.runToPosition();
}

void ESP_ISR dial_encoder_callback(NewEncoder *encPtr, const volatile NewEncoder::EncoderState *state, void *uPtr) {
  (void) encPtr;
  (void) uPtr;
  position -= dial_encoder.getAndSet(0); // Negative to fix hardware direction
}

void hardware_test() {
  Serial.println("Starting hardware test");

  while (true) {
    Serial.println("Top of hardware test loop");
    lcd.clear();
    lcd.print("Current Position");
    lcd.setCursor(0, 1);
    lcd.print(position);

    dial_stepper.enableOutputs();
    sleep_microseconds(300);

    dial_stepper.runToNewPosition(1);
    sleep_microseconds(1000 * 1000);

    dial_stepper.runToNewPosition(0);
    sleep_microseconds(1000 * 1000);

    lcd.setCursor(0, 1);
    lcd.print(position);

    dial_stepper.disableOutputs();
    sleep_microseconds(300);

    dial_stepper.runToNewPosition(1);
    sleep_microseconds(1000 * 1000);

    dial_stepper.runToNewPosition(0);
    sleep_microseconds(1000 * 1000);
    
    lcd.setCursor(0, 1);
    lcd.print(position);

    Serial.println("Disengaging stepper dial");
    dial_stepper.disableOutputs();
    sleep_microseconds(1000 * 1000);
    digitalWrite(dial_stepper_pin_step, LOW);
    digitalWrite(dial_stepper_pin_direction, LOW);
    sleep_microseconds(1000 * 1000);
    lcd.setCursor(0, 1);
    lcd.print(position);
    digitalWrite(dial_stepper_pin_step, HIGH);
    digitalWrite(dial_stepper_pin_direction, HIGH);
    sleep_microseconds(1000 * 1000);
  }
}

/*
On boot initialization process:
  Wait until you see the dial move forward and back.
  You have 5 seconds to set the dial to 0.
  Dial will engage and barely move forward and back.
  You have 5 seconds to turn the dial one full turn FORWARD(where the numbers count up from 0) to the next 0.
  The dial will turn 9 more revolutions.
  You have 5 seconds to align the dial with the closest 0.
  The dial will turn 10 more revolutions.
  You have 5 seconds to set the dial to notch "20".
    If you want to assume the notches have lenience, like they may be 1.5 numbers wide, use a higher position like 30.
  The dial will then tune speed and acceleration. This may take a couple minutes.
  The safecracker will now start cracking.
*/
void setup() {
  // Setup PC com
  Serial.begin(115200);
  
  // Setup dial_encoder
  pinMode(dial_encoder_pin_zero, INPUT_PULLUP);
  pinMode(dial_encoder_pin_one, INPUT_PULLUP);
  dial_encoder.begin();
  dial_encoder.attachCallback(dial_encoder_callback);

  // Setup LCD
  lcd.begin(16, 2);
  lcd.clear();
  lcd.print("Wait...");

  // Setup dial_stepper
  pinMode(dial_stepper_pin_enable, OUTPUT);
  digitalWrite(dial_stepper_pin_enable, LOW);
  pinMode(dial_stepper_pin_direction, OUTPUT);
  digitalWrite(dial_stepper_pin_direction, LOW);
  pinMode(dial_stepper_pin_step, OUTPUT);
  digitalWrite(dial_stepper_pin_step, HIGH);
  dial_stepper.setMaxSpeed(MAX_SPEED);
  dial_stepper.setAcceleration(ACCELERATION);
  dial_stepper.setEnablePin(dial_stepper_pin_enable);
  // dial_stepper.setPinsInverted(true, false, false); // Invert direction if necessary

  // Hardware Test. Comment out for prod
  // hardware_test();

  // reset stepper motor and let the user know we've booted
  dial_stepper.runToNewPosition(100);
  dial_stepper.runToNewPosition(0);

  // Find ticks per cam, to set ticks per notch, even though we don't know how many notches yet.
  lcd.clear();
  lcd.print("Dial to zero");
  dial_stepper.disableOutputs();
  sleep_seconds(5);
  dial_stepper.enableOutputs();
  sleep_microseconds(50 * 1000); // engage motor
  position = 0;
  dial_stepper.setCurrentPosition(0);
  lcd.clear();
  lcd.print("Wait...");
  dial_stepper.runToNewPosition(STEPS_PER_NOTCH * NOTCHES_PER_CAM);
  long _tmp_position = position;
  dial_stepper.runToNewPosition(0);
  lcd.clear();
  lcd.print("Dial FORWARD to");
  lcd.setCursor(0, 1);
  lcd.print("next zero");
  dial_stepper.disableOutputs();
  sleep_seconds(5);
  dial_stepper.enableOutputs();
  lcd.clear();
  lcd.print("Wait 9 turns...");

  // Set rough estimates
  ENCODER_TICKS_PER_NOTCH = position / NOTCHES_PER_CAM;
  STEPS_PER_NOTCH = std::round((STEPS_PER_NOTCH * NOTCHES_PER_CAM) / (_tmp_position / ENCODER_TICKS_PER_NOTCH));

  // Set more precise encoder ticks per notch
  move(9.0 * NOTCHES_PER_CAM);
  lcd.clear();
  lcd.print("Dial to");
  lcd.setCursor(0, 1);
  lcd.print("nearest zero");
  dial_stepper.disableOutputs();
  sleep_seconds(5);
  dial_stepper.enableOutputs();
  ENCODER_TICKS_PER_NOTCH = std::round((position / NOTCHES_PER_CAM) / 10.0);

  // Find steps per notch
  lcd.clear();
  lcd.print(ENCODER_TICKS_PER_NOTCH);
  lcd.setCursor(0, 1);
  lcd.print("Wait 10 turns...");
  sleep_microseconds(1 * 10000); // Wait until wheel is physically engaged
  long current_steps = dial_stepper.currentPosition();
  long dial_test_rounds = 10.0;
  move(dial_test_rounds * NOTCHES_PER_CAM);
  STEPS_PER_NOTCH = std::round(((dial_stepper.currentPosition() - current_steps) / dial_test_rounds) / NOTCHES_PER_CAM);

  // Advise STEPS_PER_NOTCH
  lcd.clear();
  lcd.print("Steps per notch");
  lcd.setCursor(0, 1);
  lcd.print(STEPS_PER_NOTCH);
  sleep_seconds(3);

  // Set notches per cam
  //// The user sets the dial to position "20". On a 100 notch dial where the notches are really 1.5 big, you point to "30".
  lcd.clear();
  lcd.print("Dial to twenty,");
  lcd.setCursor(0, 1);
  lcd.print(" or 30 for loose");
  long current_ticks = position;
  dial_stepper.disableOutputs();
  sleep_seconds(5);
  dial_stepper.enableOutputs();
  NOTCHES_PER_CAM = std::round(20.0 / ((position - current_ticks) / (ENCODER_TICKS_PER_NOTCH * NOTCHES_PER_CAM)));

  // Advise NOTCHES_PER_CAM
  lcd.clear();
  lcd.print("Notches per cam");
  lcd.setCursor(0, 1);
  lcd.print(NOTCHES_PER_CAM);
  sleep_seconds(3);

  // Done
  lcd.clear();
  lcd.print("Initialized.");
  move(0);
  position = 0;
  target = 0;
  dial_stepper.setCurrentPosition(0);
}

void loop() {
  while (true) {
    move(1);
    sleep_seconds(1);
    move(NOTCHES_PER_CAM);
    sleep_seconds(1);
  }
}

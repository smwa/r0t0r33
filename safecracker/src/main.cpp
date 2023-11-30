#include <Arduino.h>
#include <LiquidCrystal.h> // https://github.com/arduino-libraries/LiquidCrystal
#include <AccelStepper.h> // https://github.com/waspinator/AccelStepper

// Vocab
//// Notch is a position on a combination lock that may be part of a combination. Most locks have 3 cams that have 100 notches.
//// Cam is a disc in a combination lock. There are typically 3 cams in a combination lock.
//// Step is a stepper motor step.

// Hardware
int success_pin = 18;

LiquidCrystal lcd(13, 12, 25, 26, 27, 14);

int dial_stepper_pin_enable = 21; // Engage motor
int dial_stepper_pin_step = 22; // Step
int dial_stepper_pin_direction = 23; // Direction

AccelStepper dial_stepper(AccelStepper::DRIVER, dial_stepper_pin_step, dial_stepper_pin_direction);

// CONSTANTS
double MAX_SPEED = 2000.0000; // steps/second // 2000 is solid at half step
double ACCELERATION = MAX_SPEED * 4.0; // steps/second^2
double STEPS_PER_REVOLUTION = 400.0;
long NOTCHES_PER_CAM = 67;
double STEPS_PER_NOTCH = STEPS_PER_REVOLUTION / NOTCHES_PER_CAM;

// RUNNING VALUES
long position = 0; // in notches
long sequence_cam_one = 0; // Adjust when resuming
long sequence_cam_two = 0;
long sequence_cam_three = 0;

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

void move(long notches) {
  position += notches;
  dial_stepper.runToNewPosition(position * STEPS_PER_NOTCH);
}

void setup() {
  // Setup PC com
  Serial.begin(115200);

  // Setup success pin
  pinMode(success_pin, INPUT_PULLUP);

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
  dial_stepper.setPinsInverted(true, false, true); // Invert stepper pins if necessary
  dial_stepper.enableOutputs();

  // reset stepper motor and let the user know we've booted
  dial_stepper.runToNewPosition(16);
  dial_stepper.runToNewPosition(0);
}

long notches_to_notch(long target, long step) {
  long notches = 0;
  long _position = position;
  while (_position < NOTCHES_PER_CAM) _position += NOTCHES_PER_CAM;
  while (true) {
    notches += step;
    if ((notches + _position) % NOTCHES_PER_CAM == target) {
      break;
    }
  }
  return notches;
}

void loop() {
  lcd.clear();
  lcd.print("Dial to 0");
  dial_stepper.disableOutputs();
  sleep_microseconds(5 * 1000 * 1000);
  dial_stepper.enableOutputs();
  sleep_microseconds(100 * 1000);
  move(1);
  long _reset_notches = -1;

  for (long cam_one = sequence_cam_one; cam_one < NOTCHES_PER_CAM; cam_one++) {
    for (long cam_two = sequence_cam_two; cam_two < NOTCHES_PER_CAM; cam_two++) {
      for (long cam_three = sequence_cam_three; cam_three < NOTCHES_PER_CAM; cam_three++) {
        lcd.clear();
        lcd.print(cam_one);
        lcd.print(" ");
        lcd.print(cam_two);
        lcd.print(" ");
        lcd.print(cam_three);
        // one - Go right(negative) past 0 once(because we always reset) and then to cam_one
        move(_reset_notches - NOTCHES_PER_CAM - (NOTCHES_PER_CAM - cam_one));

        // two - One full revolution to the left and then to cam_two
        move(NOTCHES_PER_CAM + notches_to_notch(cam_two, 1));

        // three - Go right(negative) to cam_three
        move(notches_to_notch(cam_three, -1));

        // test
        if (digitalRead(success_pin) == HIGH) {
          sleep_microseconds(1 * 1000 * 1000);
        }

        // reset to 0
        _reset_notches = notches_to_notch(0, -1);
      }
    }
  }
}

#include <Arduino.h>
#include <LiquidCrystal.h> // https://github.com/arduino-libraries/LiquidCrystal
#include <NewEncoder.h>
#include <AccelStepper.h> // https://github.com/waspinator/AccelStepper

#include <math.h> /* round */
#include <stdlib.h> /* abs */

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

int dial_encoder_pin_zero = 16;
int dial_encoder_pin_one = 17;

void ESP_ISR dial_encoder_callback(NewEncoder *encPtr, const volatile NewEncoder::EncoderState *state, void *uPtr);
NewEncoder dial_encoder(dial_encoder_pin_zero, dial_encoder_pin_one, -15000, 15000, 0, FULL_PULSE);

// CONSTANTS
double MAX_SPEED = 900.0000; // steps/second // 2000 is solid at half step
double ACCELERATION = MAX_SPEED * 9.0; // steps/second^2
double STEPS_PER_CAM = 200.0;

long REAL_NOTCHES_PER_CAM = 100;
double NOTCH_LENIENCY = 1.5; // If you want to try every other notch, use 2.0
long NOTCHES_PER_CAM = round(REAL_NOTCHES_PER_CAM / NOTCH_LENIENCY);

long ENCODER_TICKS_PER_CAM = 600; // TODO
double STEPS_PER_NOTCH = STEPS_PER_CAM / NOTCHES_PER_CAM;
bool ENCODER_REVERSED = false;

double ALLOWED_ERROR = 0.2; // This is how many cam rotations. 0.2 is 20% of a dial turn

// RUNNING VALUES
long position = 0; // in notches
long encoder_position = 0;

// Adjust when resuming
long sequence_cam_one = NOTCHES_PER_CAM; // Cam 1 goes negative
long sequence_cam_two = 0;
long sequence_cam_three = NOTCHES_PER_CAM; // Cam 3 goes negative

void ESP_ISR dial_encoder_callback(NewEncoder *encPtr, const volatile NewEncoder::EncoderState *state, void *uPtr) {
  (void) encPtr;
  (void) uPtr;
  if (ENCODER_REVERSED) {
    encoder_position -= dial_encoder.getAndSet(0);
  }
  else {
    encoder_position += dial_encoder.getAndSet(0);
  }
}

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

double get_encoder_to_stepper_error() {
  return abs((encoder_position / ENCODER_TICKS_PER_CAM) - (dial_stepper.currentPosition() / STEPS_PER_CAM));
}

void move(long notches) {
  position += notches;
  dial_stepper.runToNewPosition(position * STEPS_PER_NOTCH);
  // Check error and bail
  if (get_encoder_to_stepper_error() > ALLOWED_ERROR) {
    while (true) {
      lcd.setCursor(0, 1);
      lcd.print("EncoderErr maxed");
      delay(10000);
    }
  }
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

  // Setup dial_encoder
  pinMode(dial_encoder_pin_zero, INPUT_PULLUP);
  pinMode(dial_encoder_pin_one, INPUT_PULLUP);
  dial_encoder.begin();
  dial_encoder.attachCallback(dial_encoder_callback);

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
  encoder_position = 0;
  move(1);

  long old_step_tracker = 0;

  for (long cam_one = sequence_cam_one; cam_one < NOTCHES_PER_CAM; cam_one++) {
    // Use encoder to find error and fix within dial_stepper internal state
    old_step_tracker = dial_stepper.currentPosition();
    dial_stepper.setCurrentPosition(round(STEPS_PER_CAM * encoder_position / ENCODER_TICKS_PER_CAM));
    lcd.setCursor(0, 1);
    lcd.print("StepFix ");
    lcd.print(dial_stepper.currentPosition() - old_step_tracker);

    // one - Go right(negative) past 0 once(because we always reset) and then to cam_one
    if (cam_one == sequence_cam_one) {
      move(notches_to_notch(cam_one, -1) - (3 * NOTCHES_PER_CAM));
    }
    else {
      move(notches_to_notch(cam_one, -1) - (2 * NOTCHES_PER_CAM));
    }

    for (long cam_two = sequence_cam_two; cam_two < NOTCHES_PER_CAM; cam_two++) {
      // two - One full revolution to the left and then to cam_two
      if (cam_two == sequence_cam_two) {
        // First time after bumping previous cam
        move((NOTCHES_PER_CAM * 2) + notches_to_notch(cam_two, 1));
      }
      else {
        // Bump cam 2 one notch
        move((NOTCHES_PER_CAM * 1) + notches_to_notch(cam_two, 1));
      }

      for (long cam_three = sequence_cam_three; cam_three >= 0; cam_three--) {
        lcd.setCursor(0, 0);
        lcd.print(round(cam_one * NOTCH_LENIENCY));
        lcd.print(" ");
        lcd.print(round(cam_two * NOTCH_LENIENCY));
        lcd.print(" ");
        lcd.print(round(cam_three * NOTCH_LENIENCY));
        lcd.print("   ");

        // three - Go right(negative) to cam_three
        if (cam_three == sequence_cam_three) {
          // First time
          move(notches_to_notch(cam_three, -1) - (1 * NOTCHES_PER_CAM));
        }
        else {
          // Not first time
          move(notches_to_notch(cam_three, -1));
        }

        // Turn dial to pull back latch, *specific to current lock*
        move(NOTCHES_PER_CAM);
        // check for success
        while (digitalRead(success_pin) == HIGH) {
          sleep_microseconds(100 * 1000 * 1000);
          lcd.setCursor(0, 1);
          lcd.print("SuccessPin LOW");
        }
        
        move(-1 * NOTCHES_PER_CAM);
        // check for success
        while (digitalRead(success_pin) == HIGH) {
          sleep_microseconds(100 * 1000 * 1000);
          lcd.setCursor(0, 1);
          lcd.print("SuccessPin LOW");
        }


      }
    }
  }
  while (true) {
    sleep_microseconds(4200000000);
  }
}

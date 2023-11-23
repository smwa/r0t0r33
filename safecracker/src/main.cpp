#include <Arduino.h>
#include <cmath>
#include <algorithm>
#include <NewEncoder.h> // https://github.com/gfvalvo/NewEncoder
#include <LiquidCrystal.h> // https://github.com/arduino-libraries/LiquidCrystal

// Notes

// A step every 250 uS is about max

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

int dial_stepper_current_direction = LOW;

// CONSTANTS, all will be tuned in initialization
double MAX_SPEED = 0.1000; // steps per millisecond
double ACCELERATION = 0.4; // percentage of speed to increase each millisecond
double STEPS_PER_NOTCH = 1.0; // Initial value needs to be low to force minimum speed
double ENCODER_TICKS_PER_NOTCH = 6.0;
long NOTCHES_PER_CAM = 67;

// RUNNING VALUES
long position = 0; // in encoder ticks. position in notches = position * ENCODER_TICKS_PER_NOTCH
double speed = 0.0; // steps per millisecond
long target = 0; // in notches
long dial_stepper_step = 0; // SIGNED
unsigned long time_of_next_step = 0;
unsigned long time_of_next_acceleration_change = 0;

unsigned long get_time_in_microseconds() {
  return micros();
}

void engage_stepper_dial(bool engage) {
  if (engage) {
    digitalWrite(dial_stepper_pin_enable, LOW);
  }
  else {
    digitalWrite(dial_stepper_pin_enable, HIGH);
  }
}

void sleep_microseconds(unsigned long microseconds) {
  delayMicroseconds(microseconds);
}

void step_stepper_dial(long steps) {
  dial_stepper_step += steps;
  if (steps < 0) {
    if (dial_stepper_current_direction == HIGH) {
      dial_stepper_current_direction = LOW;
      digitalWrite(dial_stepper_pin_direction, dial_stepper_current_direction);
      sleep_microseconds(250);
    }
    steps = -1 * steps;
  }
  else {
    if (dial_stepper_current_direction == LOW) {
      dial_stepper_current_direction = HIGH;
      digitalWrite(dial_stepper_pin_direction, dial_stepper_current_direction);
      sleep_microseconds(250);
    }
  }
  for (int i = 0; i < steps; i++) {
    digitalWrite(dial_stepper_pin_step, LOW);
    sleep_microseconds(2);
    digitalWrite(dial_stepper_pin_step, HIGH);
    sleep_microseconds(2);
  }
}

bool need_to_decelerate(double current_speed, double speed_minimum, double speed_to_change) {
  double _steps_per_ms_to_change = std::abs(speed_to_change);
  double _steps_per_ms = std::abs(current_speed);
  // Serial.println("pt2.3");
  _steps_per_ms = _steps_per_ms + _steps_per_ms_to_change;
  double _target_steps_per_ms = std::abs(speed_minimum);
  long _steps_remaining = std::abs(target - position * ENCODER_TICKS_PER_NOTCH) * STEPS_PER_NOTCH;
  // Serial.println("pt2.4");

  while (_steps_remaining > 0) {
    // Serial.println("pt2.5");
    // Serial.println(_steps_remaining);
    // Serial.println(_steps_per_ms);
    // Serial.println(_steps_per_ms_to_change);
    
    _steps_per_ms = std::max(speed_minimum, _steps_per_ms);
    _steps_remaining -= _steps_per_ms;
    _steps_per_ms -= _steps_per_ms_to_change;
  }
  // Serial.println("pt2.6");
  return (_steps_per_ms >= _target_steps_per_ms);
}

void step_if_ready() {
  unsigned long now = get_time_in_microseconds();
  if (now < time_of_next_step) {
    // Serial.println("Loop");
    // Serial.println(now);
    // Serial.println(time_of_next_step);
    return;
  }

  double speed_minimum = MAX_SPEED * ACCELERATION * 1; // TODO Maybe needs manual tuning

  long target_from_position = target - std::round(position / ENCODER_TICKS_PER_NOTCH);
  if (target_from_position == 0) {
    speed = 0.0;
  }
  Serial.println(target * ENCODER_TICKS_PER_NOTCH - position);

  // Do not step motor
  if (speed < speed_minimum && speed > -1.0 * speed_minimum) {
    time_of_next_step = now + 10; // Wait 10us
    time_of_next_acceleration_change = now + 1000; // Wait 1ms
    if (target_from_position > 0) {
      speed = speed_minimum;
    }
    else if (target_from_position < 0) {
      speed = -1.0 * speed_minimum;
    }
  }
  // Serial.println(speed);
  // Not using else because we're changing speed in the above "if"
  if (speed > 0.001) {
    step_stepper_dial(1);
  }
  else if (speed < -0.001) {
    step_stepper_dial(-1);
  }
  // Serial.println("pt1");
  // Accelerate or decelerate
  if (now >= time_of_next_acceleration_change) {
    // Serial.println("pt2");
    time_of_next_acceleration_change = now + 1000;
    // Serial.println("pt2.1");
    double speed_to_change = ACCELERATION * MAX_SPEED;
    // Serial.println("pt2.2");
    bool decelerate = need_to_decelerate(speed, speed_minimum, speed_to_change);
    // Serial.println("pt3");
    if (speed > 0) {
      // Serial.println("pt4");
      if (!decelerate) {
        // Serial.println("pt5");
        speed += speed_to_change;
        speed = std::min(MAX_SPEED, speed);
        // Serial.println("pt6");
      }
      else {
        // Serial.println("pt7");
        speed -= speed_to_change;
        speed = std::max(speed_minimum * 1.00000000001, speed);
        // Serial.println("pt8");
      }
    }
    else {
      if (!decelerate) {
        speed -= speed_to_change;
        speed = std::max(-1.0 * MAX_SPEED, speed);
      }
      else {
        speed += speed_to_change;
        speed = std::min(speed_minimum * -1.00000000001, speed);
      }
    }
  }
  // Serial.println("pt20");
  if (speed < speed_minimum && speed > -1.0 * speed_minimum) {
    time_of_next_step = now + 100;
  }
  else {
    // Serial.println(speed);
    // Serial.println(speed_minimum);
    // sleep_microseconds(3000000);
    time_of_next_step = now + std::floor(1000.0 / speed);
  }
  // Serial.println("pt21");
}

void move(double notches) {
  // Serial.println("Moving");
  long _notches = std::round(notches);
  target = std::round(position / ENCODER_TICKS_PER_NOTCH) + _notches;
  // Serial.println("entering while");
  while (target != std::round(position / ENCODER_TICKS_PER_NOTCH)) {
    // Serial.println("not there yet");
    step_if_ready();
  }
}

unsigned long check_time_required_to_move() {
  unsigned long now = get_time_in_microseconds();
  move(3 * NOTCHES_PER_CAM);
  return (get_time_in_microseconds() - now);
}

void custom_loop() {
  while (true) {
    move(1);
    sleep_microseconds(1 * 1000 * 1000);
    move(NOTCHES_PER_CAM);
    sleep_microseconds(1 * 1000 * 1000);
  }
}

void ESP_ISR dial_encoder_callback(NewEncoder *encPtr, const volatile NewEncoder::EncoderState *state, void *uPtr) {
  (void) encPtr;
  (void) uPtr;
  position += dial_encoder.getAndSet(0);
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
  sleep_microseconds(100 * 1000); // Let power flow
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
  engage_stepper_dial(true);
  pinMode(dial_stepper_pin_direction, OUTPUT);
  digitalWrite(dial_stepper_pin_direction, dial_stepper_current_direction);
  pinMode(dial_stepper_pin_step, OUTPUT);
  digitalWrite(dial_stepper_pin_step, HIGH);

  // TMP Test code
//   engage_stepper_dial(false);
//   for (int i = 0; i < 800; i++) {
//     lcd.clear();
//     lcd.print(position);
//     lcd.setCursor(0, 1);
//     lcd.print(i);
//     delay(10);
//   }

// engage_stepper_dial(true);
//   while (true) {
//     for (int i = 1500; i > 10; i--) {
//       for (int j = 0; j < 100000 / i; j++) {
//         digitalWrite(dial_stepper_pin_step, LOW);
//         sleep_microseconds(2);
//         digitalWrite(dial_stepper_pin_step, HIGH);
//         sleep_microseconds(i);
//       }
//       // Serial.println(i);
//     }
//   }

// engage_stepper_dial(true);
// while (true) {
//   Serial.println("steppy");
// step_stepper_dial(-1);
// sleep_microseconds(100000);
// }

  // reset stepper motor and let the user know we've booted
  move(1);
  move(-1);

  // Find ticks per cam, to set ticks per notch, even though we don't know how many notches yet.
  lcd.clear();
  lcd.print("Dial to zero");
  engage_stepper_dial(false);
  sleep_microseconds(5 * 1000000);
  engage_stepper_dial(true);
  position = 0;
  target = 0;
  lcd.clear();
  lcd.print("Wait...");
  move(1);
  move(-1);
  lcd.clear();
  lcd.print("Dial FORWARD to");
  lcd.setCursor(0, 1);
  lcd.print("next zero");
  engage_stepper_dial(false);
  sleep_microseconds(5 * 1000000);
  engage_stepper_dial(true);
  lcd.clear();
  lcd.print("Wait 9 turns...");
  // Set rough estimate
  ENCODER_TICKS_PER_NOTCH = position / NOTCHES_PER_CAM;
  move(9.0 * NOTCHES_PER_CAM);
  lcd.clear();
  lcd.print("Dial to");
  lcd.setCursor(0, 1);
  lcd.print("nearest zero");
  engage_stepper_dial(false);
  sleep_microseconds(5 * 1000000);
  engage_stepper_dial(true);
  // Set more precise number
  ENCODER_TICKS_PER_NOTCH = std::round((position / NOTCHES_PER_CAM) / 10.0);

  // Find steps per notch
  lcd.clear();
  lcd.print(ENCODER_TICKS_PER_NOTCH);
  lcd.setCursor(0, 1);
  lcd.print("Wait 10 turns...");
  sleep_microseconds(1 * 10000); // Wait until wheel is physically engaged
  long current_steps = dial_stepper_step;
  move(10.0 * NOTCHES_PER_CAM);
  long steps_per_ten_rounds = dial_stepper_step - current_steps;
  STEPS_PER_NOTCH = std::round((steps_per_ten_rounds / NOTCHES_PER_CAM) / 10.0);

  // Advise STEPS_PER_NOTCH
  lcd.clear();
  lcd.print("Steps per notch");
  lcd.setCursor(0, 1);
  lcd.print(STEPS_PER_NOTCH);
  sleep_microseconds(3 * 1000000);

  // Set notches per cam
  //// The user sets the dial to position "20". On a 100 notch dial where the notches are really 1.5 big, you point to "30".
  lcd.clear();
  lcd.print("Dial to twenty,");
  lcd.setCursor(0, 1);
  lcd.print(" or 30 for loose");
  long current_ticks = position;
  engage_stepper_dial(false);
  sleep_microseconds(5 * 1000000);
  engage_stepper_dial(true);
  NOTCHES_PER_CAM = std::round(20.0 / ((position - current_ticks) / (ENCODER_TICKS_PER_NOTCH * NOTCHES_PER_CAM)));

  // Advise NOTCHES_PER_CAM
  lcd.clear();
  lcd.print("Notches per cam");
  lcd.setCursor(0, 1);
  lcd.print(NOTCHES_PER_CAM);
  sleep_microseconds(3 * 1000000);

  // Tune speed
  lcd.clear();
  lcd.print("Tuning speed...");
  lcd.setCursor(0, 1);
  lcd.print(MAX_SPEED);
  unsigned long best_time = check_time_required_to_move();
  MAX_SPEED *= 2.0;
  lcd.setCursor(0, 1);
  lcd.print(MAX_SPEED);
  unsigned long trial_time = check_time_required_to_move();
  while (trial_time <= best_time) {
    best_time = trial_time;

    // Next trial
    MAX_SPEED *= 2.0;
    lcd.setCursor(0, 1);
    lcd.print(MAX_SPEED);
    trial_time = check_time_required_to_move();
  }
  MAX_SPEED = MAX_SPEED / 2.0;

  // Tune acceleration
  lcd.clear();
  lcd.print("Tuning accel...");
  lcd.setCursor(0, 1);
  lcd.print(ACCELERATION);
  best_time = check_time_required_to_move();
  ACCELERATION *= 2.0;
  lcd.setCursor(0, 1);
  lcd.print(ACCELERATION);
  trial_time = check_time_required_to_move();
  while (trial_time <= best_time) {
    best_time = trial_time;

    // Next trial
    ACCELERATION *= 2.0;
    lcd.setCursor(0, 1);
    lcd.print(ACCELERATION);
    trial_time = check_time_required_to_move();
  }
  ACCELERATION = ACCELERATION / 2.0;
  lcd.clear();
  lcd.print("Initialized.");

  custom_loop();
}

void loop() {

}

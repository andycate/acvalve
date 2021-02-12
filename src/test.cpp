#include <Arduino.h>

#define STEP_PIN 25
#define DIR_PIN 24
#define RST_PIN 32

#define MIN_DELAY 75 // imperical data
#define RAMP_RATE 1000 // imperical
#define START_RATE 300

float pos;
float sp;
float freq;
float max_freq;
int step;
uint32_t last_time;
uint32_t last_c;
uint32_t de;
uint8_t state;
uint32_t d;
uint32_t steps;

uint32_t get_delay(uint32_t end) {
  if(steps < RAMP_RATE - START_RATE) {
    return (uint32_t) (MIN_DELAY * RAMP_RATE / (float) (steps + START_RATE));
  } else if(steps > end - RAMP_RATE + START_RATE) {
    return (uint32_t) (MIN_DELAY * RAMP_RATE / (float) (end - steps + START_RATE));
  }
  return MIN_DELAY;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(RST_PIN, OUTPUT);

  pos = 0.0;
  sp = 10000.0;
  max_freq = 15000.0;
  freq = 100.0;
  step = 1;
  last_time = micros();
  last_c = micros();
  state = 0;
  d = 50;
  de = 5000;
  steps = 0;

  digitalWriteFast(RST_PIN, LOW);
  digitalWriteFast(DIR_PIN, HIGH);
  // analogWriteFrequency(STEP_PIN, freq);
  // analogWrite(STEP_PIN, 127); // half duty cycle by default
  // digitalWriteFast(STEP_PIN, HIGH);
  pinMode(38, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWriteFast(RST_PIN, HIGH);
  digitalWriteFast(DIR_PIN, LOW);

  // if(Serial.available()) {
  //   float new_freq = Serial.readStringUntil('\n').toFloat();
  //   analogWriteFrequency(STEP_PIN, new_freq);
  // }
  // uint32_t curr_time = micros();
  // if(curr_time - last_time > d) {
  //   state = state ? 0 : 1;
  //   digitalWriteFast(STEP_PIN, state);
  //   last_time = curr_time;
  // }

  

  // if(curr_time - last_c > de * (1000/d)) {
  //   if(d < 20) step = 1;
  //   if(d > 1000) step = -1;
  //   d += step;
  //   last_c = curr_time;
  // }
  
//   uint32_t curr_time = micros();
//   pos += (float) (curr_time - last_time) * freq / 1000000.0;
//   last_time = curr_time;

//   freq = (sp - pos) * 1.2;
//   analogWriteFrequency(STEP_PIN, min(abs(freq), max_freq));
//   Serial.println(pos);

  if(steps < (uint32_t) ((90.0 * 200.0 / 360.0) * 16.0 * 5.18)) {
    digitalWriteFast(STEP_PIN, HIGH);
    delayNanoseconds(1000);
    digitalWriteFast(STEP_PIN, LOW);
    delayMicroseconds(get_delay((uint32_t) ((90.0 * 200.0 / 360.0) * 16.0 * 5.18)));
    steps++;
  }

  // Serial.println(analogRead(38) * 3.3 / 1024.0);
  
}

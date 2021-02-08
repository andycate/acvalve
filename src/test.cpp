#include <Arduino.h>

#define STEP_PIN 25
#define DIR_PIN 24
#define RST_PIN 32

float pos;
float sp;
float freq;
float max_freq;
uint32_t last_time;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(RST_PIN, OUTPUT);

  pos = 0.0;
  sp = 10000.0;
  max_freq = 15000.0;
  freq = max_freq;
  last_time = micros();

  digitalWriteFast(RST_PIN, LOW);
  digitalWriteFast(DIR_PIN, HIGH);
  analogWriteFrequency(STEP_PIN, 2000.0);
  analogWrite(STEP_PIN, 127); // half duty cycle by default
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWriteFast(RST_PIN, HIGH);
  digitalWriteFast(DIR_PIN, HIGH);
  
//   uint32_t curr_time = micros();
//   pos += (float) (curr_time - last_time) * freq / 1000000.0;
//   last_time = curr_time;

//   freq = (sp - pos) * 1.2;
//   analogWriteFrequency(STEP_PIN, min(abs(freq), max_freq));
//   Serial.println(pos);

//   digitalWriteFast(STEP_PIN, HIGH);
//   delay(10);
//   digitalWriteFast(STEP_PIN, LOW);
//   delay(10);
  
}

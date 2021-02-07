#include <Arduino.h>

#define STEP_PIN 0
#define MS1_PIN 1
#define MS2_PIN 2
#define MS3_PIN 3
#define DIR_PIN 4

#define LP_PIN 23
#define ADC_RES 1024.0
#define MIN_PT_VOLTAGE 0.5
#define MAX_PT_VOLTAGE 4.5
#define MAX_ADC_VOLTAGE 3.3
#define MIN_ADC_VOLTAGE (MIN_PT_VOLTAGE / MAX_ADC_VOLTAGE)
#define MAX_PRESSURE 300.0
#define PPV (MAX_PT_VOLTAGE - MIN_PT_VOLTAGE) * MAX_ADC_VOLTAGE / (MAX_PRESSURE * MAX_PT_VOLTAGE)

#define CONST_P 0.1
#define CONST_I 0.0
#define CONST_D 0.0

uint32_t curr_time;
uint32_t last_time;
uint32_t t_diff;
float pressure;
float last_pressure;
float p_diff;
float setpoint;
float speed;
float integrator;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(MS1_PIN, OUTPUT);
  pinMode(MS2_PIN, OUTPUT);
  pinMode(MS3_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  
  pinMode(LP_PIN, INPUT);

  // set step mode to 16th by default
  digitalWriteFast(MS1_PIN, HIGH);
  digitalWriteFast(MS2_PIN, HIGH);
  digitalWriteFast(MS3_PIN, HIGH);

  curr_time = micros();
  last_time = micros();
  t_diff = 0;
  pressure = 0.0;
  last_pressure = 0.0;
  p_diff = 0.0;
  setpoint = 0.0;
  speed = 0.0;
  integrator = 0.0;

  analogWriteFrequency(STEP_PIN, 0.0);
  analogWrite(STEP_PIN, 127); // half duty cycle by default
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available()) setpoint = Serial.readStringUntil('\n').toFloat();

  pressure = ((float) analogRead(LP_PIN) * MAX_ADC_VOLTAGE / ADC_RES - MIN_ADC_VOLTAGE) * PPV;
  curr_time = micros();
  t_diff = (float) (curr_time - last_time);
  last_time = curr_time;
  p_diff = pressure - last_pressure;
  integrator += p_diff / t_diff;

  speed = CONST_P * p_diff + CONST_I * integrator + CONST_D * p_diff / t_diff;
  if(speed < 0) digitalWriteFast(DIR_PIN, LOW);
  else digitalWriteFast(DIR_PIN, HIGH);
  analogWriteFrequency(STEP_PIN, speed);
}
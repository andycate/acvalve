#include <Arduino.h>

#define GEAR_RATIO 26.85
#define DEG_PER_STEP 1.8 / (GEAR_RATIO * 16.0)

#define STEP_PIN 25
#define DIR_PIN 24
#define SENSOR_PIN 38

#define SEND_INTERVAL 50000

int position = 0;
bool direction = true; // true is forward (opening valve)

void step(bool dir) {
    if(dir != direction) {
        direction = dir;
        digitalWriteFast(DIR_PIN, direction);
        delayNanoseconds(200);
    }
    digitalWriteFast(STEP_PIN, HIGH);
    delayNanoseconds(1000);
    digitalWriteFast(STEP_PIN, LOW);
    delayNanoseconds(1000);
    position += direction ? 1 : -1;
}

float angle() {
    return (float) position * DEG_PER_STEP;
}

float lpressure() {
    return ((analogRead(SENSOR_PIN) * 3.3 / 1024.0) - (0.5 * 3.3 / 5.0)) * (300 / 2.64);
}

int main(int argc, char** argv) {
    float deg_sp = 0.0;
    uint8_t state = 0; // 0: disabled, 1: running, 2: go to angle
    uint32_t last_send = micros();
    uint32_t last_step = micros();

    Serial.begin(115200);

    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(SENSOR_PIN, INPUT);
    digitalWriteFast(DIR_PIN, true);

    while(1) {
        // loop
        if(Serial.available()) {
            String packet = Serial.readStringUntil('\n'); // s, p, d
            char command = packet.substring(0, 1).c_str()[0];
            Serial.println(command);
            switch(command) {
                case 'p':
                    state = 0;
                    break;
                case 'd':
                    deg_sp = packet.substring(2).toFloat();
                    state = 2;
                    break;
                default: break;
            }
        }

        switch(state) {
            case 0: break;
            case 1:
                // float press = lpressure();
                // uint32_t now = micros();
                // if(press < 10.0) {
                //     if(now - last_step > (1.0 / ((10.0 - press) + 1.0)) * 100) {
                //         step(true);
                //         last_step = now;
                //     }
                // } else if(press > 10.0) {
                //     step(false);
                // }
                break;
            case 2:
                float ang = angle();
                if(abs(ang - deg_sp) < DEG_PER_STEP) {
                    state = 0;
                    break;
                }
                if(ang < deg_sp) {
                    step(true);
                } else if(ang > deg_sp) {
                    step(false);
                }
                delayMicroseconds(40);
                break;
            default: break;
        }

        uint32_t send_now = micros();
        if(send_now - last_send > SEND_INTERVAL) {
            Serial.println(String(angle()) + "," + String(lpressure()));
            last_send = send_now;
        }

        yield();
    }
    return 0;
}

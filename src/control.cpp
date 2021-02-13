#include <Arduino.h>
#include <vector>
#include <queue>
#include <deque>

#define KP_PRESS 0.6
#define KP_RATE 1.2

#define GEAR_RATIO 26.85
#define DEG_PER_STEP (1.8 / (GEAR_RATIO * 16.0))
#define MAX_POSITION (uint32_t) (90.0 / (DEG_PER_STEP))
#define MIN_STEP_DELAY 40

#define STEP_PIN 25
#define DIR_PIN 24
#define SENSOR_PIN 38

#define AVG_SAMPLES 256

#define SEND_INTERVAL 50000

using namespace std;

struct Task {
    uint32_t when;
    uint32_t period; // milliseconds between executions
    void (*func)(uint32_t);

    bool operator >(const Task& a) const {
        return when > a.when;
    }
    bool operator <(const Task& a) const {
        return when < a.when;
    }
    bool operator ==(const Task& a) const {
        return when == a.when;
    }
};

priority_queue<Task, vector<Task>, greater<Task>> taskq;

int position = 0;
bool direction = true; // true is forward (opening valve)
float speed = 0.0; // rpm
uint32_t last_pulse = 0;
uint8_t state = 0;
float deg_sp = 0.0;
deque<float> press_deque = deque<float>(AVG_SAMPLES);
float press_avg = 0.0;
float press_der = 0.0;

float press_sp = 30.0;

// utilities
void step(bool dir) {
    if(dir != direction) {
        direction = dir;
        digitalWriteFast(DIR_PIN, direction);
        delayNanoseconds(200);
    }
    if(state == 1) {
        if(direction) {
            if(position >= MAX_POSITION) return;
        } else {
            if(position <= 0) return;
        }
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

float hpressure() {
    return 100.0; // PSI
}

float lpressure() {
    #define CONST_PRESS_OFFSET (int) (0.5 * 1024.0 / 5.0)
    #define CONST_PRESS_FACTOR 5.0 * 300.0 / (1024.0 * (4.5 - 0.5))
    return (analogRead(SENSOR_PIN) - CONST_PRESS_OFFSET) * CONST_PRESS_FACTOR;
}

// subsystems
void listen(uint32_t last_time) {
    if(Serial.available()) {
        String packet = Serial.readStringUntil('\n'); // s, p, d
        char command = packet.substring(0, 1).c_str()[0];
        Serial.println(command);
        switch(command) {
            case 'p':
                state = 0;
                speed = 0.0;
                break;
            case 'c':
                state = 1;
                speed = 0.0;
                break;
            case 'd':
                deg_sp = packet.substring(2).toFloat();
                state = 2;
                speed = 0.0;
                break;
            case 'r':
                deg_sp = deg_sp += packet.substring(2).toFloat();
                state = 2;
                speed = 0.0;
                break;
            case 's':
                speed = packet.substring(2).toFloat();
                break;
            case 'z':
                state = 0;
                position = 0;
                speed = 0.0;
                break;
            case 't':
                press_sp = packet.substring(2).toFloat();
                break;
            default: break;
        }
    }
}

void report(uint32_t last_time) {
    Serial.println(String(state) + "," + String(angle()) + "," + String(lpressure()) + "," + String(press_der));
}

void div_calc(uint32_t last_time) {
    float press = lpressure();
    float new_avg = press_avg + (press - press_deque.front()) / AVG_SAMPLES;
    press_der = (new_avg - press_avg) / 0.001;

    press_deque.pop_front();
    press_deque.push_back(press);
}

void control(uint32_t last_time) {
    if(state == 0);
    else if(state == 1) {
        float lpress = lpressure();
        // float hpress = hpressure();
        float rate_sp = (press_sp - lpress) * KP_PRESS;
        speed = (rate_sp - press_der) * KP_RATE;
    } else if(state == 2) {
        float ang = angle();
        if(abs(ang - deg_sp) < DEG_PER_STEP) {
            state = 0;
            return;
        }
        if(ang < deg_sp) {
            step(true);
        } else if(ang > deg_sp) {
            step(false);
        }
    } else;
}

void motor() {
    #define SPEED_FACTOR (1000000.0 / (360.0 / 60.0 / DEG_PER_STEP))
    if(speed == 0.0) return;
    uint32_t delay = max((uint32_t) (SPEED_FACTOR / abs(speed)), MIN_STEP_DELAY); // micros per pulse
    uint32_t now = micros();
    if(now - last_pulse > delay) {
        step(speed > 0.0);
        last_pulse = now;
    }
}

int main(int argc, char** argv) { // 0: disabled, 1: running, 2: go to angle

    Serial.begin(115200);

    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(SENSOR_PIN, INPUT);
    digitalWriteFast(DIR_PIN, true);

    // set up tasks
    Task rep = {.when = micros(), .period = 50000, .func = report};
    taskq.push(rep);
    Task lis = {.when = micros(), .period = 50000, .func = listen};
    taskq.push(lis);
    Task div = {.when = micros(), .period = 1000, .func = div_calc};
    taskq.push(div);
    Task con = {.when = micros(), .period = 1000, .func = control};
    taskq.push(con);

    while(1) {
        // loop
        if(taskq.top().when <= micros()) {
            Task next = taskq.top();
            taskq.pop();
            taskq.push({.when = micros() + next.period, .period = next.period, .func = next.func});
            next.func(next.when - next.period);
        }
        motor();

        yield();
    }
    return 0;
}

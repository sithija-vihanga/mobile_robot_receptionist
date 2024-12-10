#ifndef HUBWHEEL_H
#define HUBWHEEL_H

#include <Arduino.h>

// status of the wheel (direction)
#define FORWARD 1
#define BACKWARD 0
#define STOP 3

// ZF value according to wheel type
#define RIGHT_FORWARD 0
#define RIGHT_BACKWARD 1
#define LEFT_FORWARD 1
#define LEFT_BACKWARD 0

// Wheel types
#define RIGHT_WHEEL 0
#define LEFT_WHEEL 1

// Maximum encoder count
#define MAX_COUNT 1000

// Converting parameters
#define MILIMETERS_PER_TICK 5.4598

// PID parameters
#define MAX_PWM 255
#define KI 5
#define KP 8

class HubWheel{
    private:
        // Pins for the motor
        /*
            *SIGNAL: Encoder pulse signal
            *ZF: Direction change (0 or 1)
            *VR: Speed controlling pin
            *EL: Enable pin 
        */ 
        uint8_t SIGNAL,ZF,VR,EL,wheel_type;

    public:

        volatile float velocity = 0, target_velocity = 0;
        float sum_of_errors = 0;
        int pwm_vel = 0, pre_pwm_vel = 0;
        int direction = STOP;
        volatile int count = 0, pre_count = 0;
        volatile long t = 0;

        HubWheel(uint8_t SIGNAL_, uint8_t ZF_, uint8_t VR_, uint8_t EL_, uint8_t wheel_type_);
        void drive();
        void calPWM();
};

#endif
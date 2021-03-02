#include "mbed.h"
#include "motor.h"

DigitalOut led1(PC_13);

CAN can1(PA_11, PA_12, 100000);

InterruptIn encoderLeftA(PA_8);
InterruptIn encoderLeftB(PA_9);
IGVCMotor motorLeft(PA_0, PA_1, PB_0);

InterruptIn encoderRightA(PC_6);
InterruptIn encoderRightB(PC_7);
IGVCMotor motorRight(PA_2, PA_3, PB_1);

Ticker ticker;

void updateLeft() {
    motorLeft.pulse(encoderLeftA.read(), encoderLeftB.read());
}

void updateRight() {
    motorRight.pulse(encoderRightA.read(), encoderRightB.read());
}

void tick() {
    motorLeft.update();
    motorRight.update();
}

int main()
{
    // create poll
    ticker.attach(&tick, 1.0/MOTOR_UPDATE_RATE);
    
    // attach all interrupts
    encoderLeftA.rise(&updateLeft);
    encoderLeftA.fall(&updateLeft);
    // encoderLeftB.rise(&updateLeft);
    // encoderLeftB.fall(&updateLeft);
    encoderRightA.rise(&updateRight);
    encoderRightA.fall(&updateRight);
    // encoderRightB.rise(&updateRight);
    // encoderRightB.fall(&updateRight);

    printf("main()\n");
    CANMessage msg;

    signed char l;
    signed char r;

    while(1) {
        if(can1.read(msg)) {
            printf("Message received: %d\n", msg.data[0]);
            l = msg.data[0];
            r = msg.data[1];

            motorLeft = (float)(l/128);
            motorRight = (float)(r/128);
            led1 = !led1;
        } 
    }
}
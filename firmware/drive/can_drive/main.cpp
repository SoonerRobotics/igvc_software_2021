#include "mbed.h"
#include "motor.h"




DigitalOut led1(PC_13);

CAN can1(PA_11, PA_12, 100000); // init can

InterruptIn encoderLeftA(PA_8); //init left encoder a pin
InterruptIn encoderLeftB(PA_9); // init left encoder b pin

DigitalOut motorLeftA(PA_0);
DigitalOut motorLeftB(PA_1);
IGVCMotor motorLeft(&motorLeftA, &motorLeftB, PB_0); // init left motor
 
InterruptIn encoderRightA(PC_6); //init right encoder a
InterruptIn encoderRightB(PC_7); //init right encoder b

DigitalOut motorRightA(PA_2);
DigitalOut motorRightB(PA_3);


IGVCMotor motorRight(&motorRightA, &motorRightB, PB_1); //init  right motor

Ticker ticker;
Ticker comms;


void updateLeft() {
    motorLeft.pulse(encoderLeftA.read(), encoderLeftB.read());
}

void updateRight() {
    motorRight.pulse(encoderRightA.read(), encoderRightB.read());
}

void tick(void) {
    motorLeft.update();
    motorRight.update();
}

struct char_speed{
    signed char left;
    signed char right;
    unsigned char speed;
};




int main()
{
    motorRightA = 0;
    motorRightB = 0;

    motorLeftA = 0;
    motorLeftB = 0;

    // create poll
    ticker.attach(tick, 10000us);
    
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


    
    char speed2 = 0;
    



    char speed[3];

    signed char left_speed;
    signed char right_speed;
    speed[2] = 52;
    struct char_speed input;

    motorLeft = 2.0f;
    motorRight = 2.0f;

    while(1) {
            
    


    left_speed = motorLeft.getSpeedEstimate() * 128 / 5.2;
    right_speed = motorRight.getSpeedEstimate() * 128 / 5.2;
    
    speed[0] = (char) left_speed;
    speed[1] = (char) right_speed;

    if(can1.read(msg)){
        input = *(struct char_speed*) msg.data;

       motorLeft = input.left * input.speed / 10.0 / 128.0;
       motorRight = input.right * input.speed / 10.0 / 128.0;

        if(can1.write(CANMessage(11, speed, 3))) {
                        led1 = !led1;
        } 
    }    
            

    }   
}
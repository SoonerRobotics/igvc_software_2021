#include "mbed.h"
#include "motor.h"
#include <ratio>




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

bool is_stopped = true;

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
    std::chrono::microseconds update_period_microseconds((long long)(1000000/MOTOR_UPDATE_RATE));
    ticker.attach(tick, update_period_microseconds);
    
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

    motorLeft = 0.0f;
    motorRight = 0.0f;

    

    while(1) {
        left_speed = motorLeft.getSpeedEstimate() * 128 / 5.2;
        right_speed = motorRight.getSpeedEstimate() * 128 / 5.2;
        
        speed[0] = (char) left_speed;
        speed[1] = (char) right_speed;

        // Reads the next message in queue in the can mailbox. 
        if(can1.read(msg)){
        
        float tune_float = *(float *)&msg.data;
        char motor_choice = msg.data[5];
        
        switch(msg.id) {
            case 0 :// This means that we need to hit the nuclear button, and the robot needs to shut down immediately. 
                motorLeft = 0.0f;
                motorRight = 0.0f;
                motorLeft.tuneP(0.0f);
                motorLeft.tuneI(0.0f);
                motorLeft.tuneD(0.0f);
                motorRight.tuneP(0.0f);
                motorRight.tuneI(0.0f);
                motorRight.tuneD(0.0f);
                ticker.detach();
                motorLeft.brake();
                motorRight.brake();  

                while(true); //creates infinite loop that requires a reset to fix. 
                return -1;
                break;
            case 1: // mobi stop
                motorLeft = 0.0f;
                motorRight = 0.0f;
                is_stopped = true;
                break;
            case 9: //mobi start
                is_stopped = false;
                break;
            case 10:        // This checks to see if the msg id is 10 which is the message that controls the speed of the motors.
                if(!is_stopped){
                    //sets the char array for speed control.
                    input = *(struct char_speed*) msg.data;
                    motorLeft = input.left * input.speed / 10.0 / 128.0;
                    motorRight = input.right * input.speed / 10.0 / 128.0;
                }
                break;
            case 100:
                tune_float = *(float *)&msg.data;
                motor_choice = msg.data[5];
                if(motor_choice == 0)  // left motor
                        motorLeft.tuneP(tune_float);
                else if(motor_choice ==1) //right motor
                        motorRight.tuneP(tune_float);
                else if(motor_choice == 2){
                    motorLeft.tuneP(tune_float);
                    motorRight.tuneP(tune_float);
                } 
                break;
            case 101:  // This checks to see if the msg id is 101 which is the message that tunes the I of the PID.
                tune_float = *(float *)&msg.data;
                motor_choice = msg.data[5];
                if(motor_choice == 0)  // left motor
                        motorLeft.tuneI(tune_float);
                else if(motor_choice ==1) //right motor
                        motorRight.tuneI(tune_float);
                else if(motor_choice == 2){
                    motorLeft.tuneI(tune_float);
                    motorRight.tuneI(tune_float);
                }
                break;
            case 102: // This checks to see if the msg id is 102 which is the message that tunes the D of the PID.
                float tune_float = *(float *)&msg.data;
                char motor_choice = msg.data[5];
                if(motor_choice == 0)  // left motor
                        motorLeft.tuneD(tune_float);
                else if(motor_choice ==1) //right motor
                        motorRight.tuneD(tune_float);
                else if(motor_choice == 2){
                    motorLeft.tuneD(tune_float);
                    motorRight.tuneD(tune_float);
                }
                break;
            }
            if(can1.write(CANMessage(11, speed, 3))) led1 = !led1; //writes the estimated speed to the can bus.
        }    
    }
}
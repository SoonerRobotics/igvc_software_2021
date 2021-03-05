#include "mbed.h"
#include "ArduinoJson.h"
#include "IGVCMotor.h"

//InterruptIn encoderLeftA(PB_4);
//InterruptIn encoderLeftB(PB_5);
InterruptIn encoderLeftA(PA_11);
InterruptIn encoderLeftB(PA_8);
IGVCMotor motorLeft(PB_6, PB_1, PA_1);

InterruptIn encoderRightA(PB_4);
InterruptIn encoderRightB(PB_5);
IGVCMotor motorRight(PB_0, PB_7, PA_3);

Serial pc(SERIAL_TX, SERIAL_RX);
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
    
    pc.baud(115200);
    wait(0.01);
        
    StaticJsonDocument<128> doc;
    char rawJson[128];
    
    while (1) {
        if (pc.readable()) {
            pc.scanf("%s", rawJson);
            DeserializationError err = deserializeJson(doc, rawJson);
            
            if (!err) { 
                if (doc.containsKey("motorLeft"))
                    motorLeft = (float)doc["motorLeft"];
                    
                if (doc.containsKey("motorRight"))
                    motorRight = (float)doc["motorRight"];
                    
                if (doc.containsKey("tune_p")) {
                    motorLeft.tuneP((float)doc["tune_p"]);
                    motorRight.tuneP((float)doc["tune_p"]);
                }
                    
                if (doc.containsKey("tune_i")) {
                    motorLeft.tuneI((float)doc["tune_i"]);
                    motorRight.tuneI((float)doc["tune_i"]);
                }
                    
                if (doc.containsKey("tune_d")) {
                    motorLeft.tuneD((float)doc["tune_d"]);
                    motorRight.tuneD((float)doc["tune_d"]);     
                }     
            }
            
            printf("%f,%f,%f,%f\n", motorLeft.getSpeedEstimate(), motorLeft.getTargetSpeed(), motorRight.getSpeedEstimate(), motorRight.getTargetSpeed());
        }
    }
}
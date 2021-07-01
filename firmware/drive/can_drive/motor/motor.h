#ifndef MOTORH
#define MOTORH
#include "mbed.h"
#define MOTOR_UPDATE_RATE 80 // Frequency that motor PID is updated (Hz)
#define MAX_SPEED 2.2f // (m/s)
#define PULSES_PER_REV 2400 // (revs)
#define LINEAR_PER_REV 0.2054f // Wheel radius (m)
#define MILLIS_TO_FULL 80 // Milliseconds to go from 0 output speed to 1
#define LPIIR_DECAY 0.1f // Decay rate of low pass filter on velocity

#define PI 3.14159265f
#define INCREMENT_AMT (1000.0f / (MILLIS_TO_FULL * MOTOR_UPDATE_RATE))

#define PREV_MASK 0x1 // Mask for the previous state in determining direction of rotation.
#define CURR_MASK 0x2 // Mask for the current state in determining direction of rotation.
#define INVALID   0x3 // XORing two states where both bits have changed.

class IGVCMotor {
    public:
        IGVCMotor(DigitalOut* motorA, DigitalOut* motorB, PinName pwm){
            this->motorA =  motorA;
            this->motorB = motorB;
            this->pwmPin = new PwmOut(pwm);
            
            this->pwmPin->period(1.0/500.0); // 15 kHz frequency
            
            this->pulses = 0;
            this->targetSpeed = 0.0f;
            this->curOutput = 0.0f;
            this->speedEstimate = 0.0f;
            
            // PID values
            this->integrator = 0.0f;
            this->error = 0.0f;
            
            // States
            this->prev_error = 0.0f;
            this->last_state = 0.0f;
            
            // Equation constants
            this->kP = 0.05f;
            this->kI = 0.0f;
            this->kD = 0.0001f;
        }
        
        void output(float speed) {
            this->targetSpeed = speed;
        }
        
        // Stolen from QEI library
        void pulse(int left, int right) {
            currState_ = (left << 1) | (right);
            
            //11->00->11->00 is counter clockwise rotation or "forward".
            if ((prevState_ == 0x3 && currState_ == 0x0) ||
                    (prevState_ == 0x0 && currState_ == 0x3)) {
                this->pulses++;
            }
            //10->01->10->01 is clockwise rotation or "backward".
            else if ((prevState_ == 0x2 && currState_ == 0x1) ||
                     (prevState_ == 0x1 && currState_ == 0x2)) {
                this->pulses--;
            }
            
            prevState_ = currState_;
        }

        void brake(){
            motorA->write(0);
            motorB->write(0);
            pwmPin->write(0.0f);
        }
        
        // Poll to update PID
        void update() {
            float instantaneousSpeed = this->pulses / (float)PULSES_PER_REV * 2.0 * PI * LINEAR_PER_REV * MOTOR_UPDATE_RATE;
            this->speedEstimate += (1.0f - LPIIR_DECAY) * (instantaneousSpeed - this->speedEstimate); // Low pass filter our speed estimate
            
            float output = this->updatePID(this->targetSpeed, this->speedEstimate);
            curOutput += output;

            //printf("%f,%f,%f\n\r", this->targetSpeed, this->pulses / (float)PULSES_PER_REV * 2.0 * PI * LINEAR_PER_REV * MOTOR_UPDATE_RATE, curOutput);
            
            this->pulses = 0;
            
            if (curOutput < 0.01f && curOutput > -0.01f) { // break
                motorA->write(0);
                motorB->write(0);
                pwmPin->write(0.0f);
            } else if (curOutput > 0.0f) { // forward
                motorA->write(0);
                motorB->write(1);
                pwmPin->write(0.05f + curOutput * 0.30f);
            } else { // reverse
                motorA->write(1);
                motorB->write(0);
                pwmPin->write(0.05f + (-curOutput) * 0.30f); // curOutput is negative so it subtracts
            }  
        }
        
        void tuneP(float p) {
            this->kP = p;
        }
        
        void tuneI(float i) {
            this->kI = i;
        }
        
        void tuneD(float d) {
            this->kD = d;
        }
        
        float getSpeedEstimate() {
            return this->speedEstimate;
        }
        
        IGVCMotor& operator= (float v) {
            output(v);
            return *this;
        }
        
    private:
        DigitalOut* motorA;
        DigitalOut* motorB;
        PwmOut* pwmPin;
        
        int prevState_;
        int currState_;
        volatile int pulses;
        
        float curOutput;
        float targetSpeed;
        
        float speedEstimate;
        
        // PID Values
        float integrator;
        float error;

        // States
        float prev_error;
        float last_state;

        // Equation coefficients
        float kP;
        float kI;
        float kD;
        
        // Adapted (Stolen) from RobotLib :)
        float updatePID(float target_state, float cur_state) {            
            // Declare local variables
            float P, I, D;
            float result;
            float slope;
            float dt;
            
            // Get the time step
            dt = 1.0f/MOTOR_UPDATE_RATE;
            
            // Calculate error
            this->error = target_state - cur_state;
            
            // Integrate error using trapezoidal Riemann sums
            this->prev_error = target_state - this->last_state;
            this->integrator += 0.5f * (this->error + this->prev_error) * dt;
            
            // Find the slope of the error curve using secant approximation
            slope = (cur_state - this->last_state) / dt;
            
            // Apply PID gains
            P = this->kP * this->error;
            I = this->kI * this->integrator;
            D = this->kD * slope;
            
            // Sum P, I, D to get the result of the equation
            // Bind the output if needed
            result = clamp(P + I + D, -INCREMENT_AMT, INCREMENT_AMT);
            
            // Update timing and increment to the next state
            this->last_state = cur_state;
            
            // Return the PID result
            return result;
        }
        
        static float clamp(float val, float min, float max)
        {
            if(val > max)
            {
                return max;
            }
            else if(val < min)
            {
                return min;
            }
            return val;
        }
};

#endif
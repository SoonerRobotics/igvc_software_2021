/*
 * IGVCMotor.h
 *
 *  Created on: Aug 8, 2020
 *      Author: tyler
 */

#ifndef IGVCMOTORH
#define IGVCMOTORH

// Stolen from igvc 2020 Software drivetrain

#define MOTOR_UPDATE_RATE 80 // Frequency that motor PID is updated (Hz)
#define MAX_SPEED 2.2f // (m/s)
#define PULSES_PER_REV 2400 // (revs)
#define LINEAR_PER_REV 0.254f // Wheel radius (m)
#define MILLIS_TO_FULL 90 // Milliseconds to go from 0 output speed to 1
#define LPIIR_DECAY 0.1f // Decay rate of low pass filter on velocity

#define PI 3.14159265f
#define INCREMENT_AMT (1000.0f / (MILLIS_TO_FULL * MOTOR_UPDATE_RATE))

#define PREV_MASK 0x1 // Mask for the previous state in determining direction of rotation.
#define CURR_MASK 0x2 // Mask for the current state in determining direction of rotation.
#define INVALID   0x3 // XORing two states where both bits have changed.


extern TIM_HandleTypeDef htim3;
/*
 * We use channel A for all of the enable pins
 * We use Timer 3 for all of the PWM pins
 *
 * @brief Constructor for IGVCMotor
 * @param GPIO_Pin_A: The gpio Pin number to control the first enable pin
 * @param GPIO_Pin_B: The gpio Pin number to control the second enable pin
 * @param Channel: The channel of the PWM signal that you want to set
 * @retval None
 * */
class IGVCMotor {
	public:
		IGVCMotor(uint16_t GPIO_Pin_A, uint16_t GPIO_Pin_B, uint32_t channel){
			this->motor_A = GPIO_Pin_A;
			this->motor_B = GPIO_Pin_B;
			this->pwm_channel = channel;

			this->pulses = 0;
			this->target_speed = 0.0f;
			this->cur_output = 0.0f;
			this->speed_estimate = 0.0f;
			// PID
			this->integrator = 0.0f;
			this->error = 0.0f;
			//States
			this->prev_error = 0.0f;
			this->last_state = 0.0f;
			//equation constants
			this->kP = 0.05f;
			this->kI = 0.0f;
			this->kD = 0.0001f;
		}

		void output(float speed){
			this->target_speed = speed;
		}

		// Stolen from QEI library
		void pulse(int left, int right) {
			curr_state_ = (left<<1) | (right);
			//11->00->11->00 is counter clockwise rotation or "forward".
			if ((prev_state_ == 0x3 && curr_state_ == 0x0) ||
					(prev_state_ == 0x0 && curr_state_ == 0x3)) {
				this->pulses++;
			}
			//10->01->10->01 is clockwise rotation or "backward".
			else if ((prev_state_ == 0x2 && curr_state_ == 0x1) ||
					 (prev_state_ == 0x1 && curr_state_ == 0x2)) {
				this->pulses--;
			}

			prev_state_ = curr_state_;
		}
		void update(){

			float instantaneous_speed = this->pulses / (float)PULSES_PER_REV * 2.0 * PI * LINEAR_PER_REV * MOTOR_UPDATE_RATE;
			this->speed_estimate += (1.0f - LPIIR_DECAY) * (instantaneous_speed - this->speed_estimate); // low pass filter

			float output = this->updatePID(this->target_speed, this->speed_estimate);

			cur_output += output;
			cur_output = clamp(cur_output, -1.0,1.0);

			this->pulses = 0;

			if(cur_output < 0.01f && cur_output > -0.01f){ // BrEAK
				HAL_GPIO_WritePin(GPIOA, motor_A, GPIO_PIN_RESET); // A = 0
				HAL_GPIO_WritePin(GPIOA, motor_B, GPIO_PIN_RESET); // B = 0
				setPWM(pwm_channel,this->period, 0);
			}
			else if(cur_output > 0.0f){ // forward
				HAL_GPIO_WritePin(GPIOA, motor_A, GPIO_PIN_SET); // A = 0
				HAL_GPIO_WritePin(GPIOA, motor_B, GPIO_PIN_RESET); // B = 0
				setPWM(pwm_channel,this->period, .05f + cur_output*.95f);
			}
			else{
				HAL_GPIO_WritePin(GPIOA, motor_A, GPIO_PIN_RESET); // A = 0
				HAL_GPIO_WritePin(GPIOA, motor_B, GPIO_PIN_SET); // B = 1
				setPWM(pwm_channel,this->period, .05f + (-cur_output) * .95f );
			}





		}
		float getSpeedEstimate(){
			return this->speed_estimate;
		}
		IGVCMotor& operator= (float v){
			output(v);
			return *this;
		}


	private:
		uint16_t motor_A;
		uint16_t motor_B;
		uint32_t pwm_channel;
		uint16_t period = 15000; // 15 khz


		int prev_state_;
		int curr_state_;
		volatile int pulses;

		float cur_output;
		float target_speed;

		float speed_estimate;

		//PID values
		float integrator;
		float error;
		//states
		float prev_error;
		float last_state;
		//equation co effiecients
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
		// stolen from robotlib / IGVCMotor 2020
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

		void setPWM(uint32_t channel, uint16_t period, float output){
			TIM_HandleTypeDef timer = htim3;
			uint16_t pulse = uint16_t(output * period);

			HAL_TIM_PWM_Stop(&timer, channel); // Stop the PWM temporarily
			TIM_OC_InitTypeDef sConfigOC;
			timer.Init.Period = period;			// set period duration
			HAL_TIM_PWM_Init(&timer);			// reinit new period value

			sConfigOC.OCMode = TIM_OCMODE_PWM1;
			sConfigOC.Pulse = pulse; //set pulse duration
			sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
			sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
			HAL_TIM_PWM_ConfigChannel(&timer, &sConfigOC, channel);
			HAL_TIM_PWM_Start(&timer, channel);

		}
};

#endif /* SRC_IGVCMOTOR_H_ */

#include <STM32F103XB.h>

void Set_up();
void delay100ms();
void PWM_Config1();
void PWM_Config2();
void delay_us(int us);
int UltraSonic_GetDistance(int sensorId);
void DcMotor_Control(int motorId, int direction, int speed);
float kp = 0.5;
float kd = 0.1;
int speed = 35;

int main() {
	Set_up();

	while (1) {

		if ((GPIOB->IDR & (1 << 4)) == 0) {
			int right_reference = UltraSonic_GetDistance(1);
			int left_reference = UltraSonic_GetDistance(2);
			int prev_right_error = 0;
			int prev_left_error = 0;

			while (1) {
				int right_error = right_reference - UltraSonic_GetDistance(1);
				int left_error = left_reference - UltraSonic_GetDistance(2);

				int right_speed = speed
						+ ((kp * right_error)
								+ (kd * (prev_right_error - right_error)));
				int left_speed = speed
						- ((kp * right_error)
								+ (kd * (prev_right_error - right_error)));

				prev_right_error = right_error;
				prev_left_error = left_error;

				DcMotor_Control(1, 1, right_speed);
				DcMotor_Control(2, 1, left_speed);

				delay100ms();
			}

		}
	}
}

void Set_up() {
	// === Enable Clocks ===
	RCC->APB2ENR |= (1 << 0);    // AFIO
	RCC->APB2ENR |= (1 << 2);    // GPIOA
	RCC->APB2ENR |= (1 << 3);    // GPIOB
	RCC->APB2ENR |= (1 << 9);    // ADC1
	RCC->APB2ENR |= (1 << 11);	 // Timer1 (Delay)
	RCC->APB1ENR |= (1 << 0);    // Timer2 (PWM motor 1)
	RCC->APB1ENR |= (1 << 1);    // Timer3 (PWM motor 2)

	// === Disable AFIO Remap for TIM3 CH1 on PA6 ===
	AFIO->MAPR &= ~(1 << 8);     // No remap

	// === GPIO Configuration ===
	GPIOA->CRL = 0x4B44444B;		//PA0 output PWM motor1
									//PA6 output PWM motor2
	GPIOA->CRH = 0x44443344;//PA10-PA11 output push-pull for the dir. of motor1
	GPIOB->CRL = 0x44484444;			//PB4 input pull up ( for start button)
	GPIOB->CRH = 0x34343344;//PB10-PB11 output push-pull for the dir. of motor2
							//PB12 input floating ( ultrasonic1 echo )
							//PB13 output push-pull ( ultrasonic1 trig )
							//PB14 input floating ( ultrasonic2 echo )
							//PB15 output push-pull ( ultrasonic2 trig )
	GPIOB->ODR |= (1 << 4);

	PWM_Config1();
	PWM_Config2();

}

void DcMotor_Control(int motorId, int direction, int speed) {
	if (speed > 100 || speed < 0) {
		return;
	}

	if (motorId == 1) {							// left motor
		if (direction == 1) {					//clockwise
			GPIOA->ODR |= (1 << 10);			//IN1
			GPIOA->ODR &= ~(1 << 11);			//IN2
		} else {								//anti-clockwise
			GPIOA->ODR &= ~(1 << 10);
			GPIOA->ODR |= (1 << 11);
		}
		TIM2->CCR1 = speed * 40;			//Duty Cycle

	} else {
		if (direction == 1) {					//clockwise
			GPIOB->ODR |= (1 << 10);
			GPIOB->ODR &= ~(1 << 11);
		} else {								//anti-clockwise
			GPIOB->ODR &= ~(1 << 10);			//IN1
			GPIOB->ODR |= (1 << 11);			//IN2
		}
		TIM3->CCR1 = speed * 40;			//Duty Cycle

	}
}

void delay100ms() {       //100ms
	TIM1->CNT = 0;
	TIM1->PSC = 9999;
	TIM1->ARR = 79;
	TIM1->SR = 0;
	TIM1->CR1 = 1;
	while ((TIM1->SR & 1) == 0)
		;
	TIM1->CR1 = 0;
}

void delay_us(int us) {
	TIM1->CNT = 0;
	TIM1->PSC = 8 - 1;
	TIM1->ARR = us - 1;
	TIM1->SR = 0;
	TIM1->CR1 = 1;
	while ((TIM1->SR & 1) == 0)
		;
	TIM1->CR1 = 0;
}

void PWM_Config1() {
	TIM2->CCER |= (1 << 0); 				// Enable CH1 output (PA0)
	TIM2->CCMR1 |= (0b110 << 4); 			// PWM Mode 1
	TIM2->CCMR1 |= (1 << 3); 				// Preload enable
	TIM2->CR1 |= (1 << 7); 					// ARPE enable
	TIM2->CR1 |= (0b10 << 5); 				// Center-aligned mode 2
	TIM2->ARR = 4000; 						// Auto-reload (period)
	TIM2->CCR1 = 0; 						// Start at 0% duty
	TIM2->EGR |= (1 << 0); 					// Update registers
	TIM2->CR1 |= (1 << 0); 					// Start TIM3
}

void PWM_Config2() {
	TIM3->CCER |= (1 << 0); 				// Enable CH1 output (PA6)
	TIM3->CCMR1 |= (0b110 << 4); 			// PWM Mode 1
	TIM3->CCMR1 |= (1 << 3); 				// Preload enable
	TIM3->CR1 |= (1 << 7); 					// ARPE enable
	TIM3->CR1 |= (0b10 << 5); 				// Center-aligned mode 2
	TIM3->ARR = 4000; 						// Auto-reload (period)
	TIM3->CCR1 = 0; 						// Start at 0% duty
	TIM3->EGR |= (1 << 0); 					// Update registers
	TIM3->CR1 |= (1 << 0); 					// Start TIM3
}

//output distance in CM
int UltraSonic_GetDistance(int sensorId) {
	int trigPin, echoPin;

	if (sensorId == 1) {
		trigPin = 13; // PB13
		echoPin = 12; // PB12
	} else {
		trigPin = 15; // PB15
		echoPin = 14; // PB14
	}

	// Send trigger pulse
	GPIOB->ODR |= (1 << trigPin);
	delay_us(10);
	GPIOB->ODR &= ~(1 << trigPin);

	// Prepare timer
	TIM1->PSC = 8 - 1;   // 1 MHz -> 1us resolution
	TIM1->ARR = 65535;
	TIM1->CNT = 0;

	// Wait for echo to go high with timeout
	int timeout = 30000; // 30 ms max wait
	while (!(GPIOB->IDR & (1 << echoPin)) && --timeout)
		;
	if (timeout == 0)
		return -1; // Timeout error

	TIM1->CR1 |= (1 << 0); // Start timer

	timeout = 30000; // Reset timeout
	while ((GPIOB->IDR & (1 << echoPin)) && --timeout)
		;
	TIM1->CR1 = 0;

	if (timeout == 0)
		return -1; // Timeout error

	int duration_us = TIM1->CNT;
	int distance_cm = duration_us / 58;

	return distance_cm;
}


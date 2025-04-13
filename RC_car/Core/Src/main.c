#include <STM32F103XB.h>

void Set_up();
void delay100();
void PWM_Config1();
void PWM_Config2();
void DcMotor_Control(int motorId, int direction, int speed);

int main() {

	Set_up();

	while (1) {

		DcMotor_Control(2, 1, 50);
		for (int i = 0; i <= 50; i++) {
			delay100();
		}
		DcMotor_Control(2, 1, 100);
		for (int i = 0; i <= 50; i++) {
			delay100();
		}
		DcMotor_Control(2, 1, 0);
		for (int i = 0; i <= 50; i++) {
			delay100();
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
	GPIOA->CRL = 0x4B44444B;		//PA0 output motor1
									//PA6 output motor2
	GPIOA->CRH = 0x44443344;//PA10-PA11 output push-pull for the dir. of motor1
	GPIOB->CRL = 0x44444444;
	GPIOB->CRH = 0x44443344;//PB10-PB11 output push-pull for the dir. of motor2

	PWM_Config1();
	PWM_Config2();

}

void DcMotor_Control(int motorId, int direction, int speed) {

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

void delay100() {       //100ms
	TIM1->PSC = 9999;
	TIM1->ARR = 79;
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

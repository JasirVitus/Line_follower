#include "stm32f0xx.h"

//ADC_TRG
#define TIM1_PERIOD 1000
#define TIM1_PRESCALE 4800

//PWM
#define TIM3_PERIOD 10000
#define TIM3_PRESCALE 48

GPIO_Config(void);
NVIC_Config(void);
DMA_Config(void);
TIM3_Config(void);
TIM1_Config(void);
ADC_Config(void);
USART_Config(void);
RS_Send(uint8_t ch);

uint8_t ADC_array[5] ={0};
uint16_t i;

uint16_t TIM3_CCR1_PERIOD = 0;
uint16_t TIM3_CCR3_PERIOD = 0;

enum LF_state
{
	OUT,
    STARTING,
    FIRST,
    SECOND,
    THIRD,
    FOURTH,
    FIFTH
};

LF_state Smash_kart = OUT;

int main(void)
{
	SystemInit();

	GPIO_Config();
	NVIC_Config();
	DMA_Config();
	TIM3_Config();
	TIM1_Config();
	ADC_Config();
	USART_Config();

	ADC1->CR |= ADC_CR_ADSTART; // start conversion

    while(1)
    {
    }
}

GPIO_Config(void)
{
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

	// PC8, PC9
	GPIOC->MODER   |= GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0;       // Output mode
	GPIOC->OTYPER  &= ~(GPIO_OTYPER_OT_8 | GPIO_OTYPER_OT_9) ;         // Push-Pull
	GPIOC->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR8 | GPIO_OSPEEDER_OSPEEDR9; // Speed
	GPIOC->PUPDR   |= GPIO_PUPDR_PUPDR8_1 | GPIO_PUPDR_PUPDR9_1;       // Pull-Up

	// PA0 (ADC_IN0)
	GPIOA->MODER |=GPIO_MODER_MODER0_0 | GPIO_MODER_MODER0_1; 	  // Analog mode
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR0_1 | GPIO_PUPDR_PUPDR0_0); // 00
	// PA1 (ADC_IN1)
	GPIOA->MODER |=GPIO_MODER_MODER1_0 | GPIO_MODER_MODER1_1; 	  // Analog mode
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR1_1 | GPIO_PUPDR_PUPDR1_0); // 00
	// PA2 (ADC_IN2)
	GPIOA->MODER |=GPIO_MODER_MODER2_0 | GPIO_MODER_MODER2_1; 	  // Analog mode
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR2_1 | GPIO_PUPDR_PUPDR2_0); // 00
	// PA3 (ADC_IN3)
	GPIOA->MODER |=GPIO_MODER_MODER3_0 | GPIO_MODER_MODER3_1; 	  // Analog mode
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR3_1 | GPIO_PUPDR_PUPDR3_0); // 00
	// PA4 (ADC_IN4)
	GPIOA->MODER |=GPIO_MODER_MODER4_0 | GPIO_MODER_MODER4_1; 	  // Analog mode
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR4_1 | GPIO_PUPDR_PUPDR4_0); // 00

	//PA8 (TIM3_CH1)
	GPIOA->MODER  &= ~GPIO_MODER_MODER8; // reset mode
	GPIOA->MODER  |= GPIO_MODER_MODER8_1 ; // Alternate function
	GPIOA->AFR[1] |= 0x02 ; // AF2 << 4*(8-8)
	//PA7 (GND)
	GPIOA->MODER   |= GPIO_MODER_MODER7_0 ;   // Output mode
	GPIOA->OTYPER  &= ~GPIO_OTYPER_OT_7  ;    // Push-Pull
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR7; // Speed
	GPIOA->PUPDR   |= GPIO_PUPDR_PUPDR7_1;

	GPIOA->ODR &= ~GPIO_ODR_7;
	//PA10 (TIM3_CH3)
	GPIOA->MODER  &= ~GPIO_MODER_MODER10; // reset mode
	GPIOA->MODER  |= GPIO_MODER_MODER10_1 ; // Alternate function
	GPIOA->AFR[1] |= 0x02 << 0x08 ; // AF2 << 4*(10-8)
	//PB1 (GND)
	GPIOB->MODER   |= GPIO_MODER_MODER1_0 ;   // Output mode
	GPIOB->OTYPER  &= ~GPIO_OTYPER_OT_1  ;    // Push-Pull
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR1; // Speed
	GPIOB->PUPDR   |= GPIO_PUPDR_PUPDR1_1;

	GPIOB->ODR &= ~GPIO_ODR_1;
}

NVIC_Config(void)
{
	NVIC->ISER[0] = (uint32_t)0x01 << (DMA1_Channel1_IRQn & (uint8_t)0x1F);

	NVIC->IP[DMA1_Channel1_IRQn >> 0x02] &= (uint32_t)(~(((uint32_t)0xFF) << ((DMA1_Channel1_IRQn & 0x03) * 8)));
	NVIC->IP[DMA1_Channel1_IRQn >> 0x02] |= (uint32_t)((((uint32_t)0x00 << 6) & 0xFF) << ((DMA1_Channel1_IRQn & 0x03) * 8));
	// priority 0x00


}

DMA_Config(void)
{
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;

	DMA1_Channel1->CPAR = (uint32_t) (&(ADC1->DR)); // Peripheral Adress
	DMA1_Channel1->CMAR = (uint32_t)(ADC_array);    // Memory Adress
	DMA1_Channel1->CNDTR = 5;                      // Number of data to Transfer

	DMA1_Channel1->CCR |= DMA_CCR_MINC | DMA_CCR_TEIE | DMA_CCR_TCIE | DMA_CCR_CIRC ;
	// Memory Increment | Transfer error Interrupt Enable | Circular mode | Transfer Complete Interrupt Enable
	DMA1_Channel1->CCR |= DMA_CCR_EN; // DMA ENABLE
}

TIM3_Config(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; //ENABLE TIM1 Clock

	TIM3->PSC = TIM3_PRESCALE-1;
	TIM3->ARR = TIM3_PERIOD;
	TIM3->CCR1 = TIM3_CCR1_PERIOD; // CH1
	TIM3->CCR3 = TIM3_CCR3_PERIOD; // CH3

	TIM3->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1| TIM_CCMR1_OC1PE; // Set PWM ("110") and enable preload register
	TIM3->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1| TIM_CCMR2_OC3PE; //

	TIM3->CCER |= TIM_CCER_CC1E | TIM_CCER_CC3E; // Enable output


	TIM3->CR1 |= TIM_CR1_CEN;   // Enable counter
	TIM3->EGR |= TIM_EGR_UG;    // Force update generation (CLEARS COUNTER)
}

TIM1_Config(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

	TIM1->PSC  = TIM1_PRESCALE -1;
	TIM1->ARR  = TIM1_PERIOD;

	TIM1->CR2 |= TIM_CR2_MMS_1; // TRGO

	TIM1->CR1 |= TIM_CR1_CEN;   // Enable Timer
}

ADC_Config(void)
{
	RCC->APB2RSTR |= RCC_APB2RSTR_ADC1RST;  // Reset ADC
	RCC->APB2RSTR &= ~RCC_APB2RSTR_ADC1RST; // Release from reset

	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;    // ENABLE ADC Clock

    ADC1->CFGR1 = 0x00;                  // Reset ADC Configuration
    ADC1->CFGR1 |= ADC_CFGR1_EXTEN_0;    // TRG Rising Edge | (TRG0 - TIM1) | (Single mode)
	ADC1->CFGR1 |= ADC_CFGR1_DMAEN | ADC_CFGR1_DMACFG | ADC_CFGR1_RES_1 ; //  enable DMA | 8 bit RES
	ADC1->CHSELR = ADC_CHSELR_CHSEL0|ADC_CHSELR_CHSEL1|ADC_CHSELR_CHSEL2|ADC_CHSELR_CHSEL3|ADC_CHSELR_CHSEL4 ;

	ADC1->SMPR |= 0x00;             // Sampling time 239.5
	ADC1->IER  |= ADC_IER_EOSIE;    // Interrupt Enable EOS

	ADC1->CR |= ADC_CR_ADEN; // Enable ADC
}

USART_Config(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	RCC->AHBENR  |= RCC_AHBENR_GPIOAEN;

	//PA9
	GPIOA->MODER  |= GPIO_MODER_MODER9_1;   // ALTERNATE Function
	GPIOA->AFR[1] |= 0x01 << 0x04;   		// USART1_TX
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR9_0; 	// Pull-Up

	USART1->CR1  = 0x00; // Reset
	USART1->CR1 |= USART_CR1_M | USART_CR1_TE | USART_CR1_PCE; // (Word Length 9)| EVEN Parity | (Hardware flow control none) | Transmitter enable
	//USART1->CR2 |=  ;          // (Stop bits: 1)
	USART1->BRR = 480000 / 96;   // 9600 bauds

	USART1->CR1 |= USART_CR1_UE ; // ENABLE USART
}

RS_Send(uint8_t ch)
{
	USART1->TDR = (ch & (uint16_t)0x01FF);
	while( (USART1->ISR & USART_ISR_TXE) == 0){}
}

DMA1_Channel1_IRQHandler()
{
	if((DMA1->ISR & DMA_ISR_TCIF1) != 0) // If Transfer Complete Flag = 1
		{
			Smash_kart = OUT;

			if(ADC_array[0]>100)
				Smash_kart = FIRST;
			if(ADC_array[1]>100)
				Smash_kart = SECOND;
			if(ADC_array[3]>100)
				Smash_kart = FOURTH;
			if(ADC_array[4]>100)
				Smash_kart = FIFTH;
			if(ADC_array[2]>100)
				Smash_kart = THIRD;

	        if((TIM3_CCR1_PERIOD < 6000)&&(TIM3_CCR3_PERIOD < 6000))
			if(Smash_kart != OUT)
				Smash_kart = STARTING;

			switch (Smash_kart){
			case STARTING:
				if(TIM3_CCR1_PERIOD < 6000)
				 TIM3_CCR1_PERIOD += 1000;
				if(TIM3_CCR3_PERIOD < 6000)
				 TIM3_CCR3_PERIOD += 1000;
			break;
			case OUT:
				if(TIM3_CCR1_PERIOD != 0)
				 TIM3_CCR1_PERIOD -= 1000;
				if(TIM3_CCR3_PERIOD != 0)
				 TIM3_CCR3_PERIOD -= 1000;
			break;
			case FIRST:
				 TIM3_CCR1_PERIOD = 6000;
				 TIM3_CCR3_PERIOD = 3000;
			break;
			case SECOND:
				 TIM3_CCR1_PERIOD = 6000;
				 TIM3_CCR3_PERIOD = 4500;
			break;
			case THIRD:
				 TIM3_CCR1_PERIOD = 6000;
				 TIM3_CCR3_PERIOD = 6000;
			break;
			case FOURTH:
				 TIM3_CCR1_PERIOD = 4500;
				 TIM3_CCR3_PERIOD = 6000;
			break;
			case FIFTH:
				 TIM3_CCR1_PERIOD = 3000;
				 TIM3_CCR3_PERIOD = 6000;
			break;
			}


			DMA1->IFCR |= DMA_IFCR_CTCIF1; // CLEAR Transfer Complete Flag
		}
}


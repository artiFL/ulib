#include "ulib.h"


void UASRT_init (u8 STAT, u32 boudrate) {

	/* USART1 init
 * 8 bit data
 * 1 stop bit
 * none parity control
 * none flow control
 * B15 RX
 * B14 TX
 */

	clock > 16000000 ? (RCC->DCKCFGR2   |= RCC_DCKCFGR2_USART1SEL_0) : (RCC->DCKCFGR2   &= ~RCC_DCKCFGR2_USART1SEL_0);

	u32 U_BRR = clock + (boudrate >> 1);
	U_BRR /= boudrate;

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIODEN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    //b15 rx b 14 tx
    GPIOB->MODER |= GPIO_MODER_MODER14_1 | GPIO_MODER_MODER15_1;
    GPIOB->AFR[1]|=(0x04<<(4*6));
    GPIOB->AFR[1]|=(0x04<<(4*7));

    USART1->BRR = U_BRR;
    USART1->CR1 |= USART_CR1_TE;
    USART1->CR1 |= USART_CR1_RE;
    USART1->CR1 |= USART_CR1_RXNEIE;

    STAT == ENABLE ? (USART1->CR1 |= USART_CR1_UE) : (USART1->CR1 &= ~USART_CR1_UE);                        //USART ENABLE

    NVIC_EnableIRQ(USART1_IRQn);

}

void USART1_IRQHandler(void) {


	if ((USART1 -> RDR) == SIGMATURE_START_RECIVE_ARRAY_STRING) {


		get_array_str(str);

		}

	else if ((USART1 -> RDR) == SIGNATURE_START_RECIVE_ARRAY_BYTE) {


		get_array_byte(bufferRessive);

	}

	else {

		get_byte(bufferRessive);

		USART1 -> RQR |= USART_RQR_RXFRQ;
		USART1 -> ICR |= USART_ICR_ORECF;

	}

	NVIC_ClearPendingIRQ(USART1_IRQn);

}

///////////////////////////////////////////////////////////////////////
void send_byte (u8 data) {

    while(!(USART1->ISR & USART_ISR_TC)){}
    	USART1->TDR = data;

}

void send_array (volatile u8* array) {

	u16 counte = 0;

	while (array[counte]) {

		send_byte(array[counte++]);

	}

}

void send_array_byte (u8* array, u16 Length) {

	for (u16 p = 0; p < Length; p ++) {

		send_byte(array[p]);

	}

}

void print_str (volatile char* str) {

	u16 counte = 0;

	while (str[counte]) {

		send_byte(str[counte++]);

	}

}

void get_byte(volatile u8* array) {

    array[addres++] = USART1 -> RDR;

    USART1 -> RQR |= USART_RQR_RXFRQ;

   flag_get_byte = 1;

}

void get_array_byte (volatile u8* array) {

	u8 tmpChar = 0;
	u16 q = 0;

	do {

		if ((USART1->ISR & USART_ISR_RXNE) != 0) {

			tmpChar = USART1 -> RDR;
			array[q++] = tmpChar;

			USART1 -> RQR |= USART_RQR_RXFRQ;
			USART1 -> ICR |= USART_ICR_ORECF;

		}
	}

	while (tmpChar != SIGNATURE_END_RECIVE_ARRAY_BYTE);

	array[--q] = 0;
	flag_get_array_byte = 1;
	//send_array(array);


}

void get_array_str (volatile char* array) {

	char tmpChar = 0;
	u16 q = 0;

	do {

		if ((USART1->ISR & USART_ISR_RXNE) != 0) {

			tmpChar = USART1 -> RDR;
			array[q++] = tmpChar;

			USART1 -> RQR |= USART_RQR_RXFRQ;
			USART1 -> ICR |= USART_ICR_ORECF;

		}
	}

	while (tmpChar != SIGNATURE_END_RECIVE_ARRAY_STRING);

	array[--q] = 0;
	flag_get_array_string = 1;
	print_str(array);

}

///////////////////////////////////////////////////////////////////////
void Connect_to_PC (void) {

	do {

		send_byte(SIGNATURE_TO_CONNECTION);

		delay(120000);

	}

	while(bufferRessive[addres] != SIGNATURE_END_SEARCH);

		for(u8 r; r < 5; r++ ) {

			send_byte(SIGNATURE_CONFIRM_CONNECTION);

		}

}

void indication_led_init (u8 STAT, u8 LED) {

	/*
 standart macro function
 * *indicationLedInit enable B7 Blue LED
 * 							 B14 Red LED (don't use as USART1)
 */

	if (STAT == ENABLE && LED == BLUE) {

		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;                    //clock GPIOB enable

		GPIOB->MODER   |= GPIO_MODER_MODER7_0;                  //output push pull
		GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR7_1;             //max speed

		flag_led_init = 1;

	}
	else if (STAT == ENABLE && LED == RED) {

		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;                     //clock GPIOB enable

		GPIOB->MODER   |= GPIO_MODER_MODER14_0;                  //output push pull
		GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR14_1;             //max speed

		flag_led_init = 1;

	}
	else if (STAT == DISABLE) {

		RCC -> AHB1ENR &= ~RCC_AHB1ENR_GPIOBEN;

		flag_led_init = 0;

	}
}

void blink (u32 duty, u8 STAT, u8 LED) {

	if (flag_led_init != 0) {

		if (STAT == ON && duty == 0) {

			if (LED == BLUE) {

				GPIOB -> BSRR |= GPIO_BSRR_BS_7;

			}
			else if (LED == RED) {

				GPIOB -> BSRR |= GPIO_BSRR_BS_14;

			}

		}
		else if (STAT == OFF && duty == 0) {

			if (LED == BLUE) {

				GPIOB -> BSRR |= GPIO_BSRR_BR_7;

			}
			else if (LED == RED) {

				GPIOB -> BSRR |= GPIO_BSRR_BR_14;

			}
		}
		else if (STAT == ON && duty != 0) {

			//print("ONOFF == ENABLE && duty != 0");

		}
		else if (STAT == OFF && duty != 0) {

			if (LED == BLUE) {

				GPIOB -> ODR ^= GPIO_ODR_ODR_7;
				delay(duty);
				GPIOB -> ODR ^= GPIO_ODR_ODR_7;
				delay(duty);

			}
			else if (LED == RED) {

				GPIOB -> ODR ^= GPIO_ODR_ODR_14;
				delay(duty);
				GPIOB -> ODR ^= GPIO_ODR_ODR_14;
				delay(duty);

			}
		}

	else if (flag_led_init != 1) {

		print_str("_ERROR [flag != 1] \n");

	}
	}
}

void delay (u32 delay) {


	for (; delay != 0; delay--) {

		__NOP();
	}
}

void HSE_Enable(void){


	//configur. flesh prescaller
	FLASH->ACR |= FLASH_ACR_PRFTEN;
	FLASH->ACR &= (uint32_t)((uint32_t)~FLASH_ACR_LATENCY);
	FLASH->ACR |= (uint32_t)FLASH_ACR_LATENCY_7WS;

	//RCC->CR |= RCC_CR_CSSON;    // Clock security sistem

	RCC->CR = 0;
	RCC->CFGR = 0;
	//baypass enable
	RCC->CR |=RCC_CR_HSEBYP;
	//Enable HSE
	RCC->CR |= RCC_CR_HSEON;
	//Wait till HSE is ready
	    while(!(RCC->CR & RCC_CR_HSERDY));

	//PLL configuration   PLLM = 4     PLLN = 216      PLLP = 2   0b11011000000100
	RCC->PLLCFGR = 0;
	RCC->PLLCFGR =  0x3604;

	  //PLL surse
	RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC;

	// HCLK = SYSCLK
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
	// PCLK2 = HCLK
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;
	// PCLK1 = HCLK
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;

	// Turn on the PLL
	RCC->CR |= RCC_CR_PLLON;
	// Wait till PLL is ready
	    while((RCC->CR & RCC_CR_PLLRDY) == 0);

	RCC->CFGR &= ~RCC_CFGR_SW;
	RCC->CFGR |= RCC_CFGR_SW_PLL;
	    while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

	clock = 216000000;

}

void RNG_Init (void) {

	RCC -> AHB2ENR |= RCC_AHB2ENR_RNGEN;

	RNG -> CR |= RNG_CR_RNGEN;

}

void number_gen (u8* array, u16 Lengt) {

	RNG_Init();

	for (u16 count = 0; count < Lengt; count++) {

		while ((RNG -> SR & RNG_SR_DRDY) != 1);
		array[count] = RNG -> DR;

	}



}

void PWM_Init (u8 STAT, float freq, float duty) {

	/*PWM Chanel
 * TIM3  CH2 A7
 *
 */

	// Тактирование  GPIOA , TIM1, альтернативных функций порта
	RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	GPIOA -> MODER |= GPIO_MODER_MODER7_1;
	GPIOA -> OSPEEDR |= GPIO_OSPEEDER_OSPEEDR7;
	GPIOA -> AFR[0] |= (0x2<<4*7);

	RCC -> APB1ENR |= RCC_APB1ENR_TIM3EN;
	//prescaler
	PWM_Put(freq, duty);
	//2 chanel output, active level HIGH
	TIM3 -> CCER |= TIM_CCER_CC2E;				// | TIM_CCER_CC2P;
	//configuration output timer
	TIM3 -> BDTR |= TIM_BDTR_MOE;
	//PWM mode 1, force PWM 2 chanel
    TIM3 -> CCMR1 = TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;
	//step up
	TIM3 -> CR1 &= ~TIM_CR1_DIR;
	//front alignment, Fast PWM
	TIM3 -> CR1 &= ~TIM_CR1_CMS;
	//count enable
    STAT == ENABLE ? (TIM3 -> CR1 |= TIM_CR1_CEN) : (TIM3 -> CR1 &= ~TIM_CR1_CEN);

}

void PWM_Put(float freq, float duty) {

	clock_TIM = (clock / T_PSC) - 1;
	T_ARR = ((1.0 / freq) * clock_TIM);
	duty = (T_ARR / (duty * 0.04));

	TIM3 -> PSC = T_PSC;
    TIM3 -> ARR = T_ARR - 1;
	TIM3 -> CCR2 = duty;
}

void interrupt_timer (u8 STAT, u32 time_count) {

	RCC -> APB2ENR |= RCC_APB2ENR_TIM1EN;

	TIM1 -> PSC = T_PSC;
	TIM1 -> ARR = time_count;
	TIM1 -> CR1 |= TIM_CR1_CEN;
	TIM1 -> DIER |= TIM_DIER_UIE;

	STAT == ENABLE ? (TIM1 -> CR1 |= TIM_CR1_CEN) : (TIM1 -> CR1 &= ~TIM_CR1_CEN);

	NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);

}

void TIM1_UP_TIM10_IRQHandler (void) {

	TIM1->SR &= ~TIM_SR_UIF;

}

void SPI_Init (void) {

	/*
  * голубой     SDA     PC12
  * оранжевый	SCL		PC10
  * зеленый		RS		PC8
  * фиолетовый	RESET	PC11
  * черный		GND * 2
  * коричневый	CS		PC9
  *
  * 1			LEDK
  * 2			LEDA
  * 3			VDD
  *
*/

	RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

	//MOSI
	GPIOC -> MODER |= GPIO_MODER_MODER12_1;
	GPIOC -> OSPEEDR  |= GPIO_OSPEEDER_OSPEEDR12;
	GPIOC -> AFR[1]	|= (0x06<<(4*4));
	//SCL
	GPIOC -> MODER |= GPIO_MODER_MODER10_1;
	GPIOC -> OSPEEDR  |= GPIO_OSPEEDER_OSPEEDR10;
	GPIOC -> AFR[1]	|= (0x06<<(2*4));

	//RS (COMMAND SELECT)
	GPIOC -> MODER |= GPIO_MODER_MODER8_1;
	GPIOC -> OSPEEDR  |= GPIO_OSPEEDER_OSPEEDR8;
	//GPIOC -> BSRR |= GPIO_BSRR_BS_8;
	//RESET
	GPIOC -> MODER |= GPIO_MODER_MODER11_1;
	GPIOC -> OSPEEDR  |= GPIO_OSPEEDER_OSPEEDR11;
	GPIOC -> BSRR |= GPIO_BSRR_BS_11;
	//CS
	GPIOC -> MODER |= GPIO_MODER_MODER9_1;
	GPIOC -> OSPEEDR  |= GPIO_OSPEEDER_OSPEEDR9;
	//GPIOC -> BSRR |= GPIO_BSRR_BS_9;

	RCC -> APB1ENR |= RCC_APB1ENR_SPI3EN;


	SPI3 -> CR1 |= SPI_CR1_BR;              //Baud rate = Fpclk/256
	SPI3 -> CR1 |= SPI_CR1_CPOL;              //Polarity cls signal CPOL = 0;
	SPI3 -> CR1 |= SPI_CR1_CPHA;              //Phase cls signal    CPHA = 0;
	SPI3 -> CR1 &= ~SPI_CR1_LSBFIRST;         //MSB will be first
	SPI3 -> CR1 |= SPI_CR1_SSM;               //Program mode NSS
	SPI3 -> CR1 |= SPI_CR1_SSI;               //анналогично состоянию, когда NSS 1
	SPI3 -> CR1 |= SPI_CR1_MSTR;              //Mode Master
	SPI3 -> CR1 |= SPI_CR1_SPE;               //Enable SPI2

	delay(1600);

/*
	SPI3 -> CR1 |= SPI_CR1_MSTR;
	SPI3 -> CR1 |= SPI_CR1_BIDIMODE;
	SPI3 -> CR1 |= SPI_CR1_BIDIOE;
	SPI3 -> CR1 |= SPI_CR1_SSM;
	SPI3 -> CR1 |= SPI_CR1_SSI;

	SPI3 -> CR1 |= SPI_CR1_BR_2;
				//|  SPI_CR1_BR_0;

	SPI3->CR2 |= SPI_CR2_SSOE;
	SPI3 -> CR1 |= SPI_CR1_SPE;
*/
}

void SPI_write_byte (u16 data) {

	  //while(!(SPI3 -> SR & SPI_SR_TXE));

	  SPI3 -> DR = data;

}

void SPI_send_cmd (u8 data) {

	SPI1 -> CR2 &= ~SPI_CR2_DS_3;

	A0_L();

	SPI_write_byte(data);

}

void SPI_send_data (u8 data) {

	SPI1 -> CR2 &= ~SPI_CR2_DS_3;

	A0_H();

	SPI_write_byte(data);

}

void SPI_send_16byte_data (u16 data) {

	SPI1 -> CR2 |= SPI_CR2_DS;

	A0_H();

	SPI_write_byte(data);

}

void init_disp (void) {

	SPI_send_cmd(0x01);
	delay(160);

	SPI_send_cmd(0x11);
	delay(160);

	SPI_send_cmd(0x3A);

	SPI_send_data(0x05);

	SPI_send_cmd(0x36);
	SPI_send_data(0xA0);

	SPI_send_cmd(0xB1);
	SPI_send_16byte_data(0x000F);
	SPI_send_16byte_data(0x000F);
	SPI_send_16byte_data(0x000F);

	SPI_send_cmd(0x29);

}

/*
AFR[1]
|  15   |   14  |  13   |  12   |
|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|
| | | | | | | | | | | | | | | | |
|   11  |   10  |   9   |   8   |

AFR[0]
|   7   |    6  |   5   |   4   |
|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|_|
| | | | | | | | | | | | | | | | |
|   3   |    2  |   1   |   0   |
*/






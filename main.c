#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_flash.h"
#include "stm32f4xx_adc.h"
#include "misc.h"          // Tu jest NVIC
#include "arm_math.h"
#include <stdlib.h>

#define NL_WIN "\r\n"  	// Znak nowej lini dla Windowsa
#define NL_LINX "\n"	// Znak nowej lini dla Linuxa

//******************************Prototypy funkcji*********************

static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_Init(void);
static void RCC_Configuration(void);
static void MX_ADC_Iinit(void);

void Usart_put_char(USART_TypeDef* USARTx, uint8_t dane);
void Usart_put_string(char* dane);


#define ILOSC_PROBEK 1024 // ilosc probek do zebrania w buforze 2^10 = 1024, 2^11 = 2048, 2^12 = 4096

volatile uint16_t Dzwiek[ILOSC_PROBEK]; // Bufor probek odczytanych
volatile uint16_t licznik = 0;			// Licznik probek odczytanych

float32_t fft_in[ILOSC_PROBEK*2]; 	// Dane wejciowe do FFT
float32_t fft_out[ILOSC_PROBEK];  	// Wyjscie z FFT

float32_t max_freq = 0;				// Znaleziona czestostliwosc


int main(void) {

	char buffor_Itoa[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

	uint32_t i = 0;
	uint32_t j = 0;

	RCC_Configuration();  			// konfig zegara
	MX_GPIO_Init();					//Konfig pinów

	MX_TIM2_Init(); 				// Konfig Timera2
	MX_USART2_Init(); 				// Konfig Usarta2

	MX_ADC_Iinit(); 				// Konfig ADC

	arm_rfft_instance_f32 S;		// Struktura ARM z modulem Complex FFT
	float32_t max_val = 0;             	// Maksymlana wartosc z FFT
	uint32_t max_index = 0;             // Indeks w tablicy wyjsciwej dla ktorej jes maksymalna wartosc FFT


	//arm_rfft_f32(); // Dla STM32F103 z rdzeniem M3

	/* Main clock = 80.5 MHz
	 * F_sampl = 47352           // (80.5*(10^6)/100 )/17
	 * Ilosc probek = 2^11 = 2048
	 * Rozdzielczosc FFT = 47352/2048 = 23.12109375 Hz
	 * */


/***********************************Nieskonczona pêtla*************************************************/
	while (1)
	{
		//GPIO_ToggleBits(GPIOA, GPIO_Pin_5);
		//printf("Hello World!\r\n");

		if(licznik >= ILOSC_PROBEK) // Rozpoczêcie analizy gdy w buforze zgromadzi sie "ILOSC_PROBEK"
		{
			//****************WYSWIETLANIE BUFORA********************
/*			for(i = 0 ; i < 10; i++) {buffor_Itoa[i] = 0;} 		// czyszczeie bufora funkcji Iniger to Asci
			licznik = 0;										//	*
			while(licznik < ILOSC_PROBEK)						//  *
			{													//  *
				itoa(Dzwiek[licznik], buffor_Itoa, 10);			//  *
				Usart_put_string(buffor_Itoa);					//  *
				Usart_put_char(USART2, ',');					//  *
				licznik++;										//  *
			}													//  *
			for(i = 0 ; i < 10; i++) {buffor_Itoa[i] = 0;} 		// czyszczeie bufora funkcji Iniger to Asci
			Usart_put_string(NL_WIN);							//  *
												//  *
			//*******************************************************
*/
			licznik = 0;
			// Dane wejsciowe musza byc w postaci RE IM RE IM RE IM.... gdzie RE nale¿y do [-1, 1]
			for(j = 0; j < ILOSC_PROBEK*2; j += 2)
			{
				fft_in[j] = (float32_t)((float32_t)Dzwiek[licznik] - ((float32_t)2048.0) ) /(float32_t)2048.0; //Czesc rzeczywista
				fft_in[j+1] = 0;			//Czesc urojona
				licznik++;
			}
			licznik = 0;

			//***********************************FFT**********************************************************************************
			arm_cfft_radix4_init_f32(&S, ILOSC_PROBEK, 0, 1); 		// Inicjalizacja modulu fft									*

    		arm_cfft_radix4_f32(&S, fft_in);							// CFFT complex fft											*

    		arm_cmplx_mag_f32(fft_in, fft_out, ILOSC_PROBEK);		 	//Obliczanie modulu											*

    		arm_max_f32(fft_out, ILOSC_PROBEK, &max_val, &max_index); // Odzukuje maksymalna wartoc modulu i jego numer indeksu	*

    		max_freq = max_index *(float32_t)21.6796875 ; 			// Obliczanie czestotliwosci o maksymalnym module w  [Hz]	*

			// FFT pobiera wskaznik do bufora z danymi i pracuje na tych danych - po zakonczeniu FFT dane sa pomieszane!
			//***********************************Koniec FFT***************************************************************************
			// Po tych operacjach mozna przeslac przez USART czestotliwosc o maksymalnym module zmienna "max_freq":

			Usart_put_string(NL_WIN);
			Usart_put_string("Wykryta czestotliwosc:  ");
			itoa((int)max_freq, buffor_Itoa, 10);				// Zaokr¹glenie do Inta!
			Usart_put_string(buffor_Itoa);
			Usart_put_string(" Hz ");
			Usart_put_string(NL_WIN);


			for(i = 0; i < ILOSC_PROBEK; i++){Dzwiek[i] = 0;} // czyszczenie bufora mo¿na pominac
			TIM_Cmd(TIM2, ENABLE); // W³¹czenie timera
			ADC_Cmd(ADC1, ENABLE); // W³aczenie ADC
		}

	}// END_While(1)
}// END_main

/***************************************PRZERWANIA*************************************/

void ADC_IRQHandler()
{

	if(ADC_GetITStatus(ADC1, ADC_IT_EOC) != RESET)
	{
		ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);

		Dzwiek[licznik] = ADC_GetConversionValue(ADC1); 	//	Wpisanuie próbki z ADC do bufora
		licznik++;										 	//	Zwiekszenie ilosci probek w buforze
		if(licznik >= ILOSC_PROBEK)							//	zgromadzono "ILOSC_PROBEK"
		{
		//	TIM_Cmd(TIM2, DISABLE); // Wy³¹czenie timera
		//	ADC_Cmd(ADC1, DISABLE); //Wy³aczenie ADC
		}
		//GPIO_ToggleBits(GPIOA, GPIO_Pin_5); // Odcytana czestotliwosc na oscyloskopie *2
	}
}

void TIM2_IRQHandler()
{
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		GPIO_ToggleBits(GPIOA, GPIO_Pin_5); // Odcytana czestotliwosc na oscyloskopie *2

}

void HardFault_Handler()
{
	while(1); // Je¿eli mikrokontrler otrzyma przerwanie Hard Fault to zostanie przekierowany tutaj
}

/**********************Definicje funkcji***************************************************************/

void MX_GPIO_Init(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_InitTypeDef GPIO_InitDef;

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_MCO);

	GPIO_InitDef.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_8;
	GPIO_InitDef.GPIO_OType = GPIO_OType_PP;
	GPIO_InitDef.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitDef.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitDef.GPIO_Speed = GPIO_Speed_100MHz;
	//Initialize pins
	GPIO_Init(GPIOA, &GPIO_InitDef);

	/*PA0 - ADC1_0 ejscie analogowe*/
	GPIO_StructInit(&GPIO_InitDef); // reset struktury

	GPIO_InitDef.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitDef.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitDef.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitDef.GPIO_Speed = GPIO_Speed_100MHz;
	//Initialize pins
	GPIO_Init(GPIOA, &GPIO_InitDef);

	/*Ogólne piny - PA9, PA5 - LED GRE*/
	GPIO_StructInit(&GPIO_InitDef); // reset struktury
	GPIO_InitDef.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_5;
	GPIO_InitDef.GPIO_OType = GPIO_OType_PP;
	GPIO_InitDef.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitDef.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitDef.GPIO_Speed = GPIO_Speed_100MHz;
	//Initialize pins
	GPIO_Init(GPIOA, &GPIO_InitDef);
}

void MX_TIM2_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	TIM_TimeBaseInitTypeDef TIM_InitDef;
	TIM_OCInitTypeDef TIM_OCInitDef;

	TIM_InitDef.TIM_Prescaler = 499;               // CLK = 80MHz /2
	TIM_InitDef.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_InitDef.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_InitDef.TIM_Period = 120;
	TIM_InitDef.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &TIM_InitDef);

	TIM_OCInitDef.TIM_OCMode = TIM_OCMode_Timing;
	TIM_OCInitDef.TIM_OutputState = TIM_OutputState_Disable;
	TIM_OCInitDef.TIM_Pulse = 0;
	TIM_OCInitDef.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM2, &TIM_OCInitDef);

	//TIM_DMACmd(TIM2, TIM_DMA_Update, ENABLE);
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_Cmd(TIM2, ENABLE);
	TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

	/*Konfiguracja NVIC*/
	NVIC_InitTypeDef NVIC_InitDef;
	NVIC_InitDef.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitDef.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitDef.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitDef.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitDef);
}

void MX_USART2_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	USART_InitTypeDef USART_InitDef;

	USART_InitDef.USART_BaudRate = 38400;
	USART_InitDef.USART_Parity = USART_Parity_No;
	USART_InitDef.USART_Mode = USART_Mode_Tx;
	USART_InitDef.USART_StopBits = USART_StopBits_1;
	USART_InitDef.USART_WordLength = USART_WordLength_8b;

	USART_Init(USART2 ,&USART_InitDef );
	USART_Cmd(USART2, ENABLE);

	//USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE); // W³¹czenie DMA request gdy Tx pusty
}

void RCC_Configuration(void)
{
	/* 8, 80, 2, 4  = 80,5MHz   ! ZMIERZONA ! */
	//HSITIM = 0x10000; fabrycznir
    RCC_DeInit();

    RCC_HSICmd(ENABLE);// W³¹cz wbudowany rezonator RC 16MHz
    RCC_PLLConfig(RCC_PLLSource_HSI,8,80,2,4); // ród³o z HSI ,Ustawienia PLL z CUB'A
    RCC_AdjustHSICalibrationValue(14);
    RCC_PLLCmd(ENABLE);// W³¹cz PLL

    RCC_PCLK1Config(RCC_HCLK_Div2);

    //RCC_ClockSecuritySystemCmd(ENABLE);
    FLASH_PrefetchBufferCmd(ENABLE);
    FLASH_SetLatency(FLASH_ACR_LATENCY_2WS);

    //RCC_HCLKConfig(RCC_SYSCLK_Div1);
    //RCC_PCLK1Config(RCC_HCLK_Div1);
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

   // while(RCC_GetSYSCLKSource() != 0x08); // Poczekaj a¿ Ÿród³em sygna³u bêdzie PLL (0x08)
    RCC_MCO1Config(RCC_MCO1Source_PLLCLK, RCC_MCO1Div_1); // zegar z PLL na PA8

}

void MX_ADC_Iinit(void)
{
	/*W³¹czanie zegara*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	ADC_DeInit(); 								//RESET rejestrów ADC

	ADC_InitTypeDef ADC_InitDef;				//Stuktura inicjalizacyjna
	ADC_CommonInitTypeDef ADC_CommmonInitDef;

	ADC_StructInit(&ADC_InitDef);				//Ustawia wszystkie pola strukury na domylne
	ADC_CommonStructInit(&ADC_CommmonInitDef);

	/* Konfiguracja trybu adc */
	ADC_CommmonInitDef.ADC_Mode = ADC_Mode_Independent;
	ADC_CommmonInitDef.ADC_Prescaler = ADC_Prescaler_Div4;
	ADC_CommmonInitDef.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommmonInitDef.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;

	/*Konfiguracja peryf adc*/
	ADC_InitDef.ADC_ContinuousConvMode = DISABLE;
	ADC_InitDef.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitDef.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
	ADC_InitDef.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_TRGO;
	ADC_InitDef.ADC_NbrOfConversion = 1;
	ADC_InitDef.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitDef.ADC_ScanConvMode = DISABLE;
	ADC_Init(ADC1, &ADC_InitDef);

	ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE); // W³¹czenie flagi End Of Conversion
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_480Cycles);
	ADC_TempSensorVrefintCmd(ENABLE); // w³aczenie Vint

	ADC_Cmd(ADC1, ENABLE); // W³¹czenie ADC

	/*Konfiguracja NVIC*/
	NVIC_InitTypeDef NVIC_InitDef;
	NVIC_InitDef.NVIC_IRQChannel = ADC_IRQn;
	NVIC_InitDef.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitDef.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitDef.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitDef);
}

void Usart_put_char(USART_TypeDef* USARTx, uint8_t dane)
{
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
	USART_SendData(USARTx, dane);
}

void Usart_put_string(char* dane)
{
	while(*dane)
	{
		Usart_put_char(USART2, *dane);
		dane++;
	}
}


// to z innego projektu TO DO -> COPY -> DELL

/* Konfig DMA Stream 1 , Channel 3 */
	/*RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE); // W³¹czenie zegara dla DMA1
	DMA_DeInit(DMA1_Stream1);
	DMA_StructInit(&DMA_InitDef);

	DMA_InitDef.DMA_BufferSize = 256;
	DMA_InitDef.DMA_Channel = DMA_Channel_3; // tim2 ch1 - DMA_Channel_3
	DMA_InitDef.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitDef.DMA_Memory0BaseAddr = gamma;
	DMA_InitDef.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
	DMA_InitDef.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
	DMA_InitDef.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitDef.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitDef.DMA_Mode = DMA_Mode_Circular;
	DMA_InitDef.DMA_Priority = DMA_Priority_High;
	DMA_InitDef.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitDef.DMA_FIFOThreshold = DMA_FIFOStatus_Empty;
	//DMA_InitDef.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitDef.DMA_PeripheralBaseAddr = (uint32_t)&(TIM2->CCR1); // adres:  (uint32_t)&(TIM2 -> CCR1)
	//0x40000000 + 0x34
	//USARTx->DR
	DMA_Init(DMA1_Stream1, &DMA_InitDef);
	DMA_Cmd(DMA1_Stream1, ENABLE);*/


/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "usb_host.h"

/* USER CODE BEGIN Includes */

// odczyt rejestrow
#define DRV8305_WARNING_REG_READ 		0x8800
#define DRV8305_OVVDS_REG_READ			0x9000
#define DRV8305_PWM_MODE_3_WRITE		0x3A96
#define DRV8305_GATE_DRIVE_REG_READ		0xBA16

// bity flag zabezp. pradowych (bity 10:0 w rejestrze OVVDS )
#define DRV8305_OVERCURRENT_FAULT_A	 	0x4
#define DRV8305_OVERCURRENT_FAULT_B	 	0x2
#define DRV8305_OVERCURRENT_FAULT_C	 	0x1

#define SINK_RESISTOR 5	// [mOhm]
#define PWM_PERIOD 800
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim10;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

//--- dane z rejestrow DRV8305 //--
uint16_t _Gate_Drive_Reg = 0;
uint16_t _Warning_Reg = 0;
uint16_t _OVVDS_Reg = 0;
uint16_t odp = 0;						// test komunikacji SPI2
uint8_t _Overcurrent_Sense_Flag = 0;	// odczyt flag przeciazenia pradowego
//---------------------------------

//--- odczyt czujnikow / pomiary //
int _Vsense_Read_A = 0;			// pomiary napiecia na rezystorach VDS
int _Vsense_Read_B = 0;			//
int _Vsense_Read_C = 0;			//
uint16_t _ADC_Joy_Read = 0;		// pomiar ADC joysticka
uint8_t _HAL_Sense_Flag = 0;	// odczyt czujnikow HALLa
//---------------------------------

//----konfiguracja ADC //----------
const float _ADC_Resolution = 4095.0;		// rozdzielczosc ADC
const float _Supply_Voltage = 3300;		// [mVolts]
//---------------------------------

unsigned int _PWM_Duty = 0;					// wypelnienie w procentach

// tablica 6 krokow BLDC (obrot w lewo)- 4 elementy: 1 - HALL binary(H1 = 0) 2,3,4 - fazy A,B,C [0 - n.c., 1 - lower, 2 - upper]
int _BLDC_Steps_Left[6][4] = {{5,0,1,2},	// step 1
							{1,2,1,0},		// step 2
							{3,2,0,1},		// step 3
							{2,0,2,1},		// step 4
							{6,1,2,0},		// step 5
							{4,1,0,2}};		// step 6

// tablica 6 krokow BLDC (obrot w prawo) - 4 elementy: 1 - HALL binary(H1 = 0) 2,3,4 - fazy A,B,C [0 - n.c., 1 - lower, 2 - upper]
int _BLDC_Steps_Right[6][4] = {{5,0,2,1},	// step 6
							 {1,1,2,0},		// step 5
							 {3,1,0,2},		// step 4
							 {2,0,1,2},		// step 3
							 {6,2,1,0},		// step 2
							 {4,2,0,1}};	// step 1

// ------- regulator PI -----------
double 		KU = 0,		// wzmocnienie, przy ktorym pojawiaja sie oscylacje
			PU = 0,			// okres oscylacji
			czas_iteracji = 1;	// ???

double		KP = 0,
			KI = 0;


int 		uchyb = 0,
			prog_napiecia = 1300,	// [mV]
			integrator = 0;
//---------------------------------

//uint16_t _ADC_Vsense_Read_A = 0;		// pomiary ADC rezystorow przeciazenia pradowego
//uint16_t _ADC_Vsense_Read_B = 0;		//
//uint16_t _ADC_Vsense_Read_C = 0;		//
unsigned int _VOL_A = 0;
unsigned int _PI_Test = 0;
unsigned int _Uchyb_Test = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM7_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC2_Init(void);
void MX_USB_HOST_Process(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void Set_Key(int key,int state)				// sterowanie gornymi kluczami falownika; key - numer klucza (1-3), state - stan (1/0)
{
	switch (key)
	{
	case 1:
		if(state)	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_9,GPIO_PIN_SET);
		else		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_9,GPIO_PIN_RESET);
	break;

	case 2:
		if(state)	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_10,GPIO_PIN_SET);
		else		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_10,GPIO_PIN_RESET);
	break;

	case 3:
		if(state)	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_11,GPIO_PIN_SET);
		else		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_11,GPIO_PIN_RESET);
	break;
	}
}

void PWM_SetValue( int channel, int duty_perc)		// ustawienie wypelnienia PWM, channel - kanal PWM (1,2,3), duty_perc - wypelnienie w procentach(1-100)
{
	int value = duty_perc*PWM_PERIOD/100;			// value - wartosc wypelnienia (0 - 1000)

    switch (channel)
    {
    case 1:
    	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, value);	// uaktualnienie wartosci wypelnienia
	break;

    case 2:
    	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, value);
    break;

    case 3:
    	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, value);
    break;
    }
}

uint16_t SPI2_SendRec(uint16_t data) {							// wysyla i pobiera dane po SPI 2
	uint8_t answer;												// data - dane do wyslania 16 bit
																// answer - odpowiedz urzadzenia 16 bit
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);

	HAL_SPI_TransmitReceive(&hspi2, (uint8_t*)&data, &answer, 1, HAL_MAX_DELAY);		// arg1 - struktura init
																						// arg2 - dane do wyslania (jawne rzutowanie na 8 bitow)
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);									// arg3 - dane zwrotne
	return answer;																		// arg4 - rozmiar bufora
																						// arg5 - maksymalny czas komunikacji
}

void DRV8305_Init()			// wysyla konfiguracje sterownika DRV8305
{
	_Gate_Drive_Reg = SPI2_SendRec(DRV8305_PWM_MODE_3_WRITE);
	//HAL_Delay(1000);
	//_Gate_Drive_Reg = SPI2_SendRec(DRV8305_GATE_DRIVE_REG_READ);
	//_Warning_Reg = SPI2_SendRec(DRV8305_WARNING_REG_READ);
	//_OVVDS_Reg = SPI2_SendRec(DRV8305_OVVDS_REG_READ);
}

void ADC_VSense_Read()		// odczytuje ADC Vsense i zapisuje do globalnych zmiennych
{
	HAL_ADC_Start(&hadc1);

		HAL_ADC_PollForConversion(&hadc1,10);
		_Vsense_Read_A = (_Supply_Voltage/(_ADC_Resolution/4)) * HAL_ADC_GetValue(&hadc1);

		HAL_ADC_PollForConversion(&hadc1,10);
		_Vsense_Read_B = (_Supply_Voltage/(_ADC_Resolution/4)) * HAL_ADC_GetValue(&hadc1);

		HAL_ADC_PollForConversion(&hadc1,10);
		_Vsense_Read_C = (_Supply_Voltage/(_ADC_Resolution/4)) * HAL_ADC_GetValue(&hadc1);

	HAL_ADC_Stop(&hadc1);
}

void ADC_Joy_Read()			// odczytuje ADC joystick i zapisuje do globalnej zmiennej
{
	if (HAL_ADC_PollForConversion(&hadc2, 10) == HAL_OK)
	{
		HAL_ADC_Start(&hadc2);
		_ADC_Joy_Read = HAL_ADC_GetValue(&hadc2);
		HAL_ADC_Start(&hadc2);
	}
}

uint8_t DRV8305_Overcurrent_Sense ()		// odczytuje rejestr OV/VDS, zwraca 8 bitow, z czego 0,1,2 to kolejno flagi A,B,C
{
	uint16_t aux_read = SPI2_SendRec(DRV8305_OVVDS_REG_READ);
	uint8_t Sense_ABC_flags = 0;

	if((aux_read & DRV8305_OVERCURRENT_FAULT_A) == DRV8305_OVERCURRENT_FAULT_A)
		Sense_ABC_flags = Sense_ABC_flags | 0x1;

	if((aux_read & DRV8305_OVERCURRENT_FAULT_B) == DRV8305_OVERCURRENT_FAULT_B)
		Sense_ABC_flags = Sense_ABC_flags | 0x2;

	if((aux_read & DRV8305_OVERCURRENT_FAULT_C) == DRV8305_OVERCURRENT_FAULT_C)
		Sense_ABC_flags = Sense_ABC_flags | 0x4;

	return Sense_ABC_flags;
}

uint8_t HAL_Sense ()			// odczytuje stan czujnikow HALLA, zwraca 8 bitow, z czego 0,1,2 to kolejno HAL_1, HAL_2, HAL_3
{
	uint8_t aux_flag = 0;

	if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_7)) 		// HAL_1
			aux_flag |= 0x1;
	else	aux_flag &= ~0x1;

	if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_8)) 		// HAL_2
			aux_flag |= 0x2;
	else	aux_flag &= ~0x2;

	if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_9)) 		// HAL_3
			aux_flag |= 0x4;
	else	aux_flag &= ~0x4;

	return aux_flag;
}

unsigned int  Regulacja_PI()
{
	KU = 0.35;
	PU = 0.1;

	KP = 0.6*KU;	// wartosci wzmocnien wedlug metody zieglera-nicholsa
	KI = 2*KP/PU;	//

	unsigned int output = 0;

	if(_Vsense_Read_A <= prog_napiecia
	|| _Vsense_Read_B <= prog_napiecia
	|| _Vsense_Read_C <= prog_napiecia)
	{
		uchyb = prog_napiecia - _Vsense_Read_A;

		integrator = 0;
		//integrator += uchyb*czas_iteracji;
		if(integrator > 1)	integrator = 1;

		output = KP*uchyb + KI*integrator;

		if(output > 98)	output = 98;
		if(output < 0)	output = 0;

		return output;
	}
	else
	{
		uchyb = 0;
		return 0;
	}
}

void Rotate()		// steruje kluczami wedlug odczytu HALLow
{
	uint8_t aux_hall = _HAL_Sense_Flag;

	if(_ADC_Joy_Read >= 500)		// obrot w lewo
	{
		for(int i = 0; i < 6; i++)						// iteracja po kolejnych stopniach obrotu
		{
			if(_BLDC_Steps_Left[i][0] == aux_hall)		// porownanie odczytu HALLow z tablica
			{
				for(int j = 1; j<4; j++)				// iteracja po fazach A B C
				{
					if(_BLDC_Steps_Left[i][j] == 0)
					{
						Set_Key(j,0);				// wylaczenie gornego klucza
						PWM_SetValue(j,0);			// wylaczenie PWM
					}
					else if(_BLDC_Steps_Left[i][j] == 1)
					{
						Set_Key(j,0);				// wylaczenie gornego klucza
						PWM_SetValue(j,_PWM_Duty);	// wlaczenie PWM
					}
					else if(_BLDC_Steps_Left[i][j] == 2)
					{
						Set_Key(j,1);				// wlaczenie gornego klucza
						PWM_SetValue(j,0);			// wylaczenie PWM
					}
				}
			}
		}
	}
	else if(_ADC_Joy_Read <= 470)		// obrot w prawo
	{
		for(int i = 0; i < 6; i++)						// iteracja po kolejnych stopniach obrotu
		{
			if(_BLDC_Steps_Right[i][0] == aux_hall)		// porownanie odczytu HALLow z tablica
			{
				for(int j = 1; j<4; j++)				// iteracja po fazach A B C
				{
					if(_BLDC_Steps_Right[i][j] == 0)
					{
						Set_Key(j,0);				// wylaczenie gornego klucza
						PWM_SetValue(j,0);			// wylaczenie PWM
					}
					else if(_BLDC_Steps_Right[i][j] == 1)
					{
						Set_Key(j,0);				// wylaczenie gornego klucza
						PWM_SetValue(j,_PWM_Duty);	// wlaczenie PWM
					}
					else if(_BLDC_Steps_Right[i][j] == 2)
					{
						Set_Key(j,1);				// wlaczenie gornego klucza
						PWM_SetValue(j,0);			// wylaczenie PWM
					}
				}
			}
		}
	}
	else
	{
		for(int j = 1; j<4; j++)		// iteracja po fazach A B C
		{
			Set_Key(j,0);				// wylaczenie gornego klucza
			PWM_SetValue(j,0);			// wylaczenie PWM
		}

	}
}

void Check_And_Rotate()
{
	_PI_Test = Regulacja_PI();

	if(_ADC_Joy_Read < 485)				// wychylenie galki w dol
	_PWM_Duty = ((100 - (_ADC_Joy_Read)/5)) /*- _PI_Test*/;
	else								// wychylenie galki w gore
	_PWM_Duty = ((2*((_ADC_Joy_Read-(_ADC_Resolution/8))/(_ADC_Resolution/4))*100)) /*- _PI_Test*/;	// regulacja PWM na podstawie joysticka

	ADC_VSense_Read();					// pomiar pradu kluczy
	_HAL_Sense_Flag = HAL_Sense();		// odczyt HALL
	ADC_Joy_Read();						// odczyt adc_joystick
	Rotate();							// ustawienie PWM i kluczy - obrot
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){	// funkcja operacji wykonywanych po zapelnieniu licznika

 if(htim->Instance == TIM7)
 {
	HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_15);						// led blink
 }

 if(htim->Instance == TIM10)
  {

  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{	// -----------------------------MAIN-----------------------------------------
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USB_HOST_Init();
  MX_ADC1_Init();
  MX_TIM10_Init();
  MX_TIM7_Init();
  MX_SPI2_Init();
  MX_TIM4_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim7);		// uruchomienie timera TIM7 	(blink LED)
  HAL_TIM_Base_Start_IT(&htim10);		// uruchomienie timera TIM10	(PWM control)

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);			// PWM start channel 1
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);			// PWM start channel 2
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);			// PWM start channel 3

  HAL_ADC_Start(&hadc1);				// uruchomienie konwersji ADC (PA1 ADC1_IN1)
  HAL_ADC_Start(&hadc2);

  //DRV8305_Init();		// konfiguracja ukladu DRV8305

  odp = SPI2_SendRec(0xBA16);		// SPI2 test

  //odp = SPI2_SendRec(0x8800);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // ------------------------------------------WHILE-----------------------------------------------------------------------
  while (1)
  {
	//_Overcurrent_Sense_Flag = DRV8305_Overcurrent_Sense();		// odczyt flag zabezpieczen pradowych

	Check_And_Rotate();

  /* USER CODE END WHILE */
    MX_USB_HOST_Process();

  /* USER CODE BEGIN 3 */

  }
 // ------------------------------------------------------------------------------------------------------------------------
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_10B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* ADC2 init function */
static void MX_ADC2_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_10B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2S3 init function */
static void MX_I2S3_Init(void)
{

  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 2-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim4);

}

/* TIM7 init function */
static void MX_TIM7_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 8400-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 5000-1;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM10 init function */
static void MX_TIM10_Init(void)
{

  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 2-1;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 1000-1;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PC3   ------> I2S2_SD
     PB10   ------> I2S2_CK
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, klucz_1_Pin|klucz_2_Pin|klucz_3_Pin|LD6_Pin 
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : HAL_in_1_Pin HAL_in_2_Pin HAL_in_3_Pin */
  GPIO_InitStruct.Pin = HAL_in_1_Pin|HAL_in_2_Pin|HAL_in_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : klucz_1_Pin klucz_2_Pin klucz_3_Pin LD6_Pin 
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = klucz_1_Pin|klucz_2_Pin|klucz_3_Pin|LD6_Pin 
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

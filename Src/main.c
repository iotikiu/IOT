
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "string.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
unsigned char buffer[3];
unsigned char ACC_X[2];
unsigned char ACC_Y[2];
unsigned char ACC_Z[2];
unsigned char TEMP[2];
unsigned char GYR_X[2];
unsigned char GYR_Y[2];
unsigned char GYR_Z[2];
uint16_t ACC_X_OUT;
uint16_t ACC_Y_OUT;
uint16_t ACC_Z_OUT;
uint16_t TEMP_OUT;
uint16_t GYR_X_OUT;
uint16_t GYR_Y_OUT;
uint16_t GYR_Z_OUT;
int ACC_OUT_X;
int ACC_OUT_Y;
int ACC_OUT_Z;
int GYR_OUT_X;
int GYR_OUT_Y;
int GYR_OUT_Z;
float  acc_x;
float  acc_y;
float  acc_z;
float  temp;
float  gyr_x;
float  gyr_y;
float gyr_z;
const float g=9.82;
char ch[20][20];
char cha[200];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_UART4_Init(void);
int HEXTODEC(uint16_t x);
float REAL_ACC (int x);
float REAL_TEMP (uint16_t x);
float REAL_GYR (int x);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
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
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
   buffer[0]=0x6B;
	 buffer[1]=0;
	 buffer[2]=0;
   HAL_I2C_Master_Transmit(&hi2c1,0x68<<1,buffer,2,100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */
    HAL_I2C_Mem_Read(&hi2c1,0x68<<1,0x3B,I2C_MEMADD_SIZE_8BIT,ACC_X,2,200);
	  ACC_X_OUT=ACC_X[0]<<8 | ACC_X[1];
		ACC_OUT_X=HEXTODEC(ACC_X_OUT);
		acc_x=REAL_ACC(ACC_OUT_X);
	  HAL_I2C_Mem_Read(&hi2c1,0x68<<1,0x3D,I2C_MEMADD_SIZE_8BIT,ACC_Y,2,200);
		ACC_Y_OUT=ACC_Y[0]<<8 | ACC_Y[1];
		ACC_OUT_Y=HEXTODEC(ACC_Y_OUT);
		acc_y=REAL_ACC(ACC_OUT_Y);
	  HAL_I2C_Mem_Read(&hi2c1,0x68<<1,0x3F,I2C_MEMADD_SIZE_8BIT,ACC_Z,2,200);
		ACC_Z_OUT=ACC_Z[0]<<8 | ACC_Z[1];
	  ACC_OUT_Z=HEXTODEC(ACC_Z_OUT);
	  acc_z=REAL_ACC(ACC_OUT_Z);
	  HAL_I2C_Mem_Read(&hi2c1,0x68<<1,0x41,I2C_MEMADD_SIZE_8BIT,TEMP,2,200);
		TEMP_OUT=TEMP[0]<<8 | TEMP[1];
		temp=REAL_TEMP(TEMP_OUT);
	  HAL_I2C_Mem_Read(&hi2c1,0x68<<1,0x43,I2C_MEMADD_SIZE_8BIT,GYR_X,2,200);
		GYR_X_OUT=GYR_X[0]<<8 | GYR_X[1];
		GYR_OUT_X=HEXTODEC(GYR_X_OUT);
		gyr_x=REAL_GYR(GYR_OUT_X);
	  HAL_I2C_Mem_Read(&hi2c1,0x68<<1,0x45,I2C_MEMADD_SIZE_8BIT,GYR_Y,2,200);
		GYR_Y_OUT=GYR_Y[0]<<8 | GYR_Y[1];
		GYR_OUT_Y=HEXTODEC(GYR_Y_OUT);
		gyr_y=REAL_GYR(GYR_OUT_Y);
	  HAL_I2C_Mem_Read(&hi2c1,0x68<<1,0x47,I2C_MEMADD_SIZE_8BIT,GYR_Z,2,200);
		GYR_Z_OUT=GYR_Z[0]<<8 | GYR_Z[1];
		GYR_OUT_Z=HEXTODEC(GYR_Z_OUT);
		gyr_z=REAL_GYR(GYR_OUT_Z);
		
		sprintf(cha,"%d %d %d",(int)ACC_X_OUT,(int)ACC_Y_OUT,(int)ACC_Z_OUT);
		HAL_UART_Transmit(&huart4,cha,strlen(cha),100);
		/*sprintf(ch[1],"%f ",acc_x);
		sprintf(ch[2],"%f ",acc_y);
		sprintf(ch[3],"%f ",acc_z);
		sprintf(ch[4],"%f ",temp);
		sprintf(ch[5],"%f ",gyr_x);
		sprintf(ch[6],"%f ",gyr_y);
		sprintf(ch[7],"%f",gyr_z);
		HAL_UART_Transmit(&huart4,ch[1],strlen(ch[1]),100);
		HAL_UART_Transmit(&huart4,ch[2],strlen(ch[2]),100);
		HAL_UART_Transmit(&huart4,ch[3],strlen(ch[3]),100);
		HAL_UART_Transmit(&huart4,ch[4],strlen(ch[4]),100);
		HAL_UART_Transmit(&huart4,ch[5],strlen(ch[5]),100);
		HAL_UART_Transmit(&huart4,ch[6],strlen(ch[6]),100);
		HAL_UART_Transmit(&huart4,ch[7],strlen(ch[7]),100);*/
  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
int HEXTODEC (uint16_t x){
	uint16_t lova=x;
	int type=0;
	int value=1;
	if(x>=0x8000){
		type =1;
		value=value*-1;
	}
	if (type==1){
	lova=x-0x0001;
	lova=0xFFFF-lova;
	}
	value=value*(int)lova;
	return value;
}
float REAL_ACC (int x){
	return (2*g*x)/(32767);
}
float REAL_TEMP (uint16_t x){
	float value=1;
	if(x>=0x8000){
		x=0xFFFF-x;
		value=-1;
	}
	value = value*((int)x)/340+36.53;
	return value;
}
float REAL_GYR (int x){
	return (250*x)/(32767);
}
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

/* UART4 init function */
static void MX_UART4_Init(void)
{

  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
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
*/
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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

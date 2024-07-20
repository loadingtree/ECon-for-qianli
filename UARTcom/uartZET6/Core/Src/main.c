/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t RxData[10]="haha";
uint8_t TxData[10];
uint8_t RxData1[10];
uint8_t TxData1[10]="hallo";
float num_test=3.14;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void float_to_u8_realnum(float ,uint8_t*);
void U32_to_U8(uint32_t FLin, uint8_t *U8out);
uint32_t U8_to_U32(uint8_t *U8in);
uint32_t FloatToU32(float dat);
float U32ToFloat(uint32_t dat);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  LED0(1);

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
    HAL_UART_Receive_IT(&huart2, RxData, 10);
  HAL_UART_Receive_IT(&huart1, RxData1, 10);
  float_to_u8_realnum(num_test,TxData1);
  uint32_t test=12345;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    U32_to_U8(test,TxData1);
    test=U8_to_U32(TxData1);
    HAL_UART_Transmit_IT(&huart1,TxData1,4);
    HAL_Delay(1000);
    HAL_UART_Transmit_IT(&huart1,TxData1,4);
    LED0_TOGGLE;
    HAL_Delay(1000);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void U32_to_U8(uint32_t FLin, uint8_t *U8out)
{
   uint32_t mid = (uint32_t)(FLin);
  U8out[0] = (mid >> 24) & 0xff;
  U8out[1] = (mid >> 16) & 0xff;
  U8out[2] = (mid >> 8) & 0xff;
  U8out[3] = (mid >> 0) & 0xff;
  /*
uint8_t farray[4];
*(float *)farray =*FLin;
U8out[3]= farray[0];
U8out[2]= farray[1];
U8out[1]= farray[2];
U8out[0]= farray[3];

//U8out[0]= farray[0];
//U8out[1]= farray[1];
//U8out[2]= farray[2];
//U8out[3]= farray[3];*/

}
uint32_t FloatToU32(float dat)
{
	uint8_t buf[4];

	buf[0] = ((uint8_t*)&dat)[0];
	buf[1] = ((uint8_t*)&dat)[1];
	buf[2] = ((uint8_t*)&dat)[2];
	buf[3] = ((uint8_t*)&dat)[3];

	return (buf[0] << 24) + (buf[1] << 16) + (buf[2] << 8) + buf[3];
}
float U32ToFloat(uint32_t dat)
{
	uint8_t buf[4];

	buf[0] = dat >> 24;
	buf[1] = dat >> 16;
	buf[2] = dat >> 8;
	buf[3] = dat & 0xff;

	return *((float*)buf);
}

uint32_t U8_to_U32(uint8_t *U8in)
{
  uint32_t mid = (U8in[0] << 24) | (U8in[1] << 16) | (U8in[2] << 8) | U8in[3];
  return (mid);
}
void float_to_u8_realnum(float FLin,uint8_t* U8out)//??????????
{
  int i,mid;
  mid=(int)FLin;
  FLin=(FLin-mid)*10;
  U8out[0]=(char)(mid+48);
  U8out[1]='.';
  for(i=2;i<8;i+=1)
  {
    mid=(int)FLin;
    U8out[i]=(char)(((int)FLin)+48);
    FLin=(FLin-mid)*10;
  }
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

  if(huart== & huart1){

      for(uint16_t i=0;i<10;i+=1)
    {
      TxData1[i]=RxData1[i];
    }
    //num_test=RxData1[0]-48;
    //for(uint16_t i=2;i<10;i+=1)
    //{
    //num_test +=(float)(RxData1[i]-48)/(10^(i-1));
    //}
    //float_to_u8_realnum(num_test,TxData1);
    LED1_TOGGLE;
    HAL_UART_Receive_IT(&huart1,RxData1,10);
  }
  if(huart== & huart2){
    HAL_UART_Receive_IT(&huart2,RxData,4);
    num_test=U8_to_Float(RxData);
    float_to_u8_realnum(num_test,TxData1);
    LED1_TOGGLE;
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

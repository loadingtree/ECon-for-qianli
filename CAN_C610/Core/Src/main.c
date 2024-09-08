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
#include "can.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "drv_can.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
 uint16_t ecd;
 int16_t speed_rpm;
 int16_t given_current;//
 uint8_t temperate;//
 int16_t last_ecd;
} motor_measure_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//get_motor_measure
#define CAN_CHASSIS_ALL_ID 0x1FF
#define CHASSIS_CAN        hcan1

#define CAN_6020_M1_ID     0x205
#define CAN_6020_M2_ID     0x206
#define CAN_6020_M3_ID     0x207
#define CAN_6020_M4_ID     0x208

#define CAN_C610_M1_ID     0x201
#define CAN_C610_M2_ID     0x202
#define CAN_C610_M3_ID     0x203
#define CAN_C610_M4_ID     0x204
#define get_motor_measure(ptr, data) \
 { \
 (ptr)->last_ecd = (ptr)->ecd; \
 (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]); \
 (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]); \
 (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]); \
 (ptr)->temperate = (data)[6]; \
 }
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef chassis_tx_message;
 motor_measure_t motor_chassis[8];//
 uint8_t tic=0;
 uint16_t v6020_1=2500;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void motorcan(void);
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
  CAN_Init(&CHASSIS_CAN);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      motorcan();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//prepare freertos
void motorcan(void)
{
    HAL_Delay(50);
  CAN_cmd_chassis(v6020_1,10000,10000,10000);
    //if(motor_chassis[4].ecd>4000)
}
//
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
  /*uint32_t send_mail_box=CAN_TX_MAILBOX0;
 chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
 chassis_tx_message.IDE = CAN_ID_STD;
 chassis_tx_message.RTR = CAN_RTR_DATA;
 chassis_tx_message.DLC = 0x08;*/
 uint8_t chassis_can_send_data[8];
 chassis_can_send_data[0] = motor1 >> 8;
 chassis_can_send_data[1] = motor1;
 chassis_can_send_data[2] = motor2 >> 8;
 chassis_can_send_data[3] = motor2;
 chassis_can_send_data[4] = motor3 >> 8;
 chassis_can_send_data[5] = motor3;
 chassis_can_send_data[6] = motor4 >> 8;
 chassis_can_send_data[7] = motor4;
 if(CAN_SEND_Data(&CHASSIS_CAN,CAN_CHASSIS_ALL_ID,chassis_can_send_data,8)!=HAL_OK)
 return;
tic += 1;
}
//
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
 CAN_RxHeaderTypeDef rx_header;
 uint8_t rx_data[8];

 HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
 switch (rx_header.StdId)
 {
 case CAN_6020_M1_ID:
 case CAN_6020_M2_ID:
 case CAN_6020_M3_ID:
 case CAN_6020_M4_ID:
 case CAN_C610_M1_ID:
 case CAN_C610_M2_ID:
 case CAN_C610_M3_ID:
 case CAN_C610_M4_ID:
 {
 static uint8_t i = 0;
 //get motor id
 i = rx_header.StdId - CAN_C610_M1_ID;
 get_motor_measure(&motor_chassis[i], rx_data);
 tic =0;
 break;
 }
 default:
 {
 break;
 }
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

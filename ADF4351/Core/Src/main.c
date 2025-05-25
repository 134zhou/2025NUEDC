/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ADF4351.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define ADC_Samples 1
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint16_t point;
uint16_t cnt = 0;
float fre_begin=90;
float fre_end=110;

float point_fre = 0;//点频值

uint16_t adc_buffer[ADC_Samples];
uint8_t rx_data;
uint8_t rx_buffer[8];//输入缓存
uint8_t rx_cnt;//输入计数

uint8_t sign = 0;//变频与采样选择
uint8_t mode = 0;//点频与扫频选择

uint8_t state_entered = 0;  // 进入标志，表示是否刚进入状态
uint8_t state_jump = 0;//状态跳转值
		
float buffer_vol[5];
float peak_vol = 0;
uint8_t vol_cnt=0;

typedef enum {
    STATE_IDLE,
    STATE_POINT,
    STATE_SCAN,
		STATE_PRINT
} State;

State current_state = STATE_IDLE;
State next_state = STATE_IDLE;


void StateMachine_Update(void)
{
    // 状态跳转逻辑
    switch (current_state)
    {
        case STATE_IDLE:
            if (state_jump == 1)
                next_state = STATE_POINT;
						else if(state_jump == 2)
								next_state = STATE_SCAN;
						else if(state_jump == 3)
								next_state = STATE_PRINT;
            else
                next_state = STATE_IDLE;
            break;

        case STATE_POINT:
            if (state_jump == 0)
                next_state = STATE_IDLE;
            else
                next_state = STATE_POINT;
            break;

        case STATE_SCAN:
            if (state_jump == 0)
                next_state = STATE_IDLE;
            else
                next_state = STATE_SCAN;
            break;
        case STATE_PRINT:
            if (state_jump == 0)
                next_state = STATE_IDLE;
            else
                next_state = STATE_PRINT;
            break;
    }

    // 如果状态切换，清除进入标志
    if (next_state != current_state)
        state_entered = 0;
		current_state = next_state;

    // 状态执行
    switch (current_state)
    {
        case STATE_IDLE:
            if (!state_entered)
            {
                state_entered = 1;
            }
            break;

        case STATE_POINT:
            if (!state_entered)
            {
                state_entered = 1;
                // 进入POINT状态时只执行一次的动作
								ADF4351_Freq_Setting(point_fre);
            }
            // POINT状态中需要反复执行的动作也写这里
            break;

        case STATE_SCAN:
            if (!state_entered)
            {
                state_entered = 1;
                // 进入SCAN状态时只执行一次的动作
								HAL_TIM_Base_Start_IT(&htim3);
            }
            // SCAN状态中循环执行的动作
            break;
				case STATE_PRINT:
						if(!state_entered)
						{
                state_entered = 1;
                // 进入SCAN状态时只执行一次的动作
								HAL_TIM_Base_Start_IT(&htim3);
            }
    }
}

//串口解码
void Uart_Decode(uint8_t rx_buffer[])
{
	if(rx_buffer[0]=='F'&&rx_buffer[1]=='F')//状态机跳转指令
	{
		state_jump = rx_buffer[2]-48;
	}
	else if(rx_buffer[0]=='F'&&rx_buffer[1]=='D')//数据
	{
		point_fre = (1000*(rx_buffer[2]-48)+100*(rx_buffer[3]-48)+10*(rx_buffer[3]-48)+(rx_buffer[4]-48))/100.0;

	}
}

//串口发送重定义
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}
//扫频范围设置
void Freq_Sweep(float fre_begin, float fre_end)
{
	uint16_t fre_scale;
	fre_scale = fre_end - fre_begin;
	point = 100*fre_scale;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	float freq = 0;
	if(htim == &htim3)
	{
		if(!sign)//为0时设置频率
		{
			sign = !sign;
			freq = fre_begin + 0.01*cnt;
			ADF4351_Freq_Setting(freq);
		}
		else//为1时进行采样，并且更新cnt
		{
			sign = !sign;
			if(cnt == point)
			{
				HAL_TIM_Base_Stop_IT(&htim3);
				cnt = 0;
			}			
			else
			{
				cnt = cnt+1;
				HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_Samples);
			}
		}
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if(hadc == &hadc1)
	{
		HAL_ADC_Stop_DMA(&hadc1);
		uint32_t sum = 0;
		float ad8307_v = 0;
		for(int i =0; i<ADC_Samples; i++)
		{
			sum = sum +adc_buffer[i];
		}
		ad8307_v = sum/ADC_Samples*3.3/4096-1.09;
//		printf("%f\n", ad8307_v);
		//每5个数据找出最大值
	  buffer_vol[vol_cnt] = ad8307_v;
		vol_cnt++;
		if(vol_cnt >= 5)
		{
			float max_val = buffer_vol[0];
        for (int i = 1; i < 5; i++)
        {
            if (buffer_vol[i] > max_val)
                max_val = buffer_vol[i];
        }
				peak_vol = max_val;
				vol_cnt = 0;
				uint8_t result = (uint8_t)(peak_vol * 10006);
				printf("add s0.id,0,%d\xff\xff\xff", result);
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1)
	{
		if(rx_data == '\n')//指令结束
		{
			rx_cnt = 0;
			Uart_Decode(rx_buffer);
			state_entered = 0;
		}
		else //接收
		{
			rx_buffer[rx_cnt] = rx_data;
			rx_cnt++;
		}
		HAL_UART_Receive_IT(&huart1, &rx_data, 1);
	}
}



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
	Freq_Sweep(fre_begin, fre_end);
	ADF4351_Init();
	HAL_UART_Receive_IT(&huart1, &rx_data, 1);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		HAL_Delay(100);
		StateMachine_Update();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
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

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  * Test RTOS, SEMA, QUEUE CON 1 Y VARIOS VALORES, CON INT/ISR
  *
  * Queues con más valores, Mutex, Sema Cont, VList NO, VSuspend NO.
  * pdMS_TO_TICKS, tarea con interrupciones sincrónicas.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
// #include "event_groups.h"
#include <stdio.h>
#include <string.h>
#include "MY_NRF24.h"
#include "stm32f4xx_hal.h"
//#include "arm_math.h"
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
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_tx;
DMA_HandleTypeDef hdma_spi2_rx;

UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
SemaphoreHandle_t Semaphore1;
SemaphoreHandle_t Semaphore2;
SemaphoreHandle_t Semaphore3;
SemaphoreHandle_t Semaphore4;

QueueHandle_t Queue_Data;
QueueHandle_t Queue_FFT;
QueueHandle_t Alert;

int cte0 = 0; int cte1 = 0;
int cte2 = 0; int cte3 = 0;
int cte4 = 0;

TaskHandle_t xHandle_Init = NULL;


uint8_t get_hardware[28] = "\nArduino Data Logger\r\n\r\n";
uint8_t get_gain[10] = "8\r\n\r\n";
uint8_t get_fs[10] = "2\r\n\r\n";
uint8_t get_decimalChannelMask[6] = "1\r\n\r\n";
uint8_t get_SerialNumber[45] = "FBFCD069-50593539-302E3120-FF110E08\r\n\r\n";
uint8_t get_SystemID[10] = "1\r\n\r\n";
uint8_t set_SystemID[32] = "param_iternalSystemID=0\r\n\r\n";
uint8_t Received[52] = "Arduino Received the following command : 11\r\n\r\n";
uint8_t bytes[8] = "1234\r\n\r\n"; // 4 bytes for the storage of individuals

#define DATA_POINTS 4096
//#define FFT_BUFFER_SIZE 4096 // This size must be a power of 2

//arm_rfft_fast_instance_f32 fft_instance;
//float32_t data_in[FFT_BUFFER_SIZE];
//float32_t data_out_fft[FFT_BUFFER_SIZE];
//uint8_t fftFlag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
void Start_ADC(void const * argument);
void FFT_Data(void const * argument);
void Output_Data(void const * argument);
void Init_Param(void const * argument);

float INL_Messages (void);		//uint32_t INL_Messages (void)
void INL_UART_Final (uint32_t);		//void INL_UART_Final (float)
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch){
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 100);
	return ch;
}
uint8_t address[][6] = { "1Node", "2Node" };
//uint64_t address1 = 0x1122334401;
//uint64_t address2 = 0x1122334402;

char myTxData[32] = "11";
char myRxData[32];

bool radioNumber;
int32_t data_time=0;
int32_t data_freq=0;
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
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
	NRF24_begin(CSNpin_GPIO_Port, CSNpin_Pin, CEpin_Pin, hspi2);
	nrf24_DebugUART_Init(huart2);

	NRF24_setAutoAck(true);
	NRF24_setChannel(76);
	NRF24_setPayloadSize(sizeof(myRxData));
	radioNumber = 0;
	NRF24_startListening();
	NRF24_openReadingPipe_B(1, address[!radioNumber]);

	HAL_Delay(100);
	NRF24_stopListening();
	NRF24_openWritingPipe_B(address[radioNumber]);
	//k = 0;
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  Semaphore1 = xSemaphoreCreateBinary();
  if(Semaphore1 == 0)
    {
  	  printf("Semaphore1 was not created\n\r");
    }
    else
    {
    	printf("Semaphore1 was created\n\r");
    }
  Semaphore2 = xSemaphoreCreateBinary();
  if(Semaphore2 == 0)
      {
	  	  printf("Semaphore2 was not created\n\r");
      }
      else
      {
    	  printf("Semaphore2 was created\n\r");
      }
  Semaphore3 = xSemaphoreCreateBinary();
  if(Semaphore3 == 0)
  	  {
      	  printf("Semaphore3 was not created\n\r");
  	  }
  else
  	  {
	  	  printf("Semaphore3 was created\n\r");
  	  }
  Semaphore4 = xSemaphoreCreateBinary();
  if(Semaphore4 == 0)
    	  {
        	  printf("Semaphore4 was not created\n\r");
    	  }
    else
    	  {
  	  	  printf("Semaphore4 was created\n\r");
    	  }
  xSemaphoreGive(Semaphore1);
  //xSemaphoreGive(Semaphore4);
  //xSemaphoreTake(Semaphore2, 0);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  Queue_FFT = xQueueCreate( DATA_POINTS, sizeof(float) ); //Queue_FFT
  if(Queue_FFT == 0)
  {
	  printf("Queue Queue_FFT was not created\n\r");
  }
  else
  {
	  printf("Queue Queue_FFT was created\n\r");
  }
  Queue_Data = xQueueCreate( DATA_POINTS, sizeof(float) ); // cantidad y tipo/tamaño Queue_Data
  if(Queue_Data == 0)
  {
	  printf("Queue Queue_Data was not created\n\r");
  }
  else
  {
	  printf("Queue Queue_Data was created\n\r");
  }
  Alert = xQueueCreate( 1, sizeof(float) ); // cantidad y tipo/tamaño Queue_Data
  if(Alert == 0)
  {
	  printf("Queue Queue_Data was not created\n\r");
  }
  else
  {
	  printf("Queue Queue_Data was created\n\r");
  }
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  xTaskCreate((void *)Start_ADC, "startADC", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 4, NULL);
  xTaskCreate((void *)FFT_Data, "FFTproc", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 3, NULL);
  xTaskCreate((void *)Output_Data, "OutData", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL);
  xTaskCreate((void *)Init_Param, "ParamIn", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, &xHandle_Init);
  vTaskStartScheduler();
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CSNpin_Pin|CEpin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CSNpin_Pin CEpin_Pin */
  GPIO_InitStruct.Pin = CSNpin_Pin|CEpin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void Start_ADC(void const * argument){
	float Data_In[DATA_POINTS];
	int uxNumberOfFreeSpaces = 0;
	int i = 0;
	UBaseType_t uxPriority;
	uxPriority = uxTaskPriorityGet( NULL );
	while(1){
		vTaskPrioritySet( xHandle_Init, ( uxPriority + 1));
		if(xSemaphoreTake(Semaphore1,portMAX_DELAY)==pdTRUE){
			for(i = 0; i<DATA_POINTS; i++){
				Data_In[i] = INL_Messages();
				bool report = NRF24_write(&myTxData, sizeof(myTxData));
				if (report) {
					NRF24_startListening();
					if(NRF24_available()){
						HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
						NRF24_read(myRxData, sizeof(myRxData));
						myRxData[32] = '\r'; myRxData[32+1] = '\n';
						data_time = (atof(myRxData))/1;
						bytes[0] = (data_time >> 24) & 0xFF;
						bytes[1] = (data_time >> 16) & 0xFF;
						bytes[2] = (data_time >> 8) & 0xFF;
						bytes[3] = data_time & 0xFF;
					}
				}
				Data_In[i] = data_time;
				xQueueSend(Queue_Data, &Data_In[i], portMAX_DELAY);
				uxNumberOfFreeSpaces = uxQueueSpacesAvailable( Queue_Data );
			}
			xSemaphoreGive(Semaphore2);
		}
	}
}
void FFT_Data(void const * argument)
{
	float Data[DATA_POINTS];
	float Processed[DATA_POINTS];
	float out[1];
	int uxNumberOfFreeSpaces = 0;
	int i = 0, k = 0, j = 0;
	float aux, sum = 0, prom;

	//arm_rfft_fast_init_f32(&fft_instance, DATA_POINTS);

	while(1)
	{
		if(xSemaphoreTake(Semaphore2,portMAX_DELAY)==pdTRUE){
			for(i = 0; i<DATA_POINTS; i++){
				xQueueReceive(Queue_Data, &Data[i], portMAX_DELAY);
				Processed[i] = Data[i];
				if(i == DATA_POINTS-1){
					// apply FFT
			//		//arm_rfft_fast_f32(&fft_instance, Data, Processed, 0);
					// extract absolute values by computing the magnitude of the complex numbers
					// Pay attention that after this operation,
					// half of the buffer possesses the magnitudes
			//		//arm_cmplx_mag_f32(Processed, Processed, FFT_BUFFER_SIZE/2);
					// bias removal
					Processed[0] = 0;
				}
				xQueueSend(Queue_FFT, &Processed[i], portMAX_DELAY);
				uxNumberOfFreeSpaces = uxQueueSpacesAvailable( Queue_Data );
			}

			xSemaphoreGive(Semaphore3);
			/*
			if(uxNumberOfFreeSpaces > DATA_POINTS-1){
				for(k = 0;k<DATA_POINTS;k++) {
					for(j = 0;j<4-k;j++) {
						if (Processed[j]>Processed[j+1]) {
							aux=Processed[j];
							Processed[j]=Processed[j+1];
							Processed[j+1]=aux;
						}
					}
				}
				k=1;
				for(k = 1;k<DATA_POINTS-1;k++) {
					sum = sum + Processed[k];
				}
				prom = sum/(DATA_POINTS-2);
				out[0] = prom;
			}*/
		}
	}
}

void Output_Data(void const * argument)
{
	float Data[DATA_POINTS];
	float Final_Output[1];		//float flag_Alert[1];

	int uxNumberOfFreeSpaces;	//TickType_t T1, T2, TE;
	const TickType_t xBlockTime = pdMS_TO_TICKS( 10 ); // 10 ms a Ticks
	UBaseType_t uxPriority; int i = 0;
	uxPriority = uxTaskPriorityGet( NULL );
	while(1)
	{
		if(xSemaphoreTake(Semaphore3,portMAX_DELAY)==pdTRUE)
		{	/* Get the time the function started. */
			for(i = 0; i<DATA_POINTS; i++){
				xQueueReceive(Queue_FFT, &Data[i], portMAX_DELAY);
				HAL_UART_Transmit_DMA(&huart2, &Data[i], sizeof(float));
				uxNumberOfFreeSpaces = uxQueueSpacesAvailable( Queue_FFT );
			}
			//uxNumberOfFreeSpaces = uxQueueSpacesAvailable( Queue_FFT );
			//xQueueReceive(Queue_FFT, Data, portMAX_DELAY);
			//uxNumberOfFreeSpaces = uxQueueSpacesAvailable( Queue_FFT );
			/*
			Final_Output[0] = Data[0] +1;
			if(Final_Output[0]<1){
				HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
			}
			else{
				HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
				INL_UART_Final(Final_Output[0]);
			}*/
			cte0 = 11;
			cte0 = 22;
			vTaskDelay(xBlockTime);
			xSemaphoreGive(Semaphore1);
		}
	}
}

void Init_Param( void const * argument)
{
	UBaseType_t uxPriority;
	uxPriority = uxTaskPriorityGet( xHandle_Init );
	const TickType_t xBlockTime = pdMS_TO_TICKS( 10 ); // 10 ms a Ticks

	for( ;; )
	{
		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		uxPriority = uxTaskPriorityGet( NULL );
		if(uxPriority >1){
			cte0 = 444;

			INL_Messages();

			vTaskDelete(xHandle_Init);
		}
		else
			vTaskDelay(xBlockTime);
	}
}

float INL_Messages (void)
{
	HAL_UART_Transmit(&huart2, get_hardware, sizeof(get_hardware), 10);

	HAL_UART_Transmit(&huart2, get_gain, sizeof(get_gain), 10);

	HAL_UART_Transmit(&huart2, get_fs, sizeof(get_fs), 10);

	HAL_UART_Transmit(&huart2, get_hardware, sizeof(get_hardware), 10);

	HAL_UART_Transmit(&huart2, get_decimalChannelMask, sizeof(get_decimalChannelMask), 10);

	HAL_UART_Transmit(&huart2, get_SerialNumber, sizeof(get_SerialNumber), 10);

	return 0;
}

void INL_UART_Final (uint32_t Salida)
{
	float flagMessageR = 0;

	int uxNumberOfFreeSpaces;
	uxNumberOfFreeSpaces = uxQueueSpacesAvailable( Alert );
	if(uxNumberOfFreeSpaces < 1){
		xQueueReceive(Alert, &flagMessageR, portMAX_DELAY);
	}
	if(flagMessageR > 0)
		cte4 = 100+flagMessageR;
	else
		cte4 = 200+flagMessageR;
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

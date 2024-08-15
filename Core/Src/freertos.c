/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "adc.h"
#include "usart.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FRAME_HEADER 0xAA
#define FRAME_TAIL 0x55

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void CreateDataFrame(uint16_t *adc_value, uint8_t *buffer);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId ADC_CaptureHandle;
osThreadId Data_TrasmitHandle;
osMessageQId Data_QueueHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void MyTask1(void const * argument);
void MyTask2(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of Data_Queue */
  osMessageQDef(Data_Queue, 1, uint8_t[22]);
  Data_QueueHandle = osMessageCreate(osMessageQ(Data_Queue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of ADC_Capture */
  osThreadDef(ADC_Capture, MyTask1, osPriorityLow, 0, 128);
  ADC_CaptureHandle = osThreadCreate(osThread(ADC_Capture), NULL);

  /* definition and creation of Data_Trasmit */
  osThreadDef(Data_Trasmit, MyTask2, osPriorityIdle, 0, 128);
  Data_TrasmitHandle = osThreadCreate(osThread(Data_Trasmit), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_MyTask1 */
/**
* @brief Function implementing the ADC_Capture thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MyTask1 */
void MyTask1(void const * argument)
{
  /* USER CODE BEGIN MyTask1 */
  /* Infinite loop */
   uint16_t adc_value[10];
	 uint8_t adc_buffer[22]={0};
    while (1) 
		{
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 100);
			for(int i=0;i<10;i++)
			{
					adc_value[i] = HAL_ADC_GetValue(&hadc1);
				  
			}
				CreateDataFrame(adc_value, adc_buffer);
        xQueueSend(Data_QueueHandle,adc_buffer, portMAX_DELAY);
			  
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
  /* USER CODE END MyTask1 */
}

/* USER CODE BEGIN Header_MyTask2 */
/**
* @brief Function implementing the Data_Trasmit thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MyTask2 */
void MyTask2(void const * argument)
{
  /* USER CODE BEGIN MyTask2 */
  /* Infinite loop */
		uint8_t adc_ans[22];
    while (1) 
		{
			  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
        xQueueReceive(Data_QueueHandle,adc_ans, portMAX_DELAY);
				
			 
				
				
//			for(int i=0;i<4;i++)
//			{
//					printf("%x\r\n",adc_ans[i]);
//			}
//				
				//memcpy(adc_ans,&adc_value,4);
        HAL_UART_Transmit(&huart1, (uint8_t *)adc_ans, sizeof(adc_ans)/sizeof(uint8_t), HAL_MAX_DELAY);
			  vTaskDelay(pdMS_TO_TICKS(1000));
		} 
  /* USER CODE END MyTask2 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void CreateDataFrame(uint16_t *adc_value, uint8_t *buffer) {
    // 添加帧头
    buffer[0] = FRAME_HEADER;
    buffer[1] = FRAME_TAIL;
    // 添加数据（假设数据为 2 字节）
		for(int i=0,j=1;i<10;i+=1,j+=2){
			
			buffer[j+1] = (uint8_t)(adc_value[i] & 0xFF);        // 低字节
			buffer[j+2] = (uint8_t)((adc_value[i] >> 8) & 0xFF); // 高字节
		
		}
    
    
    // 添加帧尾
    

}
/* USER CODE END Application */


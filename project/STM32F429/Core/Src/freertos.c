/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "ip_addr.h"
#include "semphr.h"
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
/* USER CODE BEGIN Variables */
modbus_t telegram[2];
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTaskModbusTCP */
osThreadId_t myTaskModbusTCPHandle;
const osThreadAttr_t myTaskModbusTCP_attributes = {
  .name = "myTaskModbusTCP",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myTaskMaster */
osThreadId_t myTaskMasterHandle;
const osThreadAttr_t myTaskMaster_attributes = {
  .name = "myTaskMaster",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for TaskmyTaskSlave */
osThreadId_t TaskmyTaskSlaveHandle;
const osThreadAttr_t TaskmyTaskSlave_attributes = {
  .name = "TaskmyTaskSlave",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myTaskTCPS */
osThreadId_t myTaskTCPSHandle;
const osThreadAttr_t myTaskTCPS_attributes = {
  .name = "myTaskTCPS",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTaskMaster(void *argument);
void StartTasSlave(void *argument);
void StartTaskTCPH(void *argument);
void StartTaskTCPS(void *argument);

extern void MX_LWIP_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of myTaskModbusTCP */
  // myTaskModbusTCPHandle = osThreadNew(StartTaskTCPH, NULL, &myTaskModbusTCP_attributes);

  /* creation of myTaskTCPS */
  // myTaskTCPSHandle = osThreadNew(StartTaskTCPS, NULL, &myTaskTCPS_attributes);

  /* creation of myTaskMaster */
  myTaskMasterHandle = osThreadNew(StartTaskMaster, NULL, &myTaskMaster_attributes);

  /* creation of TaskmyTaskSlave */
  TaskmyTaskSlaveHandle = osThreadNew(StartTasSlave, NULL, &TaskmyTaskSlave_attributes);



  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  MX_LWIP_Init();
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTaskTCPH */
/**
* @brief Function implementing the myTaskModbusTCP thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskTCPH */
void StartTaskTCPH(void *argument)
{
  /* USER CODE BEGIN StartTaskTCPH */
  modbus_t telegram;
	uint32_t u32NotificationValue;

	telegram.u8id = 1; // slave address
	telegram.u8fct = MB_FC_WRITE_MULTIPLE_REGISTERS; // function code (this one is registers read)
	//telegram.u16RegAdd = 0x160; // start address in slave
	telegram.u16RegAdd = 0x0; // start address in slave
	telegram.u16CoilsNo = 10; // number of elements (coils or registers) to read
	telegram.u16reg = TCPHDATA; // pointer to a memory array in the Arduino
	IP_ADDR4((ip4_addr_t *)&telegram.xIpAddress, 192, 168, 10, 107); //address of the slave
	telegram.u16Port = 502;
  /* Infinite loop */
	for(;;)
	{
		TCPHDATA[0]++;
    TCPHDATA[9]++;
		ModbusQuery(&ModbusTCPH, telegram); // make a query
    u32NotificationValue = ulTaskNotifyTake(pdTRUE, 500); // block until query finishes or timeouts
    if(u32NotificationValue)
    {
      //handle error
      //while(1);
    }
    osDelay(500);
	}
  /* USER CODE END StartTaskTCPH */
}
/* USER CODE BEGIN Header_StartTaskMaster */
/**
* @brief Function implementing the myTaskMaster thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskMaster */
void StartTaskMaster(void *argument)
{
  /* USER CODE BEGIN StartTaskMaster */
	uint32_t u32NotificationValue;

	telegram[0].u8id = 1; // slave address
	telegram[0].u8fct = 3; // function code (this one is registers read)
	//telegram[0].u16RegAdd = 0x160; // start address in slave
	telegram[0].u16RegAdd = 0x0; // start address in slave
	telegram[0].u16CoilsNo = 1; // number of elements (coils or registers) to read
	telegram[0].u16reg = RTUHDATA; // pointer to a memory array

	// telegram 0: read registers
	telegram[1].u8id = 1; // slave address
	telegram[1].u8fct = 6; // function code (this one is registers write)
	//telegram[1].u16RegAdd = 0x160; // start address in slave
	telegram[1].u16RegAdd = 0x0;
	telegram[1].u16CoilsNo = 1;
	telegram[1].u16reg = RTUHDATA; // pointer to a memory array

    /* Infinite loop */
    for(;;)
    {
      ModbusQuery(&ModbusRTUH, telegram[0]); // make a query
      u32NotificationValue = ulTaskNotifyTake(pdTRUE, 500); // block until query finishes or timeout
      if(u32NotificationValue)
      {
      //handle error
      //  while(1);
      }
      osDelay(500);
      RTUHDATA[0]++;
      ModbusQuery(&ModbusRTUH, telegram[1]); // make a query
      u32NotificationValue = ulTaskNotifyTake(pdTRUE, 500); // block until query finishes or timeout
      if(u32NotificationValue)
      {
        //handle error
        //  while(1);
      }
      osDelay(500);
    }
  /* USER CODE END StartTaskMaster */
}

/* USER CODE BEGIN Header_StartTasSlave */
/**
* @brief Function implementing the TaskmyTaskSlave thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTasSlave */
void StartTasSlave(void *argument)
{
  /* USER CODE BEGIN StartTasSlave */
  /* Infinite loop */
  for(;;)
  {
    xSemaphoreTake(ModbusRTUS.ModBusSphrHandle , portMAX_DELAY);
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, ModbusRTUS.u16regs[0] & 0x1);
    xSemaphoreGive(ModbusRTUS.ModBusSphrHandle);
    osDelay(250);
  }
  /* USER CODE END StartTasSlave */
}

/* USER CODE BEGIN Header_StartTaskTCPS */
/**
* @brief Function implementing the myTaskTCPS thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskTCPS */
void StartTaskTCPS(void *argument)
{
  /* USER CODE BEGIN StartTaskTCPS */
  /* Infinite loop */
  for(;;)
  {
    // xSemaphoreTake(ModbusTCPS.ModBusSphrHandle , portMAX_DELAY);
    // HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, ModbusTCPS.u16regs[0] & 0x1);
    // xSemaphoreGive(ModbusTCPS.ModBusSphrHandle);
    osDelay(250);
  }
  /* USER CODE END StartTaskTCPS */
}
/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

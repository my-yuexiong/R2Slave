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
#include "RobotMain.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern uint32_t step_count;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for recvRemote */
//osThreadId_t recvRemoteHandle;
//const osThreadAttr_t recvRemote_attributes = {
//  .name = "recvRemote",
//  .stack_size = 512 * 4,
//  .priority = (osPriority_t) osPriorityNormal,
//};
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for RobotMove */
osThreadId_t RobotMoveHandle;
const osThreadAttr_t RobotMove_attributes = {
  .name = "RobotMove",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

//void RecvRemote(void *argument);
void StartDefaultTask(void *argument);
void StartRobotMove(void *argument);

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
  /* creation of recvRemote */
//  recvRemoteHandle = osThreadNew(RecvRemote, NULL, &recvRemote_attributes);

  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of RobotMove */
  RobotMoveHandle = osThreadNew(StartRobotMove, NULL, &RobotMove_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_RecvRemote */
/**
  * @brief  Function implementing the recvRemote thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_RecvRemote */
//void RecvRemote(void *argument)
//{
//  /* USER CODE BEGIN RecvRemote */
//////  /* Infinite loop */
////////    RecvDR16Thread();
//////  while(1)
//////  {
//////      osDelay(100);
//////  }
//  /* USER CODE END RecvRemote */
//}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
* @brief Function implementing the defaultTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
//  RobotTest();
  //RobotMain();
  while(1)
  {
      osDelay(100);
  }

  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartRobotMove */
/**
* @brief Function implementing the RobotMove thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRobotMove */
void StartRobotMove(void *argument)
{
  /* USER CODE BEGIN StartRobotMove */
//    RobotTest();
    RobotRecvMasterCmdThread();
  /* Infinite loop */
    while(1)
    {
        osDelay(1);
    }

  /* USER CODE END StartRobotMove */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */


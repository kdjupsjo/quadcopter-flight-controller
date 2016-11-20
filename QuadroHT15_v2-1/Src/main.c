/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main quadrocopter program
  ******************************************************************************

    TODOS:


 */



/* Includes ------------------------------------------------------------------*/

#include "main.h"

/* Thread IDs ----------------------------------------------------------------*/

osThreadId mainTaskHandle;

osThreadId pwmInTaskHandle;

osThreadId analysTaskHandle;

osThreadId loadTaskHandle;

/* Mailbox Definitions--------------------------------------------------------*/

osMailQDef(pwmIn_mailbox, 1, pwmIn_struct);
osMailQId pwmIn_mailbox;

osMailQDef(analys_mailbox, 1, main_struct);
osMailQId analys_mailbox;

/* Global, iNemo or the other one. 1 for iNemo, 0 for the other one. */

uint8_t iNemo = 0;
   
/* Global. Kalman or complementary. 1 for kalman, 0 for complementary */

uint8_t kalman = 0;

/* Global frequency  -----------------------------------------------*/
const portTickType MAIN_FREQUENCY = 4; 

 
int main(void)
{

  /* Calls the function systemInit that initialises the code generated from cubeMX*/
  systemInit();
  
   //test for cold or warm start
  RCC_TypeDef rcc;
  bool warmStart=false;
  if(__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)!=RESET){
    __HAL_RCC_CLEAR_RESET_FLAGS();
    //uncomment for debugging
    //HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,1);
//    HAL_Delay(2000);
//    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,0);
    warmStart=true;
  }else{
    __HAL_RCC_CLEAR_RESET_FLAGS();      
  }
  
  /* Create the thread(s) */
  osThreadDef(mainTask, StartMainTask, osPriorityAboveNormal, 1, 128); //128,256,512
  mainTaskHandle = osThreadCreate(osThread(mainTask), (void*)warmStart);

  osThreadDef(pwmInTask, StartPwmInTask, osPriorityNormal, 1, 128);
  pwmInTaskHandle = osThreadCreate(osThread(pwmInTask), NULL);
  
  osThreadDef(analysTask, StartAnalysTask, osPriorityNormal, 1, 128);
  analysTaskHandle = osThreadCreate(osThread(analysTask), NULL);

  //setup start argument for loadHandler - sysmonitor task
  threadInfo* tInfo     = pvPortMalloc(sizeof(threadInfo));
  tInfo->size           = 3;
  tInfo->ID             = pvPortMalloc(sizeof(osThreadId)*3);
  tInfo->tDef           = pvPortMalloc(sizeof(threadDef)*3); 
  tInfo->arg            = (void*)pvPortMalloc(sizeof(void*)*3);
  
  //save thread handles. Take not that the order is the same 
  //as the order they ack the loadHandler thread through gCheckback
  tInfo->ID[0]          = mainTaskHandle;
  tInfo->ID[1]          = pwmInTaskHandle;
  tInfo->ID[2]          = analysTaskHandle;
 
  //copy thread data, note the order
  copy_osThreadDef_UD_t(&(tInfo->tDef[0]),(struct os_thread_def const*)&(os_thread_def_mainTask));
  copy_osThreadDef_UD_t(&(tInfo->tDef[1]),(struct os_thread_def const*)&(os_thread_def_pwmInTask));
  copy_osThreadDef_UD_t(&(tInfo->tDef[2]),(struct os_thread_def const*)&(os_thread_def_analysTask));
  
  
  //copy thread arguments
  tInfo->arg[0]         = NULL;
  tInfo->arg[1]         = NULL;
  tInfo->arg[2]         = NULL;

  
  //osThreadDef(loadTask,loadHandleThread, osPriorityRealtime,1,128);
  //loadTaskHandle = osThreadCreate(osThread(loadTask),(void*)tInfo);

  pwmIn_mailbox = osMailCreate(osMailQ(pwmIn_mailbox), NULL);
  analys_mailbox = osMailCreate(osMailQ(analys_mailbox), NULL);

  
  /* Start scheduler */
  osKernelStart(NULL, NULL);
  
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  while (1)
  {
    /* NO CODE IN HERE! USE THE DEFAULT TASK OR CUSTOM MADE TASKS */
  }
}


#ifdef USE_FULL_ASSERT

void assert_failed(uint8_t* file, uint32_t line)
{
  //TODO ?
}

#endif


/**********END OF FILE****/

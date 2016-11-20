
#include "main.h"

/*External definitions*/
extern TIM_HandleTypeDef htim2;
extern TIM_OC_InitTypeDef sConfigOC;
extern unsigned long start;
extern uint8_t kalman;

uint32_t gMainloopWDCheckback=0;
/* pointers for running avarage filter*/
float rollBuffer[BUFFERSIZE];
float* rollStartPointer = rollBuffer;
float* rollEndPointer = (rollBuffer + BUFFERSIZE-1);
float* rollBufferPointer;

float pitchBuffer[BUFFERSIZE];
float* pitchStartPointer = pitchBuffer;
float* pitchEndPointer = (pitchBuffer + BUFFERSIZE-1);
float* pitchBufferPointer;

void StartMainTask(void const * arguments)
{
  bool warmStart=(bool)arguments;
  start = xTaskGetTickCount();
  /*Declare local variables for the task*/
  extern const portTickType MAIN_FREQUENCY;  
  portTickType  last_task_start = xTaskGetTickCount();
  main_struct *all_values=pvPortMalloc(sizeof(main_struct));
  
  rollBufferPointer = rollStartPointer;
  pitchBufferPointer = pitchStartPointer;
  //Important delay to ESC-device otherwise the motors won't start 
  
  //sConfigOC.Pulse = 1000;
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
  if(!warmStart)osDelay(5000);
  //osDelay(5000);
  
  /* Main loop */
  while(1)
  {
    /* Allocate memory for mailbox */
    getPWMinValues(all_values);
    readSensors(all_values);
    if(kalman){
      kalmanFilter(all_values);
    }else{
      complementaryFilter(all_values);
    }
    automaticControl(all_values);
    
    
    gMainloopWDCheckback++;
    /* Delay the task to a fixed time to ensure constant execution frequncy */
    vTaskDelayUntil(&last_task_start,MAIN_FREQUENCY); 
  }
  
}

void getPWMinValues(main_struct* all_values){
  
  pwmIn_struct* message;
  
  osEvent  evt;
  evt = osMailGet(pwmIn_mailbox, 0);        /*Checks for mail*/
  if (evt.status == osEventMail) {          /*If PWMin have sent mail, save data to all_values*/ 
    message = evt.value.p;
    all_values->pitch.f = message->pitch;
    all_values->roll.f = message->roll;
    all_values->yaw.f = message->yaw;
    all_values->thrust = message->thrust;
    all_values->emergency = message->emergency;
    osMailFree(pwmIn_mailbox, message);     /* Free mailbox */
  }
  
  return;
}
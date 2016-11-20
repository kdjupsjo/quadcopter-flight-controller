/***Includes***/
//#include "analys.h"
#include "main.h"

/**Global variables**/
uint8_t *Tecken;
uint8_t numbers[10];

/**External variables**/
extern UART_HandleTypeDef huart3;
extern const portTickType MAIN_FREQUENCY;
//For test purposes

uint32_t gAnalysWDCheckback=0;
 
void StartAnalysTask(void const * argument){
  portTickType  last_task_start = xTaskGetTickCount();
  while(1){
    
    osEvent check_mail = 
      osMailGet(analys_mailbox, osWaitForever);
    
    main_struct *main_pointer = (main_struct*)check_mail.value.p; 
    
    if(main_pointer != NULL){
    
      //float2int em_temp;
     // em_temp.f = (float) main_pointer->gyro_pitch_rate.f;
    
      
     numbers[0] = (main_pointer->yaw.i[0])+2;
     numbers[1] = (main_pointer->yaw.i[1])+2;
     numbers[2] = (main_pointer->yaw.i[2])+2;
     numbers[3] = (main_pointer->yaw.i[3])+2;
     numbers[4] = 0x00;
     numbers[5] = (main_pointer->filtered_pitch_angle.i[0])+2;
     numbers[6] = (main_pointer->filtered_pitch_angle.i[1])+2;
     numbers[7] = (main_pointer->filtered_pitch_angle.i[2])+2;
     numbers[8] = (main_pointer->filtered_pitch_angle.i[3])+2;
     numbers[9] = 0x01;
      /*
     numbers[0] = (main_pointer->filtered_pitch_angle.i[0])+6;
     numbers[1] = (main_pointer->filtered_pitch_angle.i[1])+6;
     numbers[2] = (main_pointer->filtered_pitch_angle.i[2])+6;
     numbers[3] = (main_pointer->filtered_pitch_angle.i[3])+6;
     numbers[4] = 0x00;
     numbers[5] = (main_pointer->filtered_roll_angle.i[0])+6;
     numbers[6] = (main_pointer->filtered_roll_angle.i[1])+6;
     numbers[7] = (main_pointer->filtered_roll_angle.i[2])+6;
     numbers[8] = (main_pointer->filtered_roll_angle.i[3])+6;
     numbers[9] = 0x01;
     numbers[10] = (main_pointer->pitch.i[0])+6;
     numbers[11] = (main_pointer->pitch.i[1])+6;
     numbers[12] = (main_pointer->pitch.i[2])+6;
     numbers[13] = (main_pointer->pitch.i[3])+6;
     numbers[14] = 0x02;
     numbers[15] = (main_pointer->roll.i[0])+6;
     numbers[16] = (main_pointer->roll.i[1])+6;
     numbers[17] = (main_pointer->roll.i[2])+6;
     numbers[18] = (main_pointer->roll.i[3])+6;
     numbers[19] = 0x03;
     numbers[20] = (thrust_temp.i[0])+6;
     numbers[21] = (thrust_temp.i[1])+6;
     numbers[22] = (thrust_temp.i[2])+6;
     numbers[23] = (thrust_temp.i[3])+6;
     numbers[24] = 0x04;
     numbers[25] = (main_pointer->dtControll.i[0])+6;
     numbers[26] = (main_pointer->dtControll.i[1])+6;
     numbers[27] = (main_pointer->dtControll.i[2])+6;
     numbers[28] = (main_pointer->dtControll.i[3])+6;
     numbers[29] = 0x05;
     */
     
     
    HAL_UART_Transmit(&huart3,(uint8_t*)&numbers, 10, 4);
    huart3.State=HAL_UART_STATE_READY;
    
    osMailFree(analys_mailbox, main_pointer);
    
    }
    gAnalysWDCheckback++;
    vTaskDelayUntil(&last_task_start,MAIN_FREQUENCY); 
  } 
}
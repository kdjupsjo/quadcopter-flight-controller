/*******************************automaticControl.h***************************************/

#ifndef __AUTOMATICCONTROL_H
#define __AUTOMATICCONTROL_H

/* Includes ------------------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
/* Private Variables----------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
void automaticControl(main_struct*);
//void pwmOut(main_struct*);  //Not implemeted
void ChangeVelocityOnMotorsWithPulseWidth (int velocityBasedOnPulseWidth,
                                           int whichMotorToChange);

#endif


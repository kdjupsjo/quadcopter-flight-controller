/*******************************filter.h***************************************/

#ifndef __FILTER_H
#define __FILTER_H

/* Private defines -----------------------------------------------------------*/
#define BUFFERSIZE 4

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
/* Private Variables----------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
void complementaryFilter(main_struct*);
float rollFilter(float);
float pitchFilter(float);
float all(float v[]);
float getAnglePitch(float newAngle, float newRate, float dt);
float getAngleRoll(float newAngle, float newRate, float dt);
void kalmanFilter(main_struct* all_values);


#endif
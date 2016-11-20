#include "main.h"

/*-----------GLOBAL FILTER VARIABLES-----*/
/*Union variables*/
float2int pitch_angle;
float2int gyro_pitch;
float2int gyro_roll;
float2int roll_angle;
float2int roll_integrate;
float2int pitch_integrate;

/* Floats*/
static float last_pitch_angle=0;
static float last_roll_angle=0;
float acc_pitch=0;
float acc_roll=0;
float dt;
unsigned long start;
unsigned long timer;


/*------------KALMAN FILTER VARIABLES---------*/
  float kratePitch;
  float kbiasPitch = 0.0f;
  float kanglePitch = 0.0f;
  float PP[2][2] = {{100.0f, 0.0f},{0.0f, 100.0f}};
  
  float Q_anglePitch = 0.1f;
  float Q_biasPitch = 0.001f;
  float R_measurePitch = 0.1f; 

  float krateRoll;
  float kbiasRoll = 0.0f;
  float kangleRoll = 0.0f;
  float PR[2][2] = {{100.0f, 0.0f},{0.0f, 100.0f}};
  
  float Q_angleRoll = 0.1f;
  float Q_biasRoll = 0.001f;
  float R_measureRoll = 0.1f; 
  
  float running_average_filtered_roll[8] = {0,0,0,0,0,0,0,0};
  float running_average_filtered_pitch[8] = {0,0,0,0,0,0,0,0};
  uint8_t running_index = 0; 
  

/*Simple function to add all variables of a float[8] array. (Kalman)
*/
float all(float v[]){ 
  //if((sizeof(v)/sizeof(float)) != 8) return 0;
  return v[0]+v[1]+v[2]+v[3]+v[4]+v[5]+v[6]+v[7];
}

/* Kalman function */
float getAnglePitch(float newAngle, float newRate, float dt){
 
  

  //Step 1
  kratePitch = newRate - kbiasPitch;
  kanglePitch += dt*kratePitch;
  
  //step 2
    PP[0][0] += dt * (dt*PP[1][1] - PP[0][1] - PP[1][0] + Q_anglePitch);
    PP[0][1] -= dt * PP[1][1];
    PP[1][0] -= dt * PP[1][1];
    PP[1][1] += Q_biasPitch * dt;
  
    //step 3
    
    float y = newAngle - kanglePitch;
    //step 4
    float S = PP[0][0] + R_measurePitch; // Estimate error
    
    //step 5
    float K[2]; // Kalman gain - This is a 2x1 vector
    K[0] = PP[0][0] / S;
    K[1] = PP[1][0] / S;
    
    
    
    kanglePitch += K[0] * y;
    kbiasPitch += K[1] *y;
    
    float P00_temp = PP[0][0];
    float P01_temp = PP[0][1];
    
    PP[0][0] -= K[0] * P00_temp;
    PP[0][1] -= K[0] * P01_temp;
    PP[1][0] -= K[1] * P00_temp;
    PP[1][1] -= K[1] * P01_temp;
    
    return kanglePitch;
  
  
}

/* Kalman function */
float getAngleRoll(float newAngle, float newRate, float dt){
  
  /*
  Observations:
  Högre R tycks hjälpa mot små brus, men ger översväng och långsamt filter.
  Öka Q_angle för snabbare filter.
  Öka Q_bias för minskad drift.
  */

  //Step 1
  krateRoll = newRate - kbiasRoll;
  kangleRoll += dt*krateRoll;
  
  //step 2
    PR[0][0] += dt * (dt*PR[1][1] - PR[0][1] - PR[1][0] + Q_angleRoll);
    PR[0][1] -= dt * PR[1][1];
    PR[1][0] -= dt * PR[1][1];
    PR[1][1] += Q_biasRoll * dt;
  
    //step 3
    
    float y = newAngle - kangleRoll;
    //step 4
    float S = PR[0][0] + R_measureRoll; // Estimate error
    
    //step 5
    float K[2]; // Kalman gain - This is a 2x1 vector
    K[0] = PR[0][0] / S;
    K[1] = PR[1][0] / S;
    
    
    
    kangleRoll += K[0] * y;
    kbiasRoll += K[1] *y;
    
    float P00_temp = PR[0][0];
    float P01_temp = PR[0][1];
    
    PR[0][0] -= K[0] * P00_temp;
    PR[0][1] -= K[0] * P01_temp;
    PR[1][0] -= K[1] * P00_temp;
    PR[1][1] -= K[1] * P01_temp;
    
    return kangleRoll;
  
  
}

void kalmanFilter(main_struct* all_values){
  
  
  dt = (xTaskGetTickCount()-start)/1000.0;
  start = xTaskGetTickCount();
  all_values->dtControll.f= dt;
  
  if(all_values != NULL){
    
    
    /*UART variables*/
    
    
    
    /*Read values from struct*/
    gyro_pitch = all_values->gyro_pitch_rate;
    acc_pitch = all_values->accel_pitch_angle.f;
    gyro_roll = all_values->gyro_roll_rate;
    acc_roll = all_values->accel_roll_angle.f;
    
    /*Set last value for integration*/
    last_pitch_angle=pitch_angle.f;
    last_roll_angle=roll_angle.f;
    
    /*Integrate gyro values from running average filter*/
    pitch_integrate.f = (last_pitch_angle+(gyro_pitch.f*dt));
    roll_integrate.f = (last_roll_angle+(gyro_roll.f*dt));
    
    
    /**********************KALMAN FILTER**********************

    ***************************************************************/

    
      running_average_filtered_pitch[running_index%8] = getAnglePitch(acc_pitch, gyro_pitch.f, dt);
      running_average_filtered_roll[running_index%8] = getAngleRoll(acc_roll, gyro_roll.f, dt);
      running_index++;
      pitch_angle.f = all(running_average_filtered_pitch) / 8.0f;
      roll_angle.f = all(running_average_filtered_roll) / 8.0f;
    
    
    if(all_values != NULL){
      /*Store values in main struct*/
      all_values->filtered_pitch_angle = pitch_angle;
      all_values->filtered_roll_angle = roll_angle;
    }
  }
  return;
}





/*************************************************************************************
*********************************'COMPLEMENTARY FILTER********************************
*************************************************************************************/






void complementaryFilter(main_struct* all_values){
  
  
  dt = (xTaskGetTickCount()-start)/1000.0;
  start = xTaskGetTickCount();
  all_values->dtControll.f= dt;

  
  if(all_values != NULL){

    /*Read values from struct*/
    gyro_pitch = all_values->gyro_pitch_rate;
    acc_pitch = all_values->accel_pitch_angle.f;
    gyro_roll = all_values->gyro_roll_rate;
    acc_roll = all_values->accel_roll_angle.f;
    
    /*Set last value for integration*/
    last_pitch_angle=pitch_angle.f;
    last_roll_angle=roll_angle.f;
    
    /*Integrate gyro values from running average filter*/
    pitch_integrate.f = (last_pitch_angle+(gyro_pitch.f*dt));
    roll_integrate.f = (last_roll_angle+(gyro_roll.f*dt));
    
    
    /**********************COMPLEMENTARRY FILTER**********************
    *This is the actual filtering process.
    *Accelerometer values are lopass filtered and gyro values are highpass
    *filtered, and the values are weighted and added.
    *the values are then stored in a struct and passed on to next task
    *2 lines of code!! AWSOME! =)
    ***************************************************************/
    pitch_angle.f=(0.975*(pitch_integrate.f)+(0.025)*acc_pitch);
    roll_angle.f=(0.975*(roll_integrate.f)+(0.025)*acc_roll);// -0.1;
    
    if(all_values != NULL){
      /*Store values in main struct*/
      all_values->filtered_pitch_angle = pitch_angle;
      all_values->filtered_roll_angle = roll_angle;
    }
  }
  return;
}

/*******************************************************************************
*********************************RUNNING AVARAGE********************************
*******************************************************************************/

/* pointers for running avarage filter*/
extern float rollBuffer[BUFFERSIZE];
extern float* rollStartPointer;
extern float* rollEndPointer;
extern float* rollBufferPointer;

extern float pitchBuffer[BUFFERSIZE];
extern float* pitchStartPointer;
extern float* pitchEndPointer;
extern float* pitchBufferPointer;


float rollFilter(float roll_in){
  float ret;
  *rollBufferPointer = roll_in;
  rollBufferPointer ==  rollEndPointer? rollBufferPointer=rollStartPointer : rollBufferPointer++;
  ret = ((rollBuffer[0]+rollBuffer[1]+rollBuffer[2]+rollBuffer[3])/BUFFERSIZE);
  return ret;
}

float pitchFilter(float pitch_in){
  float ret;
  *pitchBufferPointer = (float)pitch_in;
  pitchBufferPointer ==  pitchEndPointer? pitchBufferPointer=pitchStartPointer : pitchBufferPointer++;
  ret = ((pitchBuffer[0]+pitchBuffer[1]+pitchBuffer[2]+pitchBuffer[3])/BUFFERSIZE);
  return ret;
}
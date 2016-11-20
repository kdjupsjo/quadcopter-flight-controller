
#include "main.h"

//Pid variables
//Control System variables for Roll and Kp, Ki and Kd
float lastErr=0, lastErrGyro=0,errorGyro=0;
float error=0;
float kp=1.5;
float ki=0.2;
float kd=0.1;
float filtered_roll_angle=0;
float desired_roll_angle=0;
float derivate=0;
float integral=0;
float PIDoutput=0;
//Gyro PID for roll
float gyro_error=0;
float gyro_desired=0;;
float gyro_rate=0;
float integralGyro=0;
float derivateGyro=0;
float PIDoutputGyro=0;
/*******************PITCH************/
float lastErrPitch=0, lastErrGyroPitch=0,errorGyroPitch=0;
float errorPitch=0;
float filtered_pitch_angle=0;
float desired_pitch_angle=0;
float derivatePitch=0;
float integralPitch=0;
float PIDoutputPitch=0;
//Gyro pitch
float gyro_errorPitch=0;
float gyro_desiredPitch=0;;
float gyro_ratePitch=0;
float integralGyroPitch=0;
float derivateGyroPitch=0;
float PIDoutputGyroPitch=0;
//********************Yaw*****************
float lastErrYaw=0, lastErrGyroYaw=0,errorGyroYaw=0;
float errorYaw=0;
float filtered_yaw_angle=0;
float desired_yaw_angle=0;
float derivateYaw=0;
float integralYaw=0;
float PIDoutputYaw=0;
float gyro_errorYaw=0;
float gyro_desiredYaw=0;;
float gyro_rateYaw=0;
float integralGyroYaw=0;
float derivateGyroYaw=0;
float PIDoutputGyroYaw=0;
/*****************Motors**********************/
int ThrustOnMotor=0;
float lastinput=0;
int RFmotor=0;
int LFmotor=0;
int RBmotor=0;
int LBmotor=0;
float toLowIntegrate=0.1;

/*External definitions*/
extern TIM_OC_InitTypeDef sConfigOC;
extern TIM_HandleTypeDef htim2;
extern float dt;
//Running average
float gyroRunning[8];
int gyroIndex = 0;
float gyroRunningPitch[8];
int gyroIndexPitch = 0;
float gyroRunningYaw[8];
int gyroIndexYaw = 0;

void automaticControl(main_struct* all_values){
  
  osEvent check_mail = 
    osMailGet(pwmIn_mailbox, osWaitForever);
  
  pwmIn_struct *pwm_pointer = (pwmIn_struct*)check_mail.value.p; 
  /****************************Yaw*********************************************/
  /*Regulation on Yaw*/
  gyroRunningYaw[gyroIndexYaw] = all_values->gyro_yaw_rate.f;
  gyro_rateYaw = 0;
  for(int i = 0; i < 8; i++){
    gyro_rateYaw += gyroRunningYaw[i];
  }
  gyro_rateYaw /= 8;
  gyroIndexYaw = (gyroIndexYaw + 1) %8;
  
  //Gyro
  errorGyroYaw=desired_yaw_angle - gyro_rateYaw; //get the error
  //check if the error is to small to integrate
  if(abs(errorGyroYaw)> toLowIntegrate)
  {
  integralGyroYaw += (errorGyroYaw*dt); //get the integral term
  }
  derivateGyroYaw = (errorGyroYaw - lastErrGyroYaw)/dt;  //get the derivate term
  PIDoutputGyroYaw=(kp*errorGyroYaw + ki*integralGyroYaw + kd*derivateGyroYaw)/4;
  /******************************Pitch*****************************************/
  
  desired_pitch_angle=pwm_pointer->pitch;  //set point
  filtered_pitch_angle = all_values->filtered_pitch_angle.f; //angle from sensor
  errorPitch=desired_pitch_angle-filtered_pitch_angle; //error
  //check if the error is to small to integrate
  if(abs(errorPitch)> toLowIntegrate)
  {
  integralPitch += errorPitch*dt; //I-term
  }
  derivatePitch = (errorPitch - lastErrPitch)/dt; //D-term
  PIDoutputPitch= (kp*errorPitch + ki*integralPitch + kd*derivatePitch)/2; //control signal
  
  //running average on gyro
  gyroRunningPitch[gyroIndexPitch] = all_values->gyro_pitch_rate.f; 
  gyro_ratePitch = 0;
  for(int i = 0; i < 8; i++){
    gyro_ratePitch += gyroRunningPitch[i];
  }
  gyro_ratePitch /= 8;
  gyroIndexPitch = (gyroIndexPitch + 1) %8;
  
  //Gyro
  errorGyroPitch=gyro_desiredPitch - gyro_ratePitch; //error
  //check if the error is to small to integrate
  if(abs(errorGyroPitch)> toLowIntegrate)
  {
  integralGyroPitch += (errorGyroPitch*dt); //I-term
  }
  derivateGyroPitch = (errorGyroPitch - lastErrGyroPitch)/dt; //D-term
  PIDoutputGyroPitch=(kp*errorGyroPitch + ki*integralGyroPitch + kd*derivateGyroPitch)/2; //control signal
  
  /*********************************Roll***************************************/
 
  desired_roll_angle=pwm_pointer->roll; //set point
  filtered_roll_angle = all_values->filtered_roll_angle.f; //angle from sensor
  error=desired_roll_angle-filtered_roll_angle; // error
  //check if the error is to small to integrate
  if(abs(error)> toLowIntegrate)
  {
  integral =integral+ error*dt; //I-term
  }
  derivate= (error - lastErr)/dt; //D-term
  PIDoutput=(kp*error + ki*integral + kd*derivate)/2; //control signal
  
  //Running average on gyro
  gyroRunning[gyroIndex] = all_values->gyro_roll_rate.f;
  gyro_rate = 0;
  for(int i = 0; i < 8; i++){
    gyro_rate += gyroRunning[i];
  }
  gyro_rate /= 8;
  gyroIndex = (gyroIndex + 1) %8;
  
  //Gyro Error, I-term, D-term and I term, control signal = PIDoutputGyro
  errorGyro=gyro_desired - gyro_rate; //the error
  //check if the error is to small to integrate
  if(abs(errorGyro)> toLowIntegrate)
  {
  integralGyro += (errorGyro*dt); //I-term
  }
  derivateGyro = (errorGyro - lastErrGyro)/dt; //D-term
  PIDoutputGyro=(kp*errorGyro + ki*integralGyro + kd*derivateGyro)/2; //control signal
  /****************************************************************************/
  ThrustOnMotor = pwm_pointer->thrust; 
  RFmotor = /*(int)(PIDoutputGyroYaw +*/ (int) (ThrustOnMotor + (int)PIDoutput-(int)PIDoutputGyro-(int)PIDoutputPitch+(int)PIDoutputGyroPitch);//Not exact, should be float
  LFmotor = /*(int)(PIDoutputGyroYaw - */(int) (ThrustOnMotor - (int)PIDoutput+(int)PIDoutputGyro-(int)PIDoutputPitch+(int)PIDoutputGyroPitch);//type cast from float to int
  RBmotor = /*(int)(PIDoutputGyroYaw - */(int) (ThrustOnMotor + (int)PIDoutput-(int)PIDoutputGyro+(int)PIDoutputPitch-(int)PIDoutputGyroPitch);//Not exact, should be float
  LBmotor = /*(int)(PIDoutputGyroYaw + */(int) (ThrustOnMotor - (int)PIDoutput+(int)PIDoutputGyro+(int)PIDoutputPitch-(int)PIDoutputGyroPitch);//type cast from float to int
  lastErr=error;
  lastErrGyro=errorGyro;
  lastErrPitch=errorPitch;
  lastErrGyroPitch=errorGyroPitch;
  lastErrYaw=errorYaw;
  lastErrGyroYaw=errorGyroYaw;
  //Saftey, if controller is in emergency stop mode, or thrust is at 0
  //disregard PID.
  if((all_values->emergency != 0) || (all_values->thrust < 1100)){
    RFmotor = 1000;
    LFmotor = 1000;
    RBmotor = 1000;
    LBmotor = 1000;
    // Yaw
    integralGyroYaw=0;
    derivateGyroYaw=0;
    //Pitch
    integralGyroPitch=0;
    derivateGyroPitch=0;
    integralPitch=0;
    derivatePitch=0;
    //Roll
    integral = 0;
    integralGyro=0;
    derivateGyro=0;
    derivate = 0;
  }
  
  
  //Right forward motor
  ChangeVelocityOnMotorsWithPulseWidth(RFmotor,1);
  //Left forward motor
  ChangeVelocityOnMotorsWithPulseWidth(LFmotor,3);
  //Right back motor
  ChangeVelocityOnMotorsWithPulseWidth(RBmotor,2);
  //Left back motor
  ChangeVelocityOnMotorsWithPulseWidth(LBmotor,4);
  
  
  all_values->thrust = pwm_pointer->thrust;
  all_values->yaw.f = pwm_pointer->yaw;
  all_values->roll.f = pwm_pointer->roll;
  all_values->pitch.f = pwm_pointer->pitch;
  
  osMailPut(analys_mailbox, all_values);
  
  osMailFree(pwmIn_mailbox, pwm_pointer);

  
}


/******************************************************************************
*Input values: PulseWidth Value that changes the speed of the the propeller 
and which motor we want to change that speed to
*Hardware setup: NONE
*General describtion: Takes an input and changes the Pulse Width to input. Then
we change to the new value and start PWM again to the selected motor.
*
*******************************************************************************/
void ChangeVelocityOnMotorsWithPulseWidth (int velocityBasedOnPulseWidth,
                                           int whichMotorToChange)
{
  sConfigOC.Pulse = velocityBasedOnPulseWidth;
    if(velocityBasedOnPulseWidth > 2000)
  {
  velocityBasedOnPulseWidth = 2000;
  }
  else if(velocityBasedOnPulseWidth < 1200)
  {
  velocityBasedOnPulseWidth= 1200;
  }
  
  switch (whichMotorToChange) 
  {
  case 1 :
    HAL_TIM_PWM_ConfigChannel(&htim2,&sConfigOC,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
    break;
    
  case 2 :
    HAL_TIM_PWM_ConfigChannel(&htim2,&sConfigOC,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
    break;
    
  case 3 :
    HAL_TIM_PWM_ConfigChannel(&htim2,&sConfigOC,TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
    break;
    
  case 4 :
    HAL_TIM_PWM_ConfigChannel(&htim2,&sConfigOC,TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
    break;
  default :
    break;
  }
}

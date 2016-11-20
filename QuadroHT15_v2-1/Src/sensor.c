/**
******************************************************************************
* @file    sensor.c
* @author  Jacob Kimblad / Max Kufa
* @version V1.1
* @date    10-12-2015
* @brief   File to run a thread which collects sensordata 
******************************************************************************
*/
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/*External definitions*/
extern SPI_HandleTypeDef hspi2;
extern uint8_t iNemo;

int read_data(int address, int type);
void write_data(int address, int data, int type);

/*
* Name: readSensors
* Desc: Writes values from sensors to all_values struct
* Args: main_struct* all_values
* Rets: Null
*/
void readSensors(main_struct* all_values){
  
  if(all_values != NULL){
    
    /** Uncomment these to use lis3dh/l3dg20h
    
    */
    
    if(iNemo){
      all_values->accel_pitch_angle.f = accelerometer_Get_Pitch(get_ACC_X_value(), get_ACC_Y_value(), get_ACC_Z_value());
      all_values->accel_roll_angle.f = accelerometer_Get_Roll(get_ACC_X_value(), get_ACC_Y_value(), get_ACC_Z_value());
      
      all_values->gyro_pitch_rate.f = (float)get_GYRO_X_value();
      all_values->gyro_roll_rate.f = (float)get_GYRO_Y_value();
      all_values->gyro_yaw_rate.f = (float)get_GYRO_Z_value();
    }else{
      all_values->accel_pitch_angle.f = accelerometer_Get_Pitch(lis3dh_Read_X(), lis3dh_Read_Y(), lis3dh_Read_Z());
      all_values->accel_roll_angle.f = accelerometer_Get_Roll(lis3dh_Read_X(), lis3dh_Read_Y(), lis3dh_Read_Z());
      
      all_values->gyro_pitch_rate.f = (float)l3gd20h_Read_X();
      all_values->gyro_roll_rate.f = (float)l3gd20h_Read_Y();
      all_values->gyro_yaw_rate.f = (float)l3gd20h_Read_Z();
      
    }
  }
  return;
}

/*
* Name: l3gd20h_Read_X
* Desc: Reads the data from the gyro sensor and returns the current 
rotationspeed on the X axis.
* Args: null
* Rets: The rotationspeed as a 16 bit integer
*/
float l3gd20h_Read_X(void)
{
  //Function variables
  int16_t x_Value = 0;
  uint8_t address;
  uint8_t received[2];
  
  //Read from x_low register
  address = 0xA8;
  HAL_GPIO_WritePin(GPIOB, cs_gyro_Pin, GPIO_PIN_RESET); //Start SPI transmission
  HAL_SPI_TransmitReceive(&hspi2, &address, (uint8_t *)received, 2, 1000); 
  HAL_GPIO_WritePin(GPIOB, cs_gyro_Pin, GPIO_PIN_SET); //End SPI transmission
  x_Value = received[1];
  x_Value = x_Value*256; //recieved bit are not correctly aligned. Shift them 8 bits.
  
  //Read from x_high register
  address = 0xA9;
  HAL_GPIO_WritePin(GPIOB, cs_gyro_Pin, GPIO_PIN_RESET);//Start SPI transmission
  HAL_SPI_TransmitReceive(&hspi2, &address, (uint8_t *)received, 2, 1000); 
  HAL_GPIO_WritePin(GPIOB, cs_gyro_Pin, GPIO_PIN_SET);//End SPI transmission
  //Add remaining 8 bits into the 16 bit int and return
  return (x_Value | received[1])*70/1000.0;
}


/*
* Name: l3gd20h_Read_Y
* Desc: Reads the data from the gyro sensor and returns the current rotation 
*       speed on the Y axis.
* Args: null
* Rets: The rotationspeed as a 16 bit integer
*/
float l3gd20h_Read_Y(void)
{
  //Function variables
  int16_t y_Value = 0;
  uint8_t address;
  uint8_t received[2];
  
  //Read from y_l register
  address = 0xAA;
  HAL_GPIO_WritePin(GPIOB, cs_gyro_Pin, GPIO_PIN_RESET); //Start SPI transmission
  HAL_SPI_TransmitReceive(&hspi2, &address, (uint8_t *)received, 2, 1000); 
  HAL_GPIO_WritePin(GPIOB, cs_gyro_Pin, GPIO_PIN_SET); //End SPI transmission
  y_Value = received[1];
  y_Value = y_Value*256;         //Shift 8 bits
  
  //Read from y_h register
  address = 0xAB;
  HAL_GPIO_WritePin(GPIOB, cs_gyro_Pin, GPIO_PIN_RESET); //Start SPI transmission
  HAL_SPI_TransmitReceive(&hspi2, &address, (uint8_t *)received, 2, 1000); 
  HAL_GPIO_WritePin(GPIOB, cs_gyro_Pin, GPIO_PIN_SET); //End SPI transmission
  //Add remaining 8 bits into the 16 bit int and return
  return (y_Value | received[1])*70/1000.0;
}



/*
* Name: l3gd20h_Read_Z
* Desc: Reads the data from the gyro sensor and returns the current 
rotationspeed on the Z axis.
* Args: null
* Rets: The rotationspeed as a 16 bit integer
*/
float l3gd20h_Read_Z(void)
{
  //Function variables
  int16_t z_Value = 0;
  uint8_t address;
  uint8_t received[2];
  
  //Read from z_l register
  address = 0xAC;
  HAL_GPIO_WritePin(GPIOB, cs_gyro_Pin, GPIO_PIN_RESET); //Start SPI transmission
  HAL_SPI_TransmitReceive(&hspi2, &address, (uint8_t *)received, 2, 1000); 
  HAL_GPIO_WritePin(GPIOB, cs_gyro_Pin, GPIO_PIN_SET); //End SPI transmission
  z_Value = received[1];
  z_Value = z_Value*256;         //Shift 8 bits
  
  //Read from z_h register
  address = 0xAD;
  HAL_GPIO_WritePin(GPIOB, cs_gyro_Pin, GPIO_PIN_RESET); //Start SPI transmission
  HAL_SPI_TransmitReceive(&hspi2, &address, (uint8_t *)received, 2, 1000); 
  HAL_GPIO_WritePin(GPIOB, cs_gyro_Pin, GPIO_PIN_SET); //End SPI transmission
  //Add remaining 8 bits into the 16 bit int and return
  return (z_Value | received[1])*70/1000.0;
}



/*
* Name: lis3dh_Read_X
* Desc: Reads the accelerometer and returns the force in the X direction
* Args: null
* Rets: The strength of the force in the X direction.
*/
float lis3dh_Read_X(void)
{
  int16_t x_Value = 0;
  uint8_t address;
  uint8_t received[2];
  
  //Read from x_l register
  address = 0xA8;
  HAL_GPIO_WritePin(GPIOB, cs_acc_Pin, GPIO_PIN_RESET); //Start SPI transmission
  HAL_SPI_TransmitReceive(&hspi2, &address, (uint8_t *)received, 2, 1000); 
  HAL_GPIO_WritePin(GPIOB, cs_acc_Pin, GPIO_PIN_SET); //End SPI transmission
  x_Value = received[1];
  x_Value = x_Value*256;         //Shift left 8 bits into 16bit int 
  
  //Read from x_h register
  address = 0xA9;
  HAL_GPIO_WritePin(GPIOB, cs_acc_Pin, GPIO_PIN_RESET); //Start SPI transmission
  HAL_SPI_TransmitReceive(&hspi2, &address, (uint8_t *)received, 2, 1000); 
  HAL_GPIO_WritePin(GPIOB, cs_acc_Pin, GPIO_PIN_SET); //End SPI transmission
  //Add remaining 8 bits into the 16 bit int and return
  return (x_Value | received[1])/16.384;       
}

/*
* Name: lis3dh_Read_Y
* Desc: Reads the accelerometer and returns the force in the Y direction
* Args: null
* Rets: The strength of the force in the Y direction.
*/
float lis3dh_Read_Y(void)
{
  int16_t y_Value = 0;
  uint8_t address;
  uint8_t received[2];
  
  //GET y_l
  address = 0xAA;
  HAL_GPIO_WritePin(GPIOB, cs_acc_Pin, GPIO_PIN_RESET); //Start SPI transmission
  HAL_SPI_TransmitReceive(&hspi2, &address, (uint8_t *)received, 2, 1000); 
  HAL_GPIO_WritePin(GPIOB, cs_acc_Pin, GPIO_PIN_SET); //Start End transmission
  y_Value = received[1];
  y_Value = y_Value*256;         //Shift 8 bits
  
  //GET y_h
  address = 0xAB;
  HAL_GPIO_WritePin(GPIOB, cs_acc_Pin, GPIO_PIN_RESET); //Start SPI transmission
  HAL_SPI_TransmitReceive(&hspi2, &address, (uint8_t *)received, 2, 1000); 
  HAL_GPIO_WritePin(GPIOB, cs_acc_Pin, GPIO_PIN_SET); //End SPI transmission
  //Add remaining 8 bits into the 16 bit int and return
  return (y_Value | received[1])/16.384;
}

/*
* Name: lis3dh_Read_Z
* Desc: Reads the accelerometer and returns the force in the Z direction
* Args: null
* Rets: The strength of the force in the Z direction.
*/
float lis3dh_Read_Z(void)
{
  int16_t z_Value = 0;
  uint8_t address;
  uint8_t received[2];
  
  //GET z_l
  address = 0xAC;
  HAL_GPIO_WritePin(GPIOB, cs_acc_Pin, GPIO_PIN_RESET); //Start SPI transmission
  HAL_SPI_TransmitReceive(&hspi2, &address, (uint8_t *)received, 2, 1000); 
  HAL_GPIO_WritePin(GPIOB, cs_acc_Pin, GPIO_PIN_SET); //End SPI transmission
  z_Value = received[1];
  z_Value = z_Value*256;         //Shift 8 bits
  
  //GET z_h
  address = 0xAD;
  HAL_GPIO_WritePin(GPIOB, cs_acc_Pin, GPIO_PIN_RESET); //Start SPI transmission
  HAL_SPI_TransmitReceive(&hspi2, &address, (uint8_t *)received, 2, 1000); 
  HAL_GPIO_WritePin(GPIOB, cs_acc_Pin, GPIO_PIN_SET); //End SPI transmission
  //Add remaining 8 bits into the 16 bit int and return
  return (z_Value | received[1])/16.384;
}

/*
* Name: accelerometer_Get_Pitch
* Desc: Transform the forces in all the axises to degrees in pitch 
* Args: The axis values.
* Rets: the current pitch as a 8 bit integer.
*/
float accelerometer_Get_Pitch(float x_Acceleration, float y_Acceleration, float z_Acceleration)
{
  //Function variables
  float pitch;
  //Mathematical calculation for PITCH  
  pitch = atan(x_Acceleration/sqrt(pow(y_Acceleration,2) + pow(z_Acceleration,2)));
  
  //Turn into degrees
  return pitch*(180.0/M_PI);
}

/*
* Name: accelerometer_Get_Roll
* Desc: Transform the forces in all the axises to degrees in roll 
* Args: The axis values.
* Rets: the current roll as a 8 bit integer.
*/
float accelerometer_Get_Roll(float x_Acceleration, float y_Acceleration, float z_Acceleration)
{
  //Function variables
  float roll;
  //Mathematical calculation for ROLL
  roll = atan(y_Acceleration/sqrt(pow(x_Acceleration,2) + pow(z_Acceleration,2)));
  
  //Turn into degrees
  return roll*(180.0/M_PI);
}




/*----------------------------------------
get ACC X value
Version: 1.0 (2015-12-08)
Author: Emil Netz
Description: Read if new acc value then get x, y and z value and return 16 bit X value.
If return value = 0 that means that no new value is detected. OBS
Args: void
Return: 16 bit value. XXXX XXXX XXXX XXXX
----------------------------------------*/
float get_ACC_X_value (void)
{
  int acc_data_ready = read_data(0x27, ACC);          //ACC - reads accelerometer status
  int new_data_acc = acc_data_ready & 8;//ACC Checks if bit number 5 is 1 or 0 to determine if new data from the accelerometer is available
  float return_value = 0;
  
  //if(new_data_acc == 8) // Retrieves x,y and z- values from accelerometer if new data is available
  //{
  uint8_t acc_low_x =  read_data(0x28, ACC);
  uint8_t acc_high_x = read_data(0x29, ACC);
  
  int16_t acc_x = acc_high_x * 256;
  acc_x = acc_x | acc_low_x;
  return_value = acc_x * 0.732;
  //}
  
  return return_value;
}

/*----------------------------------------
get ACC Y value
Version: 1.0 (2015-12-08)
Author: Emil Netz
Description: Read if new acc value then get x, y and z value and return 16 bit Y value.
If return value = 0 that means that no new value is detected. OBS
Args: void
Return: 16 bit value. YYYY YYYY YYYY YYYY
----------------------------------------*/
float get_ACC_Y_value (void)
{
  int acc_data_ready = read_data(0x27, ACC);          //ACC - reads accelerometer status
  int new_data_acc = acc_data_ready & 8;//ACC Checks if bit number 5 is 1 or 0 to determine if new data from the accelerometer is available
  float return_value = 0;
  //if(new_data_acc == 8) // Retrieves x,y and z- values from accelerometer if new data is available
  //{
  uint8_t acc_low_y =  read_data(0x2A, ACC);
  uint8_t acc_high_y = read_data(0x2B, ACC);
  
  int16_t acc_y = acc_high_y * 256;
  acc_y = acc_y | acc_low_y;
  return_value = acc_y * 0.732;
  //}
  return return_value;
}

/*----------------------------------------
get ACC Z value
Version: 1.0 (2015-12-08)
Author: Emil Netz
Description: Read if new acc value then get x, y and z value and return 16 bit Z value.
If return value = 0 that means that no new value is detected. OBS
Args: void
Return: 16 bit value. ZZZZ ZZZZ ZZZZ ZZZZ
----------------------------------------*/
float get_ACC_Z_value (void)
{
  int acc_data_ready = read_data(0x27, ACC);          //ACC - reads accelerometer status
  int new_data_acc = acc_data_ready & 8;//ACC Checks if bit number 5 is 1 or 0 to determine if new data from the accelerometer is available
  float return_value = 0;
  //if(new_data_acc == 8) // Retrieves x,y and z- values from accelerometer if new data is available
  //{
  uint8_t acc_low_z =  read_data(0x2C, ACC);
  uint8_t acc_high_z = read_data(0x2D, ACC);
  
  int16_t acc_z = acc_high_z * 256;
  acc_z = acc_z | acc_low_z;
  return_value = (acc_z * 0.732) + 100;
  //}
  return return_value;
}

/*----------------------------------------
get GYRO X value
Version: 1.0 (2015-12-08)
Author: Emil Netz
Description: Read if new acc value then get x, y and z value and return 16 bit X value.
If return value = 0 that means that no new value is detected. OBS
Args: void
Return: 16 bit value. XXXX XXXX XXXX XXXX
----------------------------------------*/
float get_GYRO_X_value (void)
{
  int gyro_data_ready = read_data(0x27, GYRO);          //ACC - reads accelerometer status
  int new_data_gyro = gyro_data_ready & 8;//ACC Checks if bit number 5 is 1 or 0 to determine if new data from the accelerometer is available
  float return_value = 0;
  if(new_data_gyro == 8) // Retrieves x,y and z- values from accelerometer if new data is available
  {
    uint8_t gyro_low_x =  read_data(0x28, GYRO);
    uint8_t gyro_high_x = read_data(0x29, GYRO);
    
    int16_t gyro_x = gyro_high_x * 256;
    gyro_x = gyro_x | gyro_low_x;
    return_value = (gyro_x * 0.07)-113;
  }
  return return_value;
}

/*----------------------------------------
get GYRO Y value
Version: 1.0 (2015-12-08)
Author: Emil Netz
Description: Read if new acc value then get x, y and z value and return 16 bit Y value.
If return value = 0 that means that no new value is detected. OBS
Args: void
Return: 16 bit value. YYYY YYYY YYYY YYYY
----------------------------------------*/
float get_GYRO_Y_value (void)
{
  int gyro_data_ready = read_data(0x27, GYRO);          //ACC - reads accelerometer status
  int new_data_gyro = gyro_data_ready & 8;//ACC Checks if bit number 5 is 1 or 0 to determine if new data from the accelerometer is available
  float return_value = 0;
  if(new_data_gyro == 8) // Retrieves x,y and z- values from accelerometer if new data is available
  {
    uint8_t gyro_low_y =  read_data(0x2A, GYRO);
    uint8_t gyro_high_y = read_data(0x2B, GYRO);
    
    int16_t gyro_y = gyro_high_y * 256;
    gyro_y = gyro_y | gyro_low_y;
    return_value = (gyro_y * 0.07)+111;
  }
  return return_value;
}

/*----------------------------------------
get GYRO Z value
Version: 1.0 (2015-12-08)
Author: Emil Netz
Description: Read if new acc value then get x, y and z value and return 16 bit Z value.
If return value = 0 that means that no new value is detected. OBS
Args: void
Return: 16 bit value. ZZZZ ZZZZ ZZZZ ZZZZ
----------------------------------------*/
float get_GYRO_Z_value (void)
{
  int gyro_data_ready = read_data(0x27, GYRO);          //ACC - reads accelerometer status
  int new_data_gyro = gyro_data_ready & 8;//ACC Checks if bit number 5 is 1 or 0 to determine if new data from the accelerometer is available
  float return_value = 0;
  if(new_data_gyro == 8) // Retrieves x,y and z- values from accelerometer if new data is available
  {
    uint8_t gyro_low_z =  read_data(0x2C, GYRO);
    uint8_t gyro_high_z = read_data(0x2D, GYRO);
    
    int16_t gyro_z = gyro_high_z * 256;
    gyro_z = gyro_z | gyro_low_z;
    return_value = (gyro_z * 0.07)+111;
  }
  return return_value;
}
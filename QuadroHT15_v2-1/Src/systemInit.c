/*
Name of functions:

1.      SystemClock_Config      # Starts the system clock
2.      MX_SPI2_Init            # SPI2 init function
3.      MX_TIM1_Init            # TIM1 init function
4.      MX_TIM2_Init            # TIM2 init function
5.      MX_TIM3_Init            # TIM3 init function
6.      MX_TIM4_Init            # TIM4 init function
7.      MX_TIM8_Init            # TIM8 init function
8.      MX_USART2_UART_Init     # USART2 init function
9.      MX_USART3_UART_Init     # USART3 init function
10.     MX_USB_PCD_Init         # USB init function
11.     MX_GPIO_Init            # Configure pins as: 
                                  * Analog 
                                  * Input 
                                  * Output
                                  * EVENT_OUT
                                  * EXTI

12.     systemInit              # The function systemInit initiates the code 
                                  generated from cubeMX in a seperate function




*/

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Defines -------------------------------------------------------------------*/

#define GYRO 0
#define ACC 1

int read_data(int address, int type);
void write_data(int address, int data, int type);

extern uint8_t iNemo;
  
/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_FS;

TIM_OC_InitTypeDef sConfigOC;

/**************************************************************************
(1)     SystemClock_Config
**************************************************************************/
/* System Clock Configuration */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_TIM1
                              |RCC_PERIPHCLK_TIM8;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.USBClockSelection = RCC_USBPLLCLK_DIV1_5;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Tim8ClockSelection = RCC_TIM8CLK_HCLK;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}


/**************************************************************************
(2)     MX_SPI2_Init
**************************************************************************/
/* SPI2 init function */
void MX_SPI2_Init(void)
{

  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLED;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLED;
  HAL_SPI_Init(&hspi2);

}


/**************************************************************************
(3)     MX_TIM1_Init
**************************************************************************/
/* TIM1 init function */
void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  HAL_TIM_Base_Init(&htim1);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);

}

/**************************************************************************
(4)      MX_TIM2_Init
**************************************************************************/
/* TIM2 init function */
void MX_TIM2_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig;


  SystemCoreClock;
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_PWM_Init(&htim2);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 700;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  //Sets channel to the PWM-signal
  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);

  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2);

  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3);

  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4);  
}

/**************************************************************************
(5)     MX_TIM3_Init
**************************************************************************/
/* TIM3 init function */
void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 17;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xFFFF;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim3);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);

  HAL_TIM_IC_Init(&htim3);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1);

  HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2);

  HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_3);

  HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_4);

}
  
/**************************************************************************
(6)     MX_TIM4_Init
**************************************************************************/ 
  /* TIM4 init function */
void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 17;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0xFFFF;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim4);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig);

  HAL_TIM_IC_Init(&htim4);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig);

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1);

  HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_2);

  HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_3);

  HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_4);

}
  
/**************************************************************************
(7)     MX_TIM8_Init
**************************************************************************/
  /* TIM8 init function */
void MX_TIM8_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 0;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  HAL_TIM_Base_Init(&htim8);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig);

  HAL_TIM_IC_Init(&htim8);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig);

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  HAL_TIM_IC_ConfigChannel(&htim8, &sConfigIC, TIM_CHANNEL_1);

}
  
/**************************************************************************
(8)     MX_USART2_UART_Init
**************************************************************************/  
 /* USART2 init function */
void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONEBIT_SAMPLING_DISABLED ;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(&huart2);

} 

/**************************************************************************
(9)     MX_USART3_UART_Init
**************************************************************************/ 
/* USART3 init function */
void MX_USART3_UART_Init(void)
{


  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONEBIT_SAMPLING_DISABLED ;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(&huart3);
  

}  
  
/**************************************************************************
(10)     MX_USB_PCD_Init
**************************************************************************/
 /* USB init function */
void MX_USB_PCD_Init(void)
{

  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.ep0_mps = DEP0CTL_MPS_64;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  HAL_PCD_Init(&hpcd_USB_FS);

} 
  
  
/**************************************************************************
(11)      MX_GPIO_Init
**************************************************************************/    
/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOC_CLK_ENABLE();
  __GPIOF_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : INT2_XM_Pin INT_G_Pin DRDY_G_Pin INT1_XM_Pin */
  GPIO_InitStruct.Pin = INT2_XM_Pin|INT_G_Pin|DRDY_G_Pin|INT1_XM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin LED4_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_ACC_MAGN_Pin CS_GYRO_Pin CS_BARO_Pin DEN_GYRO_Pin */
  GPIO_InitStruct.Pin = CS_ACC_MAGN_Pin|CS_GYRO_Pin|CS_BARO_Pin|DEN_GYRO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/*
* Name: lis3dh_init
* Desc: Initiates the accelerometer controlregisters.
        This initiation is specific to the project and should not be changed.
* Args: null
* Rets: null
*/
void lis3dh_Init(void)
{
  //lis3dh init variables
  uint8_t register_Value;       //Used to store the information sent to the register     
  uint8_t reg_Address;          //Used to store address of used register and rw and ms bit
  uint8_t status_Reg[2] = {0x00, 0x00};
  
  /*
  To start an SPI transmission, a specific pin must be set to 0.
  The MCU has the problem with having these pins set as 0 as default.
  This can confuse the sensor during the first transmission.
  To solve this, we start a transmission and try to read a specific register
  that always contain the same value.
  We know that the confusion has subsided when we receive the correct value
  from the sensor.
  */
  reg_Address = 0x8F;
  while(status_Reg[1] != 0x33)
  {
    HAL_GPIO_WritePin(GPIOB, cs_acc_Pin, GPIO_PIN_RESET); //Start SPI transmission
    HAL_SPI_TransmitReceive(&hspi2, &reg_Address, (uint8_t *)status_Reg, 2, 1000); 
    HAL_GPIO_WritePin(GPIOB, cs_acc_Pin, GPIO_PIN_SET); //End SPI transmission
   // printf("%i\n", status_Reg[1]);
  }
  
  
  //CTRL_REG1
  reg_Address = 0x20;                      //rw:0, ms:0 adress: 0x20
  register_Value = 0x57;                   //0111 0111
  //Activate accelerometer SPI
  HAL_GPIO_WritePin(GPIOB, cs_acc_Pin, GPIO_PIN_RESET);
  //Send data
  HAL_SPI_Transmit(&hspi2, &reg_Address, 1, 1000);
  HAL_SPI_Transmit(&hspi2, &register_Value, 1, 1000);
  //Deactivate accelerometer SPI
  HAL_GPIO_WritePin(GPIOB, cs_acc_Pin, GPIO_PIN_SET);
    
  //CTRL_REG2
  reg_Address = 0x21;                      //rw:0, ms:0 adress: 0x21
  register_Value = 0x00;                   //0000 0000
  //Activate accelerometer SPI
  HAL_GPIO_WritePin(GPIOB, cs_acc_Pin, GPIO_PIN_RESET);
  //Send data
  HAL_SPI_Transmit(&hspi2, &reg_Address, 1, 1000);
  HAL_SPI_Transmit(&hspi2, &register_Value, 1, 1000);
  //Deactivate accelerometer SPI
  HAL_GPIO_WritePin(GPIOB, cs_acc_Pin, GPIO_PIN_SET);
  
  //CTRL_REG3
  reg_Address = 0x22;                      //rw:0, ms:0 adress: 0x22
  register_Value = 0x00;                   //0000 0000
  //Activate accelerometer SPI
  HAL_GPIO_WritePin(GPIOB, cs_acc_Pin, GPIO_PIN_RESET);
  //Send data
  HAL_SPI_Transmit(&hspi2, &reg_Address, 1, 1000);
  HAL_SPI_Transmit(&hspi2, &register_Value, 1, 1000);
  //Deactivate accelerometer SPI
  HAL_GPIO_WritePin(GPIOB, cs_acc_Pin, GPIO_PIN_SET);
  
  //CTRL_REG4
  reg_Address = 0x23;                      //rw:0, ms:0 adress: 0x23
  register_Value = 0x48;                   //0110 1000
  //Activate accelerometer SPI
  HAL_GPIO_WritePin(GPIOB, cs_acc_Pin, GPIO_PIN_RESET);
  //Send data
  HAL_SPI_Transmit(&hspi2, &reg_Address, 1, 1000);
  HAL_SPI_Transmit(&hspi2, &register_Value, 1, 1000);
  //Deactivate accelerometer SPI
  HAL_GPIO_WritePin(GPIOB, cs_acc_Pin, GPIO_PIN_SET);
  
  //CTRL_REG5
  reg_Address = 0x24;                      //rw:0, ms:0 adress: 0x24
  register_Value = 0x00;                   //1000 0000
  //Activate accelerometer SPI
  HAL_GPIO_WritePin(GPIOB, cs_acc_Pin, GPIO_PIN_RESET);
  //Send data
  HAL_SPI_Transmit(&hspi2, &reg_Address, 1, 1000);
  HAL_SPI_Transmit(&hspi2, &register_Value, 1, 1000);
  //Deactivate accelerometer SPI
  HAL_GPIO_WritePin(GPIOB, cs_acc_Pin, GPIO_PIN_SET);

  
  //CTRL_REG6
  reg_Address = 0x25;                      //rw:0, ms:0 adress: 0x25
  register_Value = 0x00;                   //0000 0000
  //Activate accelerometer SPI
  HAL_GPIO_WritePin(GPIOB, cs_acc_Pin, GPIO_PIN_RESET);
  //Send data
  HAL_SPI_Transmit(&hspi2, &reg_Address, 1, 1000);
  HAL_SPI_Transmit(&hspi2, &register_Value, 1, 1000);
  //Deactivate accelerometer SPI
  HAL_GPIO_WritePin(GPIOB, cs_acc_Pin, GPIO_PIN_SET);
  
  
  //REFERENCE
  reg_Address = 0x26;                      //rw:0, ms:0 adress: 0x26
  register_Value = 0x00;                   //0000 0000
  //Activate accelerometer SPI
  HAL_GPIO_WritePin(GPIOB, cs_acc_Pin, GPIO_PIN_RESET);
  //Send data
  HAL_SPI_Transmit(&hspi2, &reg_Address, 1, 1000);
  HAL_SPI_Transmit(&hspi2, &register_Value, 1, 1000);
  //Deactivate accelerometer SPI
  HAL_GPIO_WritePin(GPIOB, cs_acc_Pin, GPIO_PIN_SET);  
  

  
  //INT1_THS  
  reg_Address = 0x32;                      //rw:0, ms:0 adress: 0x32
  register_Value = 0x00;                   //0000 0000
  //Activate accelerometer SPI
  HAL_GPIO_WritePin(GPIOB, cs_acc_Pin, GPIO_PIN_RESET);
  //Send data
  HAL_SPI_Transmit(&hspi2, &reg_Address, 1, 1000);
  HAL_SPI_Transmit(&hspi2, &register_Value, 1, 1000);
  //Deactivate accelerometer SPI
  HAL_GPIO_WritePin(GPIOB, cs_acc_Pin, GPIO_PIN_SET);

  
  //INT1_DUR 
  reg_Address = 0x33;                      //rw:0, ms:0 adress: 0x33
  register_Value = 0x00;                   //0000 0000
  //Activate accelerometer SPI
  HAL_GPIO_WritePin(GPIOB, cs_acc_Pin, GPIO_PIN_RESET);
  //Send data
  HAL_SPI_Transmit(&hspi2, &reg_Address, 1, 1000);
  HAL_SPI_Transmit(&hspi2, &register_Value, 1, 1000);
  //Deactivate accelerometer SPI
  HAL_GPIO_WritePin(GPIOB, cs_acc_Pin, GPIO_PIN_SET);
  
  //INT1_CFG 
  reg_Address = 0x38;                      //rw:0, ms:0 adress: 0x38
  register_Value = 0x00;                   //0000 0000
  //Activate accelerometer SPI
  HAL_GPIO_WritePin(GPIOB, cs_acc_Pin, GPIO_PIN_RESET);
  //Send data
  HAL_SPI_Transmit(&hspi2, &reg_Address, 1, 1000);
  HAL_SPI_Transmit(&hspi2, &register_Value, 1, 1000);
  //Deactivate accelerometer SPI
  HAL_GPIO_WritePin(GPIOB, cs_acc_Pin, GPIO_PIN_SET);
  
  //CTRL_REG5
  reg_Address = 0x24;                      //rw:0, ms:0 adress: 0x24
  register_Value = 0x00;                   //0000 0000
  //Activate accelerometer SPI
  HAL_GPIO_WritePin(GPIOB, cs_acc_Pin, GPIO_PIN_RESET);
  //Send data
  HAL_SPI_Transmit(&hspi2, &reg_Address, 1, 1000);
  HAL_SPI_Transmit(&hspi2, &register_Value, 1, 1000);
  //Deactivate accelerometer SPI
  HAL_GPIO_WritePin(GPIOB, cs_acc_Pin, GPIO_PIN_SET);
  
}

/*----------------------------------------
iNemo init
Version: 1.2 (2015-12-07)
Author: Emil Netz
Description: Will initiate all(gyro and acc) control register for iNemo
----------------------------------------*/
void iNemo_init(void)
{
  uint8_t reg_address = 0x0F;
  uint8_t reg_data = 0x0;
  //int TestData;
  
  //Empty print
  //TestData = read_data(reg_address, ACC);
  //printf("this is a empty test: 0: %i \n" , TestData);
  
  //CTRL for all Acc -----------------------------------------------------------

  //CTRL_REG0_XM
  reg_address = 0x1F;
  reg_data = 0x0; //0000 0000
  write_data(reg_address, reg_data, ACC);

  //CTRL_REG1_XM
  reg_address = 0x20;
  reg_data = 0xA7; //1010 0111
  write_data(reg_address, reg_data, ACC);
 
  //CTRL_REG2_XM
  reg_address = 0x21;
  reg_data = 0x20; //0010 0000
  write_data(reg_address, reg_data, ACC);
 
  //CTRL_REG3_XM
  reg_address = 0x22;
  reg_data = 0x0; //0000 0000
  write_data(reg_address, reg_data, ACC);
  
  //CTRL_REG4_XM
  reg_address = 0x23;
  reg_data = 0x0; //0000 0000
  write_data(reg_address, reg_data, ACC);
  
  //CTRL_REG5_XM
  reg_address = 0x24;
  reg_data = 0x18; //0001 1000
  write_data(reg_address, reg_data, ACC);
  
  //CTRL_REG6_XM
  reg_address = 0x25;
  reg_data = 0x20; //0010 0000
  write_data(reg_address, reg_data, ACC);
  
  //CTRL_REG7_XM
  reg_address = 0x26;
  reg_data = 0x2; //0000 0010
  write_data(reg_address, reg_data, ACC);
  
  /*
  //Testcode to check for register content
  TestData = read_data(reg_address, ACC);
  printf("CTRL_REG0_XM: 0: %i \n" , TestData);
 
  //Testcode to check for register content
  TestData = read_data(reg_address, ACC);
  printf("CTRL_REG1_XM: 167: %i \n" , TestData);
  
  //Testcode to check for register content
  TestData = read_data(reg_address, ACC);
  printf("CTRL_REG2_XM: 32: %i \n" , TestData);
  //Testcode to check for register content
  TestData = read_data(reg_address, ACC);
  printf("CTRL_REG3_XM: 0: %i \n" , TestData);
  //Testcode to check for register content
  TestData = read_data(reg_address, ACC);
  printf("CTRL_REG4_XM: 0: %i \n" , TestData);
  //Testcode to check for register content
  TestData = read_data(reg_address, ACC);
  printf("CTRL_REG5_XM: 24: %i \n" , TestData);
  
  //Testcode to check for register content
  TestData = read_data(reg_address, ACC);
  printf("CTRL_REG6_XM: 32: %i \n" , TestData);;
  
  //Testcode to check for register content
  TestData = read_data(reg_address, ACC);
  printf("CTRL_REG7_XM: 2: %i \n" , TestData);
  */


  //CTRL for all Gyro -----------------------------------------------------------

  //CTRL_REG1_G
  reg_address = 0x20;
  reg_data = 0xEF; //1110 1111
  write_data(reg_address, reg_data, GYRO);
  
  //CTRL_REG2_G
  reg_address = 0x21;
  reg_data = 0x27; //0010 0111
  write_data(reg_address, reg_data, GYRO);
  
  //CTRL_REG3_G
  reg_address = 0x22;
  reg_data = 0x0; //0000 0000
  write_data(reg_address, reg_data, GYRO);
  
  //CTRL_REG4_G
  reg_address = 0x23;
  reg_data = 0x20; //0010 0000
  write_data(reg_address, reg_data, GYRO);
  
  //CTRL_REG5_G
  reg_address = 0x24;
  reg_data = 0x10; //0001 0000
  write_data(reg_address, reg_data, GYRO);
 
  /*
  //Testcode to check for register content
  TestData = read_data(reg_address, GYRO);
  printf("CTRL_REG1_G: 207: %i \n" , TestData);
  //Testcode to check for register content
  TestData = read_data(reg_address, GYRO);
  printf("CTRL_REG2_G: 0: %i \n" , TestData);
  //Testcode to check for register content
  TestData = read_data(reg_address, GYRO);
  printf("CTRL_REG3_G: 0: %i \n" , TestData);
  //Testcode to check for register content
  TestData = read_data(reg_address, GYRO);
  printf("CTRL_REG4_G: 48: %i \n" , TestData);
  //Testcode to check for register content
  TestData = read_data(reg_address, GYRO);
  printf("CTRL_REG1_G: 0: %i \n" , TestData);
  */
}

/*
* Name: l3gd20h_Init
* Desc: Initiates the sensor by sending data to the sensors controlregisters.
        This code has been writen specificly for this project and should not
        be changed.
* Args: null
* Rets: null
*/
void l3gd20h_Init(void)
{
  //lis3dh init variables
  uint8_t register_Value;       //Used to store the information sent to the register     
  uint8_t reg_Address;          //Used to store address of used register and rw and ms bit
  uint8_t status_Reg[2] = {0x00, 0x00};
  
  /*
  To start an SPI transmission, a specific pin must be set to 0.
  The MCU has the problem with having these pins set as 0 as default.
  This can confuse the sensor during the first transmission.
  To solve this, we start a transmission and try to read a specific register
  that always contain the same value.
  We know that the confusion has subsided when we receive the correct value
  from the sensor.
  */
  reg_Address = 0x8F;
  while(status_Reg[1] != 0xD7)
  {
    HAL_GPIO_WritePin(GPIOB, cs_gyro_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi2, &reg_Address, (uint8_t *)status_Reg, 2, 1000); 
    HAL_GPIO_WritePin(GPIOB, cs_gyro_Pin, GPIO_PIN_SET);
   // printf("%i\n", status_Reg[1]);
  }
  
  //CTRL_REG1
  reg_Address = 0x20;                      //rw:0, ms:0 adress: 0x20
  register_Value = 0xEF;                   //1110 1111
  //Activate accelerometer SPI
  HAL_GPIO_WritePin(GPIOB, cs_gyro_Pin, GPIO_PIN_RESET);
  //Send data
  HAL_SPI_Transmit(&hspi2, &reg_Address, 1, 1000);
  HAL_SPI_Transmit(&hspi2, &register_Value, 1, 1000);
  //Deactivate accelerometer SPI
  HAL_GPIO_WritePin(GPIOB, cs_gyro_Pin, GPIO_PIN_SET);
  
  //CTRL_REG2
  reg_Address = 0x21;                      //rw:0, ms:0 adress: 0x21
  register_Value = 0x00;                   //0000 0000
  //Activate accelerometer SPI
  HAL_GPIO_WritePin(GPIOB, cs_gyro_Pin, GPIO_PIN_RESET);
  //Send data
  HAL_SPI_Transmit(&hspi2, &reg_Address, 1, 1000);
  HAL_SPI_Transmit(&hspi2, &register_Value, 1, 1000);
  //Deactivate accelerometer SPI
  HAL_GPIO_WritePin(GPIOB, cs_gyro_Pin, GPIO_PIN_SET);
  
  //CTRL_REG3
  reg_Address = 0x22;                      //rw:0, ms:0 adress: 0x22
  register_Value = 0x00;                   //0000 0000
  //Activate accelerometer SPI
  HAL_GPIO_WritePin(GPIOB, cs_gyro_Pin, GPIO_PIN_RESET);
  //Send data
  HAL_SPI_Transmit(&hspi2, &reg_Address, 1, 1000);
  HAL_SPI_Transmit(&hspi2, &register_Value, 1, 1000);
  //Deactivate accelerometer SPI
  HAL_GPIO_WritePin(GPIOB, cs_gyro_Pin, GPIO_PIN_SET);
  
  //CTRL_REG4
  reg_Address = 0x23;                      //rw:0, ms:0 adress: 0x23
  register_Value = 0x60;                   //0110 0000
  //Activate accelerometer SPI
  HAL_GPIO_WritePin(GPIOB, cs_gyro_Pin, GPIO_PIN_RESET);
  //Send data
  HAL_SPI_Transmit(&hspi2, &reg_Address, 1, 1000);
  HAL_SPI_Transmit(&hspi2, &register_Value, 1, 1000);
  //Deactivate accelerometer SPI
  HAL_GPIO_WritePin(GPIOB, cs_gyro_Pin, GPIO_PIN_SET);
  
  //CTRL_REG5
  reg_Address = 0x24;                      //rw:0, ms:0 adress: 0x24
  register_Value = 0x00;                   //0000 0000
  //Activate accelerometer SPI
  HAL_GPIO_WritePin(GPIOB, cs_gyro_Pin, GPIO_PIN_RESET);
  //Send data
  HAL_SPI_Transmit(&hspi2, &reg_Address, 1, 1000);
  HAL_SPI_Transmit(&hspi2, &register_Value, 1, 1000);
  //Deactivate accelerometer SPI
  HAL_GPIO_WritePin(GPIOB, cs_gyro_Pin, GPIO_PIN_SET);
}

void write_data(int address, int data, int type)
{
  if (type == GYRO)
    HAL_GPIO_WritePin(GPIOB,  cs_gyro_Pin, GPIO_PIN_RESET);

  if (type == ACC)
    HAL_GPIO_WritePin(GPIOB,  cs_acc_Pin, GPIO_PIN_RESET);

  uint8_t ADDRESS = 0x00; //0000 0000
  ADDRESS = ADDRESS | address; //00XX XXXX
  //printf("Address value: %d \n", ADDRESS);//test

  uint8_t DATA = data; //XXXX XXXX
  //printf("Data value: %d \n", DATA);//test

  HAL_SPI_Transmit(&hspi2, &ADDRESS, 1, 100000);
  HAL_SPI_Transmit(&hspi2, &DATA, 1, 100000);
  HAL_GPIO_WritePin(GPIOB,  cs_acc_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB,  cs_gyro_Pin, GPIO_PIN_SET);
}


/*----------------------------------------
Read data
Version: 1.1 (2015-12-07)
Author: Emil Netz
Description: Function that will read data from an address to iNemo.
OBS send adress in 0-5 bits. NOT 0-6 bit 
----------------------------------------*/
int read_data(int address, int type)
{
  if (type == GYRO)
    HAL_GPIO_WritePin(GPIOB,  cs_gyro_Pin, GPIO_PIN_RESET);

  if (type == ACC)
    HAL_GPIO_WritePin(GPIOB,  cs_acc_Pin, GPIO_PIN_RESET);

  uint8_t reciveDATA[7]; 
  uint8_t ADDRESS = 0x80;//1000 0000
  ADDRESS = ADDRESS | address;//10XX XXXX
  //printf("Address value: %d \n", ADDRESS);//test

  HAL_SPI_TransmitReceive (&hspi2, &ADDRESS, (uint8_t *)reciveDATA, 2, 100000);
  //printf("reciveDATA value: %d \n", reciveDATA[1]);//test
  HAL_GPIO_WritePin(GPIOB,  cs_acc_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB,  cs_gyro_Pin, GPIO_PIN_SET);
  return reciveDATA[1];
}



/**************************************************************************
(11)      systemInit
**************************************************************************/
/*  The function systemInit initiates the code generated from cubeMX
    in a seperate function.*/

void systemInit(void){
/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USB_PCD_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_4);
  
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_3);
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_4);
  
        //initialization of PWM-signal
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
  
  /* Sensor Init Code */
  
  /* Setup sensor register */
  HAL_GPIO_WritePin(GPIOB, cs_acc_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, cs_gyro_Pin, GPIO_PIN_SET);
  

  
  if(iNemo){
    iNemo_init(); /*Init iNemo*/
  }else{
    l3gd20h_Init();  /*Init gyro*/
    lis3dh_Init();   /*Init accel*/ 
  }

    /* End sensor Init Code */
}
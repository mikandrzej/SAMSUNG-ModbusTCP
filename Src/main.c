/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "lwip.h"

/* USER CODE BEGIN Includes */

#include "ADE7759/ADE7759.h"
#include "Izo_MODBUS_TCP_SRV/Izo_MODBUS_TCP_SRV.h"
#include "Izo_MODBUS_TCP_SRV/MODB_TCP_srv/Modbus_data.h"
#include "I2C_interr/i2c_interr.h"
#include "I2C_interr/i2c_selector.h"
#include "I2C_interr/i2c_sens.h"
#include "common/IOports.h"

#include "I2C_interr/temp_avg.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi5;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

uint16_t timer1s = 1000;
uint8_t timer1s_f = 0;
uint16_t timer100ms = 100;
uint8_t timer100ms_f = 0;
uint16_t timer200ms = 200;
uint8_t timer200ms_f = 0;
uint16_t timer50ms = 50;
uint8_t timer50ms_f = 0;
uint16_t timer25ms = 25;
uint8_t timer25ms_f = 0;
uint16_t timer20ms = 20;
uint8_t timer20ms_f = 0;
uint16_t timer10ms = 10;
uint8_t timer10ms_f = 0;
uint16_t timer1ms = 1;
uint8_t timer1ms_f = 0;
uint16_t timer_custom;

uint8_t do_reset = 0;

struct netif gnetif;
int8_t modb_timeout=-1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI5_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C3_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

uint8_t EVENT_1ms (void)
{
	  if(timer1ms_f)
	  {
		  timer1ms_f = 0;
		  return 1;
	  }
	  return 0;
}
uint8_t EVENT_200ms (void)
{
	  if(timer200ms_f)
	  {
		  timer200ms_f = 0;
		  return 1;
	  }
	  return 0;
}
uint8_t EVENT_100ms (void)
{
	  if(timer100ms_f)
	  {
		  timer100ms_f = 0;
		  return 1;
	  }
	  return 0;
}
uint8_t EVENT_50ms (void)
{
	  if(timer50ms_f)
	  {
		  timer50ms_f = 0;
		  return 1;
	  }
	  return 0;
}
uint8_t EVENT_25ms (void)
{
	  if(timer25ms_f)
	  {
		  timer25ms_f = 0;
		  return 1;
	  }
	  return 0;
}
uint8_t EVENT_20ms (void)
{
	  if(timer20ms_f)
	  {
		  timer20ms_f = 0;
		  return 1;
	  }
	  return 0;
}
uint8_t EVENT_10ms (void)
{
	  if(timer10ms_f)
	  {
		  timer10ms_f = 0;
		  return 1;
	  }
	  return 0;
}
uint8_t EVENT_1s (void)
{
	  if(timer1s_f)
	  {
		  timer1s_f = 0;
		  return 1;
	  }
	  return 0;
}

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* USER CODE BEGIN Init */


  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */


  selectInputI2C(0,1);

  /* USER CODE END SysInit */
  /* Initialize all configured peripherals */
  MX_GPIO_Init();


  HAL_GPIO_WritePin(ETH_RST_GPIO_Port, ETH_RST_Pin, GPIO_PIN_RESET);
  HAL_Delay(500);
  HAL_GPIO_WritePin(ETH_RST_GPIO_Port, ETH_RST_Pin, GPIO_PIN_SET);


  MX_LWIP_Init();
  MX_SPI5_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();

  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(FRIDGE_POWER_GPIO_Port, FRIDGE_POWER_Pin, 0);
  HAL_GPIO_WritePin(LAMP_FRIDGE_POWER_GPIO_Port, LAMP_FRIDGE_POWER_Pin, 0);
  HAL_GPIO_WritePin(LAMP_AC_GPIO_Port, LAMP_AC_Pin, 0);


  avgInit();
  ADE7759_Init();
  TCP_Modbus_init(502);
  i2c_interr_init();

  HAL_GPIO_WritePin(LAMP_READY_GPIO_Port, LAMP_READY_Pin, 1);
  HAL_GPIO_WritePin(LAMP_RUN_GPIO_Port, LAMP_RUN_Pin, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  size_t eth_timeout = 0;
  while (1)
  {
	  if(HAL_GPIO_ReadPin(I0_7_GPIO_Port, I0_7_Pin))
		  NVIC_SystemReset();

	  if (EVENT_1ms())
	  {
		  uint16_t control = ModbusRegReadData(MODB_REG_CONTROL);
		  uint8_t relay = control & MODB_CONTROL_RELAY_MASK;
		  HAL_GPIO_WritePin(FRIDGE_POWER_GPIO_Port, FRIDGE_POWER_Pin, relay);
		  HAL_GPIO_WritePin(LAMP_FRIDGE_POWER_GPIO_Port, LAMP_FRIDGE_POWER_Pin, relay);
		  HAL_GPIO_WritePin(LAMP_AC_GPIO_Port, LAMP_AC_Pin, relay);

		  uint16_t status = ModbusRegReadData(MODB_REG_STATUS);
		  status &= ~MODB_CONTROL_RELAY_MASK;				// stan przekaünika
		  status |= control & MODB_CONTROL_RELAY_MASK;
		  status |= MODB_STATUS_FUSE_230;					// stan zabezpieczenia
		  if(HAL_GPIO_ReadPin(I0_0_GPIO_Port, I0_0_Pin))
			  status &= ~MODB_STATUS_FUSE_230;
		  ModbusRegInsertData(MODB_REG_STATUS, status);


		  ADE_tim++;


	  }
	  if (EVENT_200ms())
	  {

		  /*
		  if (ModbusRegReadData(MODB_REG_CONTROL) & MODB_CONTROL_RELAY_MASK)
			  ModbusRegInsertData(MODB_REG_CONTROL, ModbusRegReadData(MODB_REG_CONTROL) & ~(MODB_CONTROL_RELAY_MASK));
		  else
			  ModbusRegInsertData(MODB_REG_CONTROL, ModbusRegReadData(MODB_REG_CONTROL) | MODB_CONTROL_RELAY_MASK);
			*/
		  ModbusRegInsertData(72, err_cnt_i2c);

		  ModbusRegInsertData(MODB_REG_EVENT_ID, ModbusRegReadData(MODB_REG_EVENT_ID) + 1);
		  for (int k=0; k<16; k++)
		  {
			  //ModbusRegInsertData(MODB_REG_SENS1_VAL + k*MODB_REG_SENS_OFFSET, temperature_raw[k]);
			  ModbusRegInsertData(MODB_REG_SENS1_VAL + k*MODB_REG_SENS_OFFSET, avgGetValue(k));
			  uint16_t temp;
			  temp = sid[k];
			  if(avgValuesInReg[k]==0)
				  temp=0xFFFF;
			  ModbusRegInsertData(MODB_REG_SENS1_SID + k*MODB_REG_SENS_OFFSET, temp);
		  }
		  timer_custom = ModbusRegReadData(MODB_REG_CYCLE);
		  if (timer_custom < 1000)
			  ModbusRegInsertData(MODB_REG_CYCLE, 1000);
		  else
			  ModbusRegInsertData(MODB_REG_CYCLE, timer_custom);
		  if (timer_custom < 2000)
			  timer_custom = 2000;
		  avg_values = ( timer_custom / 200);
		  if(avg_values > 100)
			  avg_values = 100;

		  int16_t *ptr;
		  for(int k = 0; k < 16; k++){
			  if (sid[k] != 0xFFFF)	{
				  ptr = (int16_t *)&temperature_raw[k];
				  if(sens[k/8][k%8].meas_cnt>5)
					  avgAddValue(k, *ptr);
			  }
		  }

		  ModbusRegInsertData(50, (I2C3->SR1>>16));
		  ModbusRegInsertData(51, (I2C3->SR1 & 0xFFFF));
		  ModbusRegInsertData(52, (I2C3->SR2>>16));
		  ModbusRegInsertData(53, (I2C3->SR2 & 0xFFFF));
		  ModbusRegInsertData(54, (I2C3->CR1>>16));
		  ModbusRegInsertData(55, (I2C3->CR1 & 0xFFFF));
		  ModbusRegInsertData(56, (I2C3->CR2>>16));
		  ModbusRegInsertData(57, (I2C3->CR2 & 0xFFFF));
		  ModbusRegInsertData(58, i2c_err_cnt1);

		  ModbusRegInsertData(60, (I2C2->SR1>>16));
		  ModbusRegInsertData(61, (I2C2->SR1 & 0xFFFF));
		  ModbusRegInsertData(62, (I2C2->SR2>>16));
		  ModbusRegInsertData(63, (I2C2->SR2 & 0xFFFF));
		  ModbusRegInsertData(64, (I2C2->CR1>>16));
		  ModbusRegInsertData(65, (I2C2->CR1 & 0xFFFF));
		  ModbusRegInsertData(66, (I2C2->CR2>>16));
		  ModbusRegInsertData(67, (I2C2->CR2 & 0xFFFF));
		  ModbusRegInsertData(68, i2c_err_cnt2);


		  ModbusRegInsertData(69, avgValuesInReg[0]);
		  ModbusRegInsertData(70, avgActVal[0]);
		  ModbusRegInsertData(71, avg_values);


		  uint32_t regvalue;
		  HAL_StatusTypeDef res;
		  res = HAL_ETH_ReadPHYRegister(&heth, PHY_ISFR, &regvalue);
		  if(res != HAL_OK || !(gnetif.flags & NETIF_FLAG_LINK_UP))
			  eth_timeout++;
		  else
			  eth_timeout = 0;
		  if (eth_timeout > 10 || modb_timeout > 10)
			  NVIC_SystemReset();
		  if(modb_timeout >= 0)
			  modb_timeout++;

	  }
	  if (EVENT_10ms())
	  {
		  task_Temperature();
	  }
	  if (EVENT_25ms())
	  {
		  meas_perm1 = 1;
		  meas_perm2 = 1;
	  }
	  if(timer1s_f)
	  {
		  timer1s_f = 0;
	  }
	  MX_LWIP_Process();
	  ADE7759_Process();
	  TCP_Modbus_Process();

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C2 init function */
static void MX_I2C2_Init(void)
{

  hi2c2.Instance->FLTR |= (2<<I2C_FLTR_DNF);
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 10000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C3 init function */
static void MX_I2C3_Init(void)
{
  hi2c3.Instance->FLTR |= (2<<I2C_FLTR_DNF);
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 10000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI5 init function */
static void MX_SPI5_Init(void)
{

  /* SPI5 parameter configuration*/
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, Q1_0_Pin|Q1_1_Pin|Q1_2_Pin|Q1_3_Pin 
                          |USER_LED1_Pin|USER_LED2_Pin|USER_LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI5_CSS_GPIO_Port, SPI5_CSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Q_REL_Pin|I2C_SELECTOR1_S0_Pin|I2C_SELECTOR2_S1_Pin|I2C_SELECTOR2_S0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(I2C_SELECTOR1_S1_GPIO_Port, I2C_SELECTOR1_S1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, Q0_0_Pin|Q0_1_Pin|Q0_2_Pin|Q0_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Q0_4_Pin|Q0_5_Pin|Q0_6_Pin|Q0_7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, ETH_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Q1_0_Pin Q1_1_Pin Q1_2_Pin Q1_3_Pin 
                           USER_LED1_Pin USER_LED2_Pin USER_LED3_Pin */
  GPIO_InitStruct.Pin = Q1_0_Pin|Q1_1_Pin|Q1_2_Pin|Q1_3_Pin 
                          |USER_LED1_Pin|USER_LED2_Pin|USER_LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : ADE_IRQ_Pin */
  GPIO_InitStruct.Pin = ADE_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ADE_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ADE_CF_Pin */
  GPIO_InitStruct.Pin = ADE_CF_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ADE_CF_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI5_CSS_Pin */
  GPIO_InitStruct.Pin = SPI5_CSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI5_CSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Q_REL_Pin I2C_SELECTOR1_S0_Pin I2C_SELECTOR2_S1_Pin I2C_SELECTOR2_S0_Pin */
  GPIO_InitStruct.Pin = Q_REL_Pin|I2C_SELECTOR1_S0_Pin|I2C_SELECTOR2_S1_Pin|I2C_SELECTOR2_S0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : I0_7_Pin */
  GPIO_InitStruct.Pin = I0_7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(I0_7_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : I0_6_Pin I0_5_Pin */
  GPIO_InitStruct.Pin = I0_6_Pin|I0_5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : I0_4_Pin I0_3_Pin I0_2_Pin I0_1_Pin 
                           I0_0_Pin */
  GPIO_InitStruct.Pin = I0_4_Pin|I0_3_Pin|I0_2_Pin|I0_1_Pin 
                          |I0_0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : I2C_SELECTOR1_S1_Pin */
  GPIO_InitStruct.Pin = I2C_SELECTOR1_S1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(I2C_SELECTOR1_S1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Q0_0_Pin Q0_1_Pin Q0_2_Pin Q0_3_Pin */
  GPIO_InitStruct.Pin = Q0_0_Pin|Q0_1_Pin|Q0_2_Pin|Q0_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : Q0_4_Pin Q0_5_Pin Q0_6_Pin Q0_7_Pin */
    GPIO_InitStruct.Pin = Q0_4_Pin|Q0_5_Pin|Q0_6_Pin|Q0_7_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins :  ETH_RST */
	  GPIO_InitStruct.Pin = ETH_RST_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);



  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */


void HAL_SYSTICK_Callback(void)
{

	if ( !(--timer1s) )
	{
		timer1s_f++;
		timer1s = 1000;
	}
	if ( !(--timer200ms) )
	{
		timer200ms_f++;
		timer200ms = 200;
	}
	if ( !(--timer100ms) )
	{
		timer100ms_f++;
		timer100ms = 100;
	}
	if ( !(--timer50ms) )
	{
		timer50ms_f++;
		timer50ms = 50;
	}
	if ( !(--timer25ms) )
	{
		timer25ms_f++;
		timer25ms = 25;
	}
	if ( !(--timer20ms) )
	{
		timer20ms_f++;
		timer20ms = 20;
	}
	if ( !(--timer10ms) )
	{
		timer10ms_f++;
		timer10ms = 10;
	}
	if ( !(--timer1ms) )
	{
		timer1ms_f++;
		timer1ms = 1;
	}
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	ADE7759_Interrupt();
}


/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define Q1_0_Pin GPIO_PIN_2
#define Q1_0_GPIO_Port GPIOE
#define Q1_1_Pin GPIO_PIN_3
#define Q1_1_GPIO_Port GPIOE
#define Q1_2_Pin GPIO_PIN_4
#define Q1_2_GPIO_Port GPIOE
#define Q1_3_Pin GPIO_PIN_5
#define Q1_3_GPIO_Port GPIOE
#define ADE_IRQ_Pin GPIO_PIN_4
#define ADE_IRQ_GPIO_Port GPIOF
#define ADE_IRQ_EXTI_IRQn EXTI4_IRQn
#define ADE_CF_Pin GPIO_PIN_5
#define ADE_CF_GPIO_Port GPIOF
#define SPI5_CSS_Pin GPIO_PIN_6
#define SPI5_CSS_GPIO_Port GPIOF
#define Q_REL_Pin GPIO_PIN_2
#define Q_REL_GPIO_Port GPIOC
#define I0_7_Pin GPIO_PIN_15
#define I0_7_GPIO_Port GPIOF
#define I0_6_Pin GPIO_PIN_0
#define I0_6_GPIO_Port GPIOG
#define I0_5_Pin GPIO_PIN_1
#define I0_5_GPIO_Port GPIOG
#define I0_4_Pin GPIO_PIN_7
#define I0_4_GPIO_Port GPIOE
#define I0_3_Pin GPIO_PIN_8
#define I0_3_GPIO_Port GPIOE
#define I0_2_Pin GPIO_PIN_9
#define I0_2_GPIO_Port GPIOE
#define I0_1_Pin GPIO_PIN_10
#define I0_1_GPIO_Port GPIOE
#define I0_0_Pin GPIO_PIN_11
#define I0_0_GPIO_Port GPIOE
#define USER_LED1_Pin GPIO_PIN_12
#define USER_LED1_GPIO_Port GPIOE
#define USER_LED2_Pin GPIO_PIN_13
#define USER_LED2_GPIO_Port GPIOE
#define USER_LED3_Pin GPIO_PIN_14
#define USER_LED3_GPIO_Port GPIOE
#define I2C_SELECTOR1_S1_Pin GPIO_PIN_15
#define I2C_SELECTOR1_S1_GPIO_Port GPIOA
#define I2C_SELECTOR1_S0_Pin GPIO_PIN_10
#define I2C_SELECTOR1_S0_GPIO_Port GPIOC
#define I2C_SELECTOR2_S1_Pin GPIO_PIN_11
#define I2C_SELECTOR2_S1_GPIO_Port GPIOC
#define I2C_SELECTOR2_S0_Pin GPIO_PIN_12
#define I2C_SELECTOR2_S0_GPIO_Port GPIOC
#define Q0_0_Pin GPIO_PIN_9
#define Q0_0_GPIO_Port GPIOG
#define Q0_1_Pin GPIO_PIN_10
#define Q0_1_GPIO_Port GPIOG
#define Q0_2_Pin GPIO_PIN_11
#define Q0_2_GPIO_Port GPIOG
#define Q0_3_Pin GPIO_PIN_12
#define Q0_3_GPIO_Port GPIOG
#define Q0_4_Pin GPIO_PIN_4
#define Q0_4_GPIO_Port GPIOB
#define Q0_5_Pin GPIO_PIN_6
#define Q0_5_GPIO_Port GPIOB
#define Q0_6_Pin GPIO_PIN_7
#define Q0_6_GPIO_Port GPIOB
#define Q0_7_Pin GPIO_PIN_8
#define Q0_7_GPIO_Port GPIOB
#define ETH_RST_Pin GPIO_PIN_2
#define ETH_RST_GPIO_Port GPIOD

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

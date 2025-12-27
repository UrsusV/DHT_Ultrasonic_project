/**
  ******************************************************************************
  * @file    switch.h
  * @brief   Header file for 2-position switch control
  ******************************************************************************
  * @attention
  *
  * This file contains the function declarations and definitions for controlling
  * a simple 2-position (push/pull) switch connected to STM32 GPIO.
  *
  * Hardware Connection:
  * - Switch Pin 1 → PB0 (STM32 GPIO with internal pull-up)
  * - Switch Pin 2 → GND
  *
  * Switch States:
  * - Position 1 (Released/Pull): PB0 reads HIGH → DHT11 Mode (Mode 1)
  * - Position 2 (Pressed/Push):  PB0 reads LOW  → Motor Mode (Mode 2)
  *
  ******************************************************************************
  */

#ifndef __SWITCH_H
#define __SWITCH_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported defines ----------------------------------------------------------*/
#define SWITCH_PIN          GPIO_PIN_0
#define SWITCH_PORT         GPIOB

/* Switch mode definitions */
#define SWITCH_MODE_DHT11   1    // Switch released (HIGH)
#define SWITCH_MODE_MOTOR   2    // Switch pressed (LOW)

/* Exported function prototypes ----------------------------------------------*/
void Switch_Init(void);
uint8_t Switch_Read_Mode(void);
uint8_t Switch_Is_Pressed(void);
uint8_t Switch_Is_Released(void);

#ifdef __cplusplus
}
#endif

#endif /* __SWITCH_H */

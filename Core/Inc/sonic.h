/**
  ******************************************************************************
  * @file    sonic.h
  * @brief   HC-SR04 Ultrasonic Sensor Library Header
  * @author  Your Name
  * @date    2024
  ******************************************************************************
  * @attention
  *
  * This library provides functions to interface with HC-SR04 ultrasonic sensor
  * Hardware Connections:
  * - Trigger Pin -> PA9 (GPIO Output)
  * - Echo Pin -> PA10 (GPIO Input)
  *
  * Usage:
  * 1. Call SONIC_Init() once during initialization
  * 2. Call SONIC_Read_Distance() to get distance measurement
  * 3. Distance is returned in centimeters (0 = error/timeout)
  *
  ******************************************************************************
  */

#ifndef __SONIC_H
#define __SONIC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* Exported types ------------------------------------------------------------*/
typedef struct {
    float distance;     // Distance in centimeters
    uint8_t valid;      // 1 if measurement is valid, 0 if error/timeout
    uint32_t timestamp; // Timestamp of last measurement
} SONIC_Data_t;

/* Exported constants --------------------------------------------------------*/
#define SONIC_TRIGGER_PIN    GPIO_PIN_9
#define SONIC_TRIGGER_PORT   GPIOA
#define SONIC_ECHO_PIN       GPIO_PIN_10
#define SONIC_ECHO_PORT      GPIOA

/* Measurement parameters */
#define SONIC_TIMEOUT        30000  // Timeout in microseconds (30ms for ~5m max range)
#define SONIC_TRIGGER_TIME   10     // Trigger pulse duration in microseconds
#define SONIC_MIN_DISTANCE   2.0    // Minimum measurable distance in cm
#define SONIC_MAX_DISTANCE   400.0  // Maximum measurable distance in cm

/* Speed of sound at room temperature (343 m/s = 0.0343 cm/us) */
#define SOUND_SPEED          0.0343 // cm per microsecond

/* Exported macro ------------------------------------------------------------*/
#define SONIC_DISTANCE_TO_TIME(distance) ((uint32_t)((distance * 2) / SOUND_SPEED))
#define SONIC_TIME_TO_DISTANCE(time) ((float)(time * SOUND_SPEED) / 2.0)

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief Initialize HC-SR04 ultrasonic sensor
 * @retval None
 */
void SONIC_Init(void);

/**
 * @brief Read distance from ultrasonic sensor
 * @retval Distance in centimeters (0.0 if error or timeout)
 */
float SONIC_Read_Distance(void);

/**
 * @brief Read distance with full data structure
 * @param data: Pointer to SONIC_Data_t structure to store results
 * @retval 1 if successful, 0 if failed
 */
uint8_t SONIC_Read_Data(SONIC_Data_t* data);

/**
 * @brief Get last valid distance measurement
 * @retval Distance in centimeters
 */
float SONIC_Get_Last_Distance(void);

/**
 * @brief Check if sensor is ready for next measurement
 * @retval 1 if ready, 0 if busy
 */
uint8_t SONIC_Is_Ready(void);

/**
 * @brief Send trigger pulse to HC-SR04
 * @retval None
 */
void SONIC_Send_Trigger(void);

/**
 * @brief Wait for echo response and measure time
 * @retval Echo time in microseconds (0 if timeout)
 */
uint32_t SONIC_Measure_Echo_Time(void);

/**
 * @brief Delay in microseconds (utility function)
 * @param us: Microseconds to delay
 * @retval None
 */
void SONIC_Delay_us(uint32_t us);

#ifdef __cplusplus
}
#endif

#endif /* __SONIC_H */

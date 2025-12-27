/**
  ******************************************************************************
  * @file    dht11.h
  * @brief   DHT11 Temperature and Humidity Sensor Library Header
  * @author  Your Name
  * @date    2024
  ******************************************************************************
  * @attention
  *
  * This library provides functions to interface with DHT11 sensor
  * Hardware Connection: DHT11 Data pin -> PA1
  *
  ******************************************************************************
  */

#ifndef DHT11_H
#define DHT11_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* Exported types ------------------------------------------------------------*/
typedef struct {
    float temperature;      // Temperature in Celsius
    float humidity;         // Humidity in percentage
    uint8_t valid;          // 1 if data is valid, 0 if error
} DHT11_Data_t;

/* Exported constants --------------------------------------------------------*/
#define DHT11_PIN           GPIO_PIN_1
#define DHT11_PORT          GPIOA

/* Exported macro ------------------------------------------------------------*/
#define DHT11_TIMEOUT       100     // Timeout in milliseconds

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief  Initialize DHT11 sensor
 * @retval None
 */
void DHT11_Init(void);

/**
 * @brief  Read temperature and humidity from DHT11 sensor
 * @param  data: Pointer to DHT11_Data_t structure to store results
 * @retval 1 if successful, 0 if failed
 */
uint8_t DHT11_Read_Data(DHT11_Data_t* data);

/**
 * @brief  Get last valid temperature reading
 * @retval Temperature in Celsius
 */
float DHT11_Get_Temperature(void);

/**
 * @brief  Get last valid humidity reading
 * @retval Humidity in percentage
 */
float DHT11_Get_Humidity(void);

/**
 * @brief  Check if last reading was valid
 * @retval 1 if valid, 0 if invalid
 */
uint8_t DHT11_Is_Data_Valid(void);

/**
 * @brief  Delay in microseconds (utility function)
 * @param  us: Microseconds to delay
 * @retval None
 */
void DHT11_Delay_us(uint32_t us);

#ifdef __cplusplus
}
#endif

#endif /* DHT11_H */

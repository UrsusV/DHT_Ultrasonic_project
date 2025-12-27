/**
  ******************************************************************************
  * @file    sonic.c
  * @brief   HC-SR04 Ultrasonic Sensor Library Implementation
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
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "sonic.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define SONIC_MEASUREMENT_INTERVAL  60  // Minimum interval between measurements (ms)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static SONIC_Data_t last_measurement = {0};
static uint32_t last_measurement_time = 0;

/* Private function prototypes -----------------------------------------------*/
static void SONIC_GPIO_Init(void);

/* Private user code ---------------------------------------------------------*/

/**
 * @brief  Initialize HC-SR04 ultrasonic sensor
 * @retval None
 */
void SONIC_Init(void)
{
    // Initialize GPIO pins
    SONIC_GPIO_Init();

    // Initialize last measurement data
    last_measurement.distance = 0.0;
    last_measurement.valid = 0;
    last_measurement.timestamp = 0;
    last_measurement_time = 0;

    // Set trigger pin low initially
    HAL_GPIO_WritePin(SONIC_TRIGGER_PORT, SONIC_TRIGGER_PIN, GPIO_PIN_RESET);

    // Wait for sensor to settle
    HAL_Delay(100);
}

/**
 * @brief  Initialize GPIO pins for ultrasonic sensor
 * @retval None
 */
static void SONIC_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Enable GPIO clock
    __HAL_RCC_GPIOA_CLK_ENABLE();

    // Configure Trigger pin (PA9) as output
    GPIO_InitStruct.Pin = SONIC_TRIGGER_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(SONIC_TRIGGER_PORT, &GPIO_InitStruct);

    // Configure Echo pin (PA10) as input
    GPIO_InitStruct.Pin = SONIC_ECHO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(SONIC_ECHO_PORT, &GPIO_InitStruct);
}

/**
 * @brief  Send trigger pulse to HC-SR04
 * @retval None
 */
void SONIC_Send_Trigger(void)
{
    // Send 10us trigger pulse
    HAL_GPIO_WritePin(SONIC_TRIGGER_PORT, SONIC_TRIGGER_PIN, GPIO_PIN_SET);
    SONIC_Delay_us(SONIC_TRIGGER_TIME);
    HAL_GPIO_WritePin(SONIC_TRIGGER_PORT, SONIC_TRIGGER_PIN, GPIO_PIN_RESET);
}

/**
 * @brief  Wait for echo response and measure time
 * @retval Echo time in microseconds (0 if timeout)
 */
uint32_t SONIC_Measure_Echo_Time(void)
{
    uint32_t start_time = 0;
    uint32_t end_time = 0;
    uint32_t timeout_counter = 0;

    // Wait for echo pin to go HIGH (start of echo)
    timeout_counter = 0;
    while (HAL_GPIO_ReadPin(SONIC_ECHO_PORT, SONIC_ECHO_PIN) == GPIO_PIN_RESET)
    {
        SONIC_Delay_us(1);
        timeout_counter++;
        if (timeout_counter > SONIC_TIMEOUT)
        {
            return 0; // Timeout - no echo received
        }
    }

    // Record start time
    start_time = DWT->CYCCNT;

    // Wait for echo pin to go LOW (end of echo)
    timeout_counter = 0;
    while (HAL_GPIO_ReadPin(SONIC_ECHO_PORT, SONIC_ECHO_PIN) == GPIO_PIN_SET)
    {
        SONIC_Delay_us(1);
        timeout_counter++;
        if (timeout_counter > SONIC_TIMEOUT)
        {
            return 0; // Timeout - echo too long
        }
    }

    // Record end time
    end_time = DWT->CYCCNT;

    // Calculate echo duration in microseconds
    uint32_t echo_time_cycles = end_time - start_time;
    uint32_t echo_time_us = echo_time_cycles / (SystemCoreClock / 1000000);

    return echo_time_us;
}

/**
 * @brief  Read distance from ultrasonic sensor
 * @retval Distance in centimeters (0.0 if error or timeout)
 */
float SONIC_Read_Distance(void)
{
    SONIC_Data_t data;

    if (SONIC_Read_Data(&data))
    {
        return data.distance;
    }
    else
    {
        return 0.0; // Error or timeout
    }
}

/**
 * @brief  Read distance with full data structure
 * @param  data: Pointer to SONIC_Data_t structure to store results
 * @retval 1 if successful, 0 if failed
 */
uint8_t SONIC_Read_Data(SONIC_Data_t* data)
{
    uint32_t echo_time_us;
    float calculated_distance;

    // Initialize data structure
    data->distance = 0.0;
    data->valid = 0;
    data->timestamp = HAL_GetTick();

    // Check if enough time has passed since last measurement
    if ((HAL_GetTick() - last_measurement_time) < SONIC_MEASUREMENT_INTERVAL)
    {
        // Return last valid measurement if available
        if (last_measurement.valid)
        {
            *data = last_measurement;
            return 1;
        }
        else
        {
            return 0;
        }
    }

    // Send trigger pulse
    SONIC_Send_Trigger();

    // Measure echo time
    echo_time_us = SONIC_Measure_Echo_Time();

    if (echo_time_us == 0)
    {
        return 0; // Timeout or error
    }

    // Calculate distance: distance = (echo_time * speed_of_sound) / 2
    calculated_distance = SONIC_TIME_TO_DISTANCE(echo_time_us);

    // Validate distance range
    if (calculated_distance < SONIC_MIN_DISTANCE || calculated_distance > SONIC_MAX_DISTANCE)
    {
        return 0; // Distance out of valid range
    }

    // Store valid measurement
    data->distance = calculated_distance;
    data->valid = 1;
    data->timestamp = HAL_GetTick();

    // Update last measurement
    last_measurement = *data;
    last_measurement_time = HAL_GetTick();

    return 1; // Success
}

/**
 * @brief  Get last valid distance measurement
 * @retval Distance in centimeters
 */
float SONIC_Get_Last_Distance(void)
{
    if (last_measurement.valid)
    {
        return last_measurement.distance;
    }
    else
    {
        return 0.0;
    }
}

/**
 * @brief  Check if sensor is ready for next measurement
 * @retval 1 if ready, 0 if busy
 */
uint8_t SONIC_Is_Ready(void)
{
    return ((HAL_GetTick() - last_measurement_time) >= SONIC_MEASUREMENT_INTERVAL) ? 1 : 0;
}

/**
 * @brief  Delay in microseconds
 * @param  us: Microseconds to delay
 * @retval None
 * @note   This function uses DWT counter for accurate microsecond timing
 */
void SONIC_Delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000);
    while ((DWT->CYCCNT - start) < ticks);
}

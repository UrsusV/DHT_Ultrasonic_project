/**
  ******************************************************************************
  * @file    dht11.c
  * @brief   DHT11 Temperature and Humidity Sensor Library
  * @author  Your Name
  * @date    2024
  ******************************************************************************
  * @attention
  *
  * This library provides functions to interface with DHT11 sensor
  * Hardware Connection: DHT11 Data pin -> PA1
  *
  * Usage:
  * 1. Call DHT11_Init() once during initialization
  * 2. Call DHT11_Read_Data() to read sensor data
  * 3. Check data validity using data.valid field
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "dht11.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define DHT11_START_SIGNAL_TIME     20      // Start signal duration in ms
#define DHT11_RESPONSE_WAIT_TIME    40      // Time to wait for response in us
#define DHT11_BIT_READ_TIME         40      // Time to read bit in us

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;  // External timer handle for microsecond delay
static DHT11_Data_t last_reading = {0}; // Store last valid reading

/* Private function prototypes -----------------------------------------------*/
static void DHT11_Set_Pin_Output(void);
static void DHT11_Set_Pin_Input(void);
static void DHT11_Start_Signal(void);
static uint8_t DHT11_Check_Response(void);
static uint8_t DHT11_Read_Bit(void);
static uint8_t DHT11_Read_Byte(void);

/* Private user code ---------------------------------------------------------*/

/**
 * @brief  Initialize DHT11 sensor
 * @retval None
 */
void DHT11_Init(void)
{
    // Set pin as input with pull-up initially
    DHT11_Set_Pin_Input();

    // Initialize last reading
    last_reading.temperature = 0.0;
    last_reading.humidity = 0.0;
    last_reading.valid = 0;
}

/**
 * @brief  Set DHT11 pin as output
 * @retval None
 */
static void DHT11_Set_Pin_Output(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = DHT11_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);
}

/**
 * @brief  Set DHT11 pin as input
 * @retval None
 */
static void DHT11_Set_Pin_Input(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = DHT11_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStruct);
}

/**
 * @brief  Send start signal to DHT11
 * @retval None
 */
static void DHT11_Start_Signal(void)
{
    DHT11_Set_Pin_Output();
    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_RESET);
    HAL_Delay(DHT11_START_SIGNAL_TIME);
    HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, GPIO_PIN_SET);
    DHT11_Delay_us(30);
    DHT11_Set_Pin_Input();
}

/**
 * @brief  Check DHT11 response after start signal
 * @retval 1 if response received, 0 if no response
 */
static uint8_t DHT11_Check_Response(void)
{
    uint8_t response = 0;
    uint32_t timeout = 0;

    // Wait for pin to go low (response signal)
    DHT11_Delay_us(DHT11_RESPONSE_WAIT_TIME);

    if (!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)))
    {
        // Wait for pin to go high
        timeout = 0;
        while (!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)) && timeout < 100)
        {
            DHT11_Delay_us(1);
            timeout++;
        }

        if (timeout < 100)
        {
            // Wait for pin to go low again
            timeout = 0;
            while ((HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)) && timeout < 100)
            {
                DHT11_Delay_us(1);
                timeout++;
            }

            if (timeout < 100)
            {
                response = 1;
            }
        }
    }

    return response;
}

/**
 * @brief  Read a single bit from DHT11
 * @retval Bit value (0 or 1)
 */
static uint8_t DHT11_Read_Bit(void)
{
    uint32_t timeout = 0;

    // Wait for pin to go high
    while (!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)) && timeout < 100)
    {
        DHT11_Delay_us(1);
        timeout++;
    }

    if (timeout >= 100) return 0; // Timeout error

    // Wait 40us and check pin state
    DHT11_Delay_us(DHT11_BIT_READ_TIME);

    if (HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN))
    {
        // Pin is still high - this is bit '1'
        // Wait for pin to go low
        timeout = 0;
        while ((HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)) && timeout < 100)
        {
            DHT11_Delay_us(1);
            timeout++;
        }
        return 1;
    }
    else
    {
        // Pin went low - this is bit '0'
        return 0;
    }
}

/**
 * @brief  Read a byte (8 bits) from DHT11
 * @retval Byte value
 */
static uint8_t DHT11_Read_Byte(void)
{
    uint8_t byte_value = 0;

    for (int i = 7; i >= 0; i--)
    {
        if (DHT11_Read_Bit())
        {
            byte_value |= (1 << i);
        }
    }

    return byte_value;
}

/**
 * @brief  Read temperature and humidity data from DHT11
 * @param  data: Pointer to DHT11_Data_t structure to store results
 * @retval 1 if successful, 0 if failed
 */
uint8_t DHT11_Read_Data(DHT11_Data_t* data)
{
    uint8_t humidity_int, humidity_dec;
    uint8_t temperature_int, temperature_dec;
    uint8_t checksum, calculated_checksum;

    // Initialize data structure
    data->valid = 0;
    data->temperature = 0.0;
    data->humidity = 0.0;

    // Send start signal
    DHT11_Start_Signal();

    // Check for response
    if (!DHT11_Check_Response())
    {
        return 0; // No response from sensor
    }

    // Read 5 bytes of data
    humidity_int = DHT11_Read_Byte();
    humidity_dec = DHT11_Read_Byte();
    temperature_int = DHT11_Read_Byte();
    temperature_dec = DHT11_Read_Byte();
    checksum = DHT11_Read_Byte();

    // Calculate checksum
    calculated_checksum = humidity_int + humidity_dec + temperature_int + temperature_dec;

    // Verify checksum
    if (calculated_checksum == checksum)
    {
        // Data is valid
        data->humidity = (float)humidity_int + ((float)humidity_dec / 10.0);
        data->temperature = (float)temperature_int + ((float)temperature_dec / 10.0);
        data->valid = 1;

        // Store as last valid reading
        last_reading = *data;

        return 1; // Success
    }
    else
    {
        return 0; // Checksum error
    }
}

/**
 * @brief  Get last valid temperature reading
 * @retval Temperature in Celsius
 */
float DHT11_Get_Temperature(void)
{
    return last_reading.temperature;
}

/**
 * @brief  Get last valid humidity reading
 * @retval Humidity in percentage
 */
float DHT11_Get_Humidity(void)
{
    return last_reading.humidity;
}

/**
 * @brief  Check if last reading was valid
 * @retval 1 if valid, 0 if invalid
 */
uint8_t DHT11_Is_Data_Valid(void)
{
    return last_reading.valid;
}

/**
 * @brief  Delay in microseconds
 * @param  us: Microseconds to delay
 * @retval None
 * @note   This function uses TIM2 counter for accurate microsecond timing
 */
void DHT11_Delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000);
    while ((DWT->CYCCNT - start) < ticks);
}


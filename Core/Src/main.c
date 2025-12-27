/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - Servo Motor, DHT11, Ultrasonic Sensor & LCD
  ******************************************************************************
  * Hardware Connections:
  * LCD: B6 (RS), B7 (E) - as per your existing setup
  * Servo Motor: A0 (PWM) - Used as indicator based on temperature/distance
  * DHT11: A1 (Data pin)
  * Ultrasonic HC-SR04:
  *   - Trigger: A9 (GPIO Output)
  *   - Echo: A10 (GPIO Input with interrupt capability)
  * Keypad 3x4: A2-A8 (keep connected but not used)
  *
  * 2-Position Switch:
  * One pin → B0 (STM32 GPIO with internal pull-up)
  * Other pin → GND
  *
  * Switch Logic:
  * Position 1 (Released/Pull): B0 = HIGH → DHT11 Temperature Mode
  * Position 2 (Pressed/Push): B0 = LOW  → Ultrasonic Distance Mode
  *
  * Motor Control Logic:
  * - Temperature ≥ 30°C: Motor runs (indicator ON)
  * - Distance ≤ 20cm: Motor runs (indicator ON)
  * - Otherwise: Motor stopped (indicator OFF)
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2c-lcd.h"
#include "sg90.h"
#include "dht11.h"
#include "switch.h"
#include "sonic.h"  // Ultrasonic sensor header
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TEMP_THRESHOLD 30.0     // Temperature threshold in Celsius
#define DISTANCE_THRESHOLD 20.0 // Distance threshold in cm
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
uint8_t motor_status = 0; // 0 = stopped, 1 = running
uint8_t current_mode = 1; // Start with DHT11 mode (switch released)
uint8_t previous_mode = 1;
uint32_t mode_change_time = 0;

// Sensor data variables
DHT11_Data_t dht11_data = {0};
float distance = 0.0;

// Motor control variables
uint8_t motor_active_temp = 0;     // Motor active due to temperature
uint8_t motor_active_distance = 0; // Motor active due to distance
uint8_t last_motor_state = 0;      // Previous motor state for change detection

static uint8_t temp_first = 1, distance_first = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
// Application Functions
void Display_Temperature_Mode(void);
void Display_Distance_Mode(void);
void Motor_Control_Logic(void);
void Update_Motor_Status(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief Motor control logic based on temperature and distance thresholds
 * Only considers readings from the currently active mode
 */
void Motor_Control_Logic(void)
{
    // Reset motor activation flags
    motor_active_temp = 0;
    motor_active_distance = 0;

    // Only check temperature if we're in temperature mode (mode 1)
    if (current_mode == 1)
    {
        if (dht11_data.valid && dht11_data.temperature >= TEMP_THRESHOLD)
        {
            motor_active_temp = 1;
        }
    }

    // Only check distance if we're in distance mode (mode 2)
    if (current_mode == 2)
    {
        if (distance > 0 && distance <= DISTANCE_THRESHOLD)
        {
            motor_active_distance = 1;
        }
    }

    // Update overall motor status
    motor_status = (motor_active_temp || motor_active_distance) ? 1 : 0;
}

/**
 * @brief Update motor physical state based on motor_status
 */
void Update_Motor_Status(void)
{
    if (motor_status != last_motor_state)
    {
        if (motor_status)
        {
            // Start motor - sweep motion for indication
            SG90_Start_Sweep();
        }
        else
        {
            // Stop motor - return to center position
            SG90_Stop();
        }
        last_motor_state = motor_status;
    }

    // Continue sweep motion if motor is active
    if (motor_status)
    {
        SG90_Update_Sweep();
    }
}

/**
 * @brief Display temperature mode with DHT11 readings and motor status
 */
void Display_Temperature_Mode(void)
{
    static uint32_t last_update = 0;
    static uint8_t first_run = 1;

    // Show mode change message once
    if (temp_first)
    {
        lcd_clear();
        lcd_put_cur(0, 0);
        lcd_send_string("DHT11 Mode      ");
        lcd_put_cur(1, 0);
        lcd_send_string("Reading...      ");
        HAL_Delay(1000);
        temp_first = 0;
        first_run = 1;
    }

    // Update DHT11 data every 2 seconds
    if (HAL_GetTick() - last_update > 2000)
    {
        DHT11_Read_Data(&dht11_data);
        last_update = HAL_GetTick();
        first_run = 1;
    }

    // Display temperature and motor status
    if (first_run)
    {
        if (dht11_data.valid)
        {
            char temp_str[17];
            char status_str[17];

            int temp_int = (int)dht11_data.temperature;
            int temp_dec = (int)((dht11_data.temperature - temp_int) * 10);

            sprintf(temp_str, "T:%d.%dC", temp_int, temp_dec);

            // Add motor status and threshold info
            if (motor_active_temp)
            {
                sprintf(status_str, "%s M:ON(T>=30)", temp_str);
            }
            else
            {
                sprintf(status_str, "%s M:OFF     ", temp_str);
            }

            lcd_put_cur(0, 0);
            lcd_send_string(status_str);

            // Second line: Humidity and overall motor status
            int hum_int = (int)dht11_data.humidity;
            int hum_dec = (int)((dht11_data.humidity - hum_int) * 10);

            char hum_str[17];
            sprintf(hum_str, "H:%d.%d%% Motor:%s ", hum_int, hum_dec, motor_status ? "ON " : "OFF");

            lcd_put_cur(1, 0);
            lcd_send_string(hum_str);
        }
        else
        {
            lcd_put_cur(0, 0);
            lcd_send_string("DHT11 Error     ");
            lcd_put_cur(1, 0);
            lcd_send_string("Check Wiring    ");
        }
        first_run = 0;
    }
}

/**
 * @brief Display distance mode with ultrasonic readings and motor status
 */
void Display_Distance_Mode(void)
{
    static uint32_t last_update = 0;
    static uint8_t first_run = 1;

    // Show mode change message once
    if (distance_first)
    {
        lcd_clear();
        lcd_put_cur(0, 0);
        lcd_send_string("Distance Mode   ");
        lcd_put_cur(1, 0);
        lcd_send_string("Measuring...    ");
        HAL_Delay(1000);
        distance_first = 0;
        first_run = 1;
    }

    // Update distance data every 500ms
    if (HAL_GetTick() - last_update > 500)
    {
        distance = SONIC_Read_Distance();
        last_update = HAL_GetTick();
        first_run = 1;
    }

    // Display distance and motor status
    if (first_run)
    {
        char dist_str[17];
        char status_str[17];

        if (distance > 0)
        {
            int dist_int = (int)distance;
            int dist_dec = (int)((distance - dist_int) * 10);

            sprintf(dist_str, "D:%d.%dcm", dist_int, dist_dec);

            // Add motor status and threshold info
            if (motor_active_distance)
            {
                sprintf(status_str, "%s M:ON(<=20)", dist_str);
            }
            else
            {
                sprintf(status_str, "%s M:OFF     ", dist_str);
            }
        }
        else
        {
            sprintf(status_str, "D:Error M:OFF   ");
        }

        lcd_put_cur(0, 0);
        lcd_send_string(status_str);

        // Second line: Overall motor status and threshold info
        char motor_str[17];
        sprintf(motor_str, "Motor: %s      ", motor_status ? "ACTIVE  " : "STOPPED ");

        lcd_put_cur(1, 0);
        lcd_send_string(motor_str);

        first_run = 0;
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();

  // Enable DWT cycle counter (for microsecond delays)
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  /* USER CODE BEGIN 2 */

  // Initialize LCD Display
  lcd_init();

  // Show startup message
  lcd_clear();
  lcd_put_cur(0, 0);
  lcd_send_string("System Ready    ");
  lcd_put_cur(1, 0);
  lcd_send_string("T>=30C|D<=20cm  ");
  HAL_Delay(2000);

  // Initialize SG90 Servo Motor
  SG90_Init();

  // Initialize DHT11 Sensor
  DHT11_Init();

  // Initialize Ultrasonic Sensor
  SONIC_Init();

  // Initialize Switch
  Switch_Init();

  // Clear screen for operation
  lcd_clear();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // Read current switch state
    uint8_t switch_mode = Switch_Read_Mode();

    // Check for mode change with debouncing
    if (switch_mode != previous_mode)
    {
        if (HAL_GetTick() - mode_change_time > 300) // 300ms debounce
        {
            current_mode = switch_mode;
            previous_mode = switch_mode;
            mode_change_time = HAL_GetTick();

            // Reset first_run flags for clean mode transition
            if (current_mode == 1)
            {
                temp_first = 1;
            }
            if (current_mode == 2)
            {
                distance_first = 1;
            }

            // Force immediate motor control update for new mode
            Motor_Control_Logic();
        }
    }

    // Update motor status regardless of current display mode
    Update_Motor_Status();

    // Execute current display mode
    switch (current_mode)
    {
        case 1: // DHT11 Temperature mode (switch released/HIGH)
            Display_Temperature_Mode();

            // Update motor control logic only for temperature mode
            Motor_Control_Logic();
            HAL_Delay(100);
            break;

        case 2: // Ultrasonic Distance mode (switch pressed/LOW)
            Display_Distance_Mode();

            // Update motor control logic only for distance mode
            Motor_Control_Logic();
            HAL_Delay(100);
            break;

        default:
            // Default to DHT11 mode
            current_mode = 1;
            break;
    }

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitStruct structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;  // 72MHz / (71+1) = 1MHz (1us per tick)
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;  // 20ms period for servo
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;  // Initial pulse width (90 degrees)
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Switch, DHT11, and Ultrasonic GPIO pins initialized in their respective Init() functions */

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

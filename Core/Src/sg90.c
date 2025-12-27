/**
  ******************************************************************************
  * @file    sg90.c
  * @brief   SG90 Servo Motor Control Library Implementation - Enhanced Version
  * @author  Your Name
  * @date    2024
  ******************************************************************************
  * @attention
  *
  * Enhanced version with sweep functionality for indicator purposes
  * Hardware Connection: SG90 Signal pin -> PA0 (TIM2_CH1 PWM)
  *
  ******************************************************************************
  */

#include "sg90.h"

/* Private variables ---------------------------------------------------------*/
static uint8_t current_angle = SERVO_CENTER_ANGLE;
static SG90_State_t servo_state = SERVO_STATE_STOPPED;
static uint32_t last_sweep_time = 0;

// Sweep control variables
static uint8_t sweep_angle = SERVO_SWEEP_MIN;
static int8_t sweep_direction = 1;  // 1 = increasing, -1 = decreasing
static uint8_t sweep_min = SERVO_SWEEP_MIN;
static uint8_t sweep_max = SERVO_SWEEP_MAX;
static uint16_t sweep_delay = SERVO_SWEEP_DELAY;

/* Private function prototypes -----------------------------------------------*/
static void SG90_SetPWM(uint8_t angle);

/* Private functions ---------------------------------------------------------*/

/**
 * @brief Set PWM duty cycle for servo angle
 * @param angle: Servo angle (0-180 degrees)
 * @retval None
 */
static void SG90_SetPWM(uint8_t angle)
{
    // Limit angle to valid range
    if (angle > SERVO_MAX_ANGLE)
        angle = SERVO_MAX_ANGLE;

    // Calculate pulse width
    uint32_t pulse_width = SERVO_ANGLE_TO_PULSE(angle);

    // Set PWM compare value
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse_width);

    // Update current angle
    current_angle = angle;
}

/* Public functions ----------------------------------------------------------*/

/**
 * @brief Initialize SG90 servo motor
 * @retval None
 */
void SG90_Init(void)
{
    // Start PWM on Timer 2 Channel 1 (PA0)
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

    // Set initial position to center (90 degrees)
    SG90_SetAngle(SERVO_CENTER_ANGLE);
    servo_state = SERVO_STATE_STOPPED;

    // Initialize sweep variables
    sweep_angle = sweep_min;
    sweep_direction = 1;
    last_sweep_time = HAL_GetTick();
}

/**
 * @brief Set servo angle (0-180 degrees)
 * @param angle: Desired angle (0-180)
 * @retval None
 */
void SG90_SetAngle(uint8_t angle)
{
    SG90_SetPWM(angle);
    servo_state = SERVO_STATE_POSITION;
}

/**
 * @brief Start continuous sweep motion (for indicator)
 * @retval None
 */
void SG90_Start_Sweep(void)
{
    servo_state = SERVO_STATE_SWEEPING;
    sweep_angle = sweep_min;
    sweep_direction = 1;
    last_sweep_time = HAL_GetTick();

    // Set initial sweep position
    SG90_SetPWM(sweep_angle);
}

/**
 * @brief Update sweep motion (call this regularly in main loop)
 * @retval None
 */
void SG90_Update_Sweep(void)
{
    // Only update if in sweeping state
    if (servo_state != SERVO_STATE_SWEEPING)
        return;

    // Check if it's time for next sweep step
    if ((HAL_GetTick() - last_sweep_time) >= sweep_delay)
    {
        // Update sweep angle
        sweep_angle += (sweep_direction * SERVO_SWEEP_STEP);

        // Check and handle sweep limits
        if (sweep_angle >= sweep_max)
        {
            sweep_angle = sweep_max;
            sweep_direction = -1;  // Change direction to decreasing
        }
        else if (sweep_angle <= sweep_min)
        {
            sweep_angle = sweep_min;
            sweep_direction = 1;   // Change direction to increasing
        }

        // Set new servo position
        SG90_SetPWM(sweep_angle);

        // Update timing
        last_sweep_time = HAL_GetTick();
    }
}

/**
 * @brief Stop servo motor (set to center position)
 * @retval None
 */
void SG90_Stop(void)
{
    SG90_SetAngle(SERVO_CENTER_ANGLE);
    servo_state = SERVO_STATE_STOPPED;
}

/**
 * @brief Get current servo angle
 * @retval Current angle (0-180)
 */
uint8_t SG90_GetAngle(void)
{
    return current_angle;
}

/**
 * @brief Get current servo state
 * @retval Current state
 */
SG90_State_t SG90_GetState(void)
{
    return servo_state;
}

/**
 * @brief Check if servo is running (sweeping or at position)
 * @retval 1 if running, 0 if stopped
 */
uint8_t SG90_IsRunning(void)
{
    return (servo_state != SERVO_STATE_STOPPED) ? 1 : 0;
}

/**
 * @brief Check if servo is sweeping
 * @retval 1 if sweeping, 0 if not
 */
uint8_t SG90_IsSweeping(void)
{
    return (servo_state == SERVO_STATE_SWEEPING) ? 1 : 0;
}

/**
 * @brief Set custom sweep range
 * @param min_angle: Minimum sweep angle (0-180)
 * @param max_angle: Maximum sweep angle (0-180)
 * @retval None
 */
void SG90_Set_Sweep_Range(uint8_t min_angle, uint8_t max_angle)
{
    // Validate angles
    if (min_angle > SERVO_MAX_ANGLE) min_angle = SERVO_MAX_ANGLE;
    if (max_angle > SERVO_MAX_ANGLE) max_angle = SERVO_MAX_ANGLE;
    if (min_angle >= max_angle) return; // Invalid range

    sweep_min = min_angle;
    sweep_max = max_angle;

    // Reset sweep if currently sweeping
    if (servo_state == SERVO_STATE_SWEEPING)
    {
        sweep_angle = sweep_min;
        sweep_direction = 1;
    }
}

/**
 * @brief Set sweep speed
 * @param delay_ms: Delay between sweep steps in milliseconds
 * @retval None
 */
void SG90_Set_Sweep_Speed(uint16_t delay_ms)
{
    if (delay_ms < 50) delay_ms = 50;   // Minimum delay for servo response
    if (delay_ms > 2000) delay_ms = 2000; // Maximum delay for reasonable speed

    sweep_delay = delay_ms;
}

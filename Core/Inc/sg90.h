/**
  ******************************************************************************
  * @file    sg90.h
  * @brief   SG90 Servo Motor Control Library Header - Enhanced Version
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

#ifndef INC_SG90_H_
#define INC_SG90_H_

#include "main.h"

/* Exported types ------------------------------------------------------------*/
typedef enum {
    SERVO_STATE_STOPPED = 0,
    SERVO_STATE_POSITION,
    SERVO_STATE_SWEEPING
} SG90_State_t;

/* Exported constants --------------------------------------------------------*/
// Servo positions (0-180 degrees)
#define SERVO_MIN_ANGLE         0
#define SERVO_MAX_ANGLE         180
#define SERVO_CENTER_ANGLE      90

// PWM pulse width values for SG90 (in microseconds)
#define SERVO_MIN_PULSE         500     // 0.5ms for 0 degrees
#define SERVO_MAX_PULSE         2500    // 2.5ms for 180 degrees
#define SERVO_CENTER_PULSE      1500    // 1.5ms for 90 degrees
#define SERVO_PERIOD            20000   // 20ms period

// Sweep parameters
#define SERVO_SWEEP_MIN         30      // Minimum angle for sweep
#define SERVO_SWEEP_MAX         150     // Maximum angle for sweep
#define SERVO_SWEEP_STEP        10      // Angle step for smooth sweep
#define SERVO_SWEEP_DELAY       200     // Delay between sweep steps (ms)

/* Exported macro ------------------------------------------------------------*/
#define SERVO_ANGLE_TO_PULSE(angle) (SERVO_MIN_PULSE + (angle * (SERVO_MAX_PULSE - SERVO_MIN_PULSE) / 180))

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;  // Timer handle (defined in main.c)

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief Initialize SG90 servo motor
 * @retval None
 */
void SG90_Init(void);

/**
 * @brief Set servo angle (0-180 degrees)
 * @param angle: Desired angle (0-180)
 * @retval None
 */
void SG90_SetAngle(uint8_t angle);

/**
 * @brief Start continuous sweep motion (for indicator)
 * @retval None
 */
void SG90_Start_Sweep(void);

/**
 * @brief Update sweep motion (call this regularly in main loop)
 * @retval None
 */
void SG90_Update_Sweep(void);

/**
 * @brief Stop servo motor (set to center position)
 * @retval None
 */
void SG90_Stop(void);

/**
 * @brief Get current servo angle
 * @retval Current angle (0-180)
 */
uint8_t SG90_GetAngle(void);

/**
 * @brief Get current servo state
 * @retval Current state (SERVO_STATE_STOPPED, SERVO_STATE_POSITION, SERVO_STATE_SWEEPING)
 */
SG90_State_t SG90_GetState(void);

/**
 * @brief Check if servo is running (sweeping or at position)
 * @retval 1 if running, 0 if stopped
 */
uint8_t SG90_IsRunning(void);

/**
 * @brief Check if servo is sweeping
 * @retval 1 if sweeping, 0 if not
 */
uint8_t SG90_IsSweeping(void);

/**
 * @brief Set custom sweep range
 * @param min_angle: Minimum sweep angle (0-180)
 * @param max_angle: Maximum sweep angle (0-180)
 * @retval None
 */
void SG90_Set_Sweep_Range(uint8_t min_angle, uint8_t max_angle);

/**
 * @brief Set sweep speed
 * @param delay_ms: Delay between sweep steps in milliseconds
 * @retval None
 */
void SG90_Set_Sweep_Speed(uint16_t delay_ms);

#endif /* INC_SG90_H_ */

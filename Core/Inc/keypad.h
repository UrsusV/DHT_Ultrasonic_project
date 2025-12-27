/**
  ******************************************************************************
  * @file    keypad.h
  * @brief   3x4 Matrix Keypad Library Header
  * @author  Your Name
  * @date    2024
  ******************************************************************************
  * @attention
  *
  * This library provides functions to interface with 3x4 matrix keypad
  * Hardware Connections:
  * Rows: PA2, PA3, PA4, PA5 (4 rows)
  * Cols: PA6, PA7, PA8 (3 columns)
  *
  * Keypad Layout:
  * [1] [2] [3]
  * [4] [5] [6]
  * [7] [8] [9]
  * [*] [0] [#]
  *
  ******************************************************************************
  */

#ifndef KEYPAD_H
#define KEYPAD_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* Exported types ------------------------------------------------------------*/
typedef struct {
    char key;           // Last pressed key
    uint8_t pressed;    // 1 if key is currently pressed, 0 otherwise
    uint32_t timestamp; // Timestamp of last key press
} Keypad_Status_t;

/* Exported constants --------------------------------------------------------*/
// Keypad pin definitions
#define KEYPAD_ROW1_PIN     GPIO_PIN_2
#define KEYPAD_ROW2_PIN     GPIO_PIN_3
#define KEYPAD_ROW3_PIN     GPIO_PIN_4
#define KEYPAD_ROW4_PIN     GPIO_PIN_5
#define KEYPAD_COL1_PIN     GPIO_PIN_6
#define KEYPAD_COL2_PIN     GPIO_PIN_7
#define KEYPAD_COL3_PIN     GPIO_PIN_8
#define KEYPAD_PORT         GPIOA

// Keypad dimensions
#define KEYPAD_ROWS         4
#define KEYPAD_COLS         3

// Keypad special keys
#define KEYPAD_STAR         '*'
#define KEYPAD_HASH         '#'
#define KEYPAD_NO_KEY       0

/* Exported macro ------------------------------------------------------------*/
#define KEYPAD_DEBOUNCE_TIME    50  // Debounce time in milliseconds

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief  Initialize keypad GPIO pins and internal variables
 * @retval None
 */
void Keypad_Init(void);

/**
 * @brief  Scan keypad for pressed keys
 * @retval Character of pressed key, or KEYPAD_NO_KEY if no key pressed
 */
char Keypad_Scan(void);

/**
 * @brief  Get keypad status including debouncing information
 * @param  status: Pointer to Keypad_Status_t structure to store status
 * @retval None
 */
void Keypad_Get_Status(Keypad_Status_t* status);

/**
 * @brief  Check if a specific key is currently pressed
 * @param  key: Character of the key to check
 * @retval 1 if key is pressed, 0 otherwise
 */
uint8_t Keypad_Is_Key_Pressed(char key);

/**
 * @brief  Wait for any key press with timeout
 * @param  timeout_ms: Timeout in milliseconds (0 = no timeout)
 * @retval Character of pressed key, or KEYPAD_NO_KEY if timeout
 */
char Keypad_Wait_For_Key(uint32_t timeout_ms);

/**
 * @brief  Clear keypad buffer (reset internal state)
 * @retval None
 */
void Keypad_Clear_Buffer(void);

/**
 * @brief  Set keypad scan rate
 * @param  scan_rate_ms: Time between scans in milliseconds
 * @retval None
 */
void Keypad_Set_Scan_Rate(uint8_t scan_rate_ms);

#ifdef __cplusplus
}
#endif

#endif /* KEYPAD_H */

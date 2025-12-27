#include "keypad.h"
#include "stm32f1xx_hal.h"

// =========================
// CONFIG: Row & Column pins
// =========================
static const uint16_t row_pins[KEYPAD_ROWS] = {
    KEYPAD_ROW1_PIN, KEYPAD_ROW2_PIN, KEYPAD_ROW3_PIN, KEYPAD_ROW4_PIN
};

static const uint16_t col_pins[KEYPAD_COLS] = {
    KEYPAD_COL1_PIN, KEYPAD_COL2_PIN, KEYPAD_COL3_PIN
};

// =========================
// Keypad mapping
// =========================
static const char keypad_map[KEYPAD_ROWS][KEYPAD_COLS] = {
    { '1', '2', '3' },
    { '4', '5', '6' },
    { '7', '8', '9' },
    { '*', '0', '#' }
};

// =========================
// Globals
// =========================
static uint8_t scan_rate_ms = 20;  // default scan delay

// =========================
// Init function
// =========================
void Keypad_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // --- Configure row pins as OUTPUT, default HIGH ---
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    for (uint8_t r = 0; r < KEYPAD_ROWS; r++) {
        GPIO_InitStruct.Pin = row_pins[r];
        HAL_GPIO_Init(KEYPAD_PORT, &GPIO_InitStruct);
        HAL_GPIO_WritePin(KEYPAD_PORT, row_pins[r], GPIO_PIN_SET);
    }

    // --- Configure col pins as INPUT with PULLUP ---
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;

    for (uint8_t c = 0; c < KEYPAD_COLS; c++) {
        GPIO_InitStruct.Pin = col_pins[c];
        HAL_GPIO_Init(KEYPAD_PORT, &GPIO_InitStruct);
    }
}

// =========================
// Set scan rate
// =========================
void Keypad_Set_Scan_Rate(uint8_t rate)
{
    if (rate > 0) {
        scan_rate_ms = rate;   // FIXED
    }
}

// =========================
// Raw scan (no debounce)
// =========================
static char Keypad_Scan_Raw(void)
{
    for (uint8_t row = 0; row < KEYPAD_ROWS; row++)
    {
        // drive current row LOW
        HAL_GPIO_WritePin(KEYPAD_PORT, row_pins[row], GPIO_PIN_RESET);

        // check all columns
        for (uint8_t col = 0; col < KEYPAD_COLS; col++)
        {
            if (HAL_GPIO_ReadPin(KEYPAD_PORT, col_pins[col]) == GPIO_PIN_RESET)
            {
                HAL_Delay(5); // simple debounce
                // release row
                HAL_GPIO_WritePin(KEYPAD_PORT, row_pins[row], GPIO_PIN_SET);
                return keypad_map[row][col];
            }
        }

        // release row
        HAL_GPIO_WritePin(KEYPAD_PORT, row_pins[row], GPIO_PIN_SET);
    }

    return KEYPAD_NO_KEY;
}

// =========================
// Public scan with debounce
// =========================
char Keypad_Scan(void)
{
    static char last_key = KEYPAD_NO_KEY;
    char key = Keypad_Scan_Raw();

    if (key != KEYPAD_NO_KEY && last_key == KEYPAD_NO_KEY) {
        last_key = key;
        return key;   // new key press
    }
    else if (key == KEYPAD_NO_KEY) {
        last_key = KEYPAD_NO_KEY; // released
    }

    HAL_Delay(scan_rate_ms);
    return KEYPAD_NO_KEY;
}

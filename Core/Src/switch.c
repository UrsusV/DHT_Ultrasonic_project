#include "switch.h"

void Switch_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

uint8_t Switch_Read_Mode(void)
{
    // Read multiple samples for debouncing
    uint8_t readings = 0;
    for (uint8_t i = 0; i < 5; i++)
    {
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == GPIO_PIN_RESET)
            readings++;
        HAL_Delay(10);
    }
    return (readings >= 3) ? 2 : 1; // Majority vote: 3/5 LOW → Motor Mode
}

/*
 * 002LEDButton.c
 *
 *  Created on: Jan 24, 2023
 *      Author: hackr6
 */


#include <stdint.h>
#include <string.h>
#include "STM32f446xx.h"
#include "STM32f446xx_GPIO.h"
#include "STM32f446xx_RCC.h"

int main3 (void)
{
    GPIO_Handle_t gpioLED, gpioButton;

    memset(&gpioLED, 0, sizeof(gpioLED));
    memset(&gpioButton, 0, sizeof(gpioButton));

    /* Configure LED GPIO */
    gpioLED.pGPIOx = GPIOA;

    gpioLED.GPIO_PinConfig.GPIO_PinNumber  = 5;
    gpioLED.GPIO_PinConfig.GPIO_PinMode    = GPIO_MODE_OUT;
    gpioLED.GPIO_PinConfig.GPIO_PinSpeed   = GPIO_SPEED_FA;
    gpioLED.GPIO_PinConfig.GPIO_PinOPType  = GPIO_OUTPUT_PP;
    gpioLED.GPIO_PinConfig.GPIO_PinPUPDCtl = GPIO_PUPD_NN;

    /* Configure Push Button (B1 USER @ PC13) */
    gpioButton.pGPIOx = GPIOC;

    gpioButton.GPIO_PinConfig.GPIO_PinNumber = 13;
    gpioButton.GPIO_PinConfig.GPIO_PinMode   = GPIO_MODE_IN;
    gpioButton.GPIO_PinConfig.GPIO_PinSpeed  = GPIO_SPEED_FA;
    
    /* Enable Clocks to both GPIO registers */
    GPIO_ClockCtl(gpioLED.pGPIOx, ENABLE);
    GPIO_ClockCtl(gpioButton.pGPIOx, ENABLE);

    /* Init GPIO devices */
    GPIO_Init(&gpioLED);
    GPIO_Init(&gpioButton);

    /* Pins */
    uint8_t ledPin    = gpioLED.GPIO_PinConfig.GPIO_PinNumber;
    uint8_t buttonPin = gpioButton.GPIO_PinConfig.GPIO_PinNumber;

    /* State var */
    uint8_t buttonState;

    while(1)
    {
        /* Set pin state based on button state */
        buttonState = GPIO_ReadPin(gpioButton.pGPIOx, buttonPin);

        /* Toggle Logic */
        if (buttonState == RESET) {
            GPIO_TogglePin(gpioLED.pGPIOx, ledPin);
        	}
        else {
            GPIO_WritePin(gpioLED.pGPIOx, ledPin, RESET);
        }
    }

    return 0;
}

/*
 * gpio.c
 *
 */

#include "fsl_gpio.h"

#include "gpio.h"

void gpio_set_swcan_mode(uint8_t mode) {
	// invert the mode bits since inverted logic is used

	// set SWCAN_MODE0
	GPIO_WritePinOutput(GPIO, GPIO_SWCAN_MODE0_PORT, GPIO_SWCAN_MODE0_PIN, ~(mode & 0x01));

	// set SWCAN_MODE1
	GPIO_WritePinOutput(GPIO, GPIO_SWCAN_MODE1_PORT, GPIO_SWCAN_MODE1_PIN, ~((mode & 0x01) >> 1));
}

void gpio_set_swcan_enable(uint8_t enable) {
	if (enable) {
		// set SWCAN_ENABLE low to enable
		GPIO_WritePinOutput(GPIO, GPIO_SWCAN_ENABLE_PORT, GPIO_SWCAN_ENABLE_PIN, 0);
	} else {
		// set SWCAN_ENABLE high to disable
		GPIO_WritePinOutput(GPIO, GPIO_SWCAN_ENABLE_PORT, GPIO_SWCAN_ENABLE_PIN, 1);
	}
}

void gpio_set_led(uint8_t led, uint8_t state) {
	switch (led) {
	case GPIO_LED_1:
		GPIO_WritePinOutput(GPIO, GPIO_LED_1_PORT, GPIO_LED_1_PIN, state);
		break;
	case GPIO_LED_2:
		GPIO_WritePinOutput(GPIO, GPIO_LED_2_PORT, GPIO_LED_2_PIN, state);
		break;
	case GPIO_LED_3:
		GPIO_WritePinOutput(GPIO, GPIO_LED_3_PORT, GPIO_LED_3_PIN, state);
		break;
	case GPIO_LED_4:
		GPIO_WritePinOutput(GPIO, GPIO_LED_4_PORT, GPIO_LED_4_PIN, state);
		break;
	}
}

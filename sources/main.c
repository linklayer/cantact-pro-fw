/*
 * The Clear BSD License
 * Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016 - 2017 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 * that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "fsl_device_registers.h"
#include "clock_config.h"
#include "board.h"

#include <stdio.h>
#include <stdlib.h>

#include "usb_device_config.h"
#include "usb.h"
#include "usb_device.h"

#include "usb_device_class.h"
#include "usb_device_ch9.h"
#include "fsl_debug_console.h"

#include "usb_device_descriptor.h"
#if (defined(FSL_FEATURE_SOC_SYSMPU_COUNT) && (FSL_FEATURE_SOC_SYSMPU_COUNT > 0U))
#include "fsl_sysmpu.h"
#endif /* FSL_FEATURE_SOC_SYSMPU_COUNT */

#if defined(USB_DEVICE_CONFIG_EHCI) && (USB_DEVICE_CONFIG_EHCI > 0)
#include "usb_phy.h"
#endif
#if defined(FSL_FEATURE_USB_KHCI_KEEP_ALIVE_ENABLED) && (FSL_FEATURE_USB_KHCI_KEEP_ALIVE_ENABLED > 0U) && \
    defined(USB_DEVICE_CONFIG_KEEP_ALIVE_MODE) && (USB_DEVICE_CONFIG_KEEP_ALIVE_MODE > 0U) &&             \
    defined(FSL_FEATURE_USB_KHCI_USB_RAM) && (FSL_FEATURE_USB_KHCI_USB_RAM > 0U)
extern uint8_t USB_EnterLowpowerMode(void);
#endif
#include "pin_mux.h"
#include <stdbool.h>
#include "fsl_power.h"
#include "fsl_mcan.h"

#include "can.h"
#include "gpio.h"
#include "gs_usb_class.h"
#include "main.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void
BOARD_InitHardware(void);
void
USB_DeviceClockInit(void);
void
USB_DeviceIsrEnable(void);
#if USB_DEVICE_CONFIG_USE_TASK
void USB_DeviceTaskFn(void *deviceHandle);
#endif

void
BOARD_DbgConsole_Deinit(void);
void
BOARD_DbgConsole_Init(void);

/*******************************************************************************
 * Variables
 ******************************************************************************/

static struct gs_host_frame tx_frames[TX_FRAME_BUF_SIZE];
static uint32_t tx_frames_index;
static struct gs_host_frame rx_frames[RX_FRAME_BUF_SIZE];
static uint32_t rx_frames_index;

#if defined(FSL_FEATURE_USB_KHCI_KEEP_ALIVE_ENABLED) && (FSL_FEATURE_USB_KHCI_KEEP_ALIVE_ENABLED > 0U) && \
    defined(USB_DEVICE_CONFIG_KEEP_ALIVE_MODE) && (USB_DEVICE_CONFIG_KEEP_ALIVE_MODE > 0U) &&             \
    defined(FSL_FEATURE_USB_KHCI_USB_RAM) && (FSL_FEATURE_USB_KHCI_USB_RAM > 0U)
volatile static uint8_t s_waitForDataReceive = 0;
volatile static uint8_t s_comOpen = 0;
#endif
/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief Application initialization function.
 *
 * This function initializes the application.
 *
 * @return None.
 */
void APPInit(void) {
	gpio_set_led(GPIO_LED_1, 1);
	gpio_set_led(GPIO_LED_2, 1);
	gpio_set_led(GPIO_LED_3, 1);
	gpio_set_led(GPIO_LED_4, 1);

	gpio_set_swcan_enable(0);

	gs_usb_init();
}

/*!
 * @brief Application task function.
 *
 * This function runs the task for application.
 *
 * @return None.
 */
void APPTask(void) {
	uint32_t primask;

	while (rx_frames_index > 0) {
		primask = DisableGlobalIRQ();
		rx_frames_index--;
		usb_send((uint8_t*) &rx_frames[rx_frames_index],
				sizeof(struct gs_host_frame));
		EnableGlobalIRQ(primask);
	}
}

#if defined(__CC_ARM) || defined(__GNUC__)
int main(void)
#else
void main(void)
#endif
{
	/* attach 12 MHz clock to FLEXCOMM0 (debug console) */
	CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);
	BOARD_InitPins();
	BOARD_BootClockFROHF96M();
	BOARD_InitDebugConsole();

	/* Set MCAN clocks to 96/2=48MHz. */
	CLOCK_SetClkDiv(kCLOCK_DivCan0Clk, 2U, true);
	CLOCK_SetClkDiv(kCLOCK_DivCan1Clk, 2U, true);

#if (defined USB_DEVICE_CONFIG_LPCIP3511HS) && (USB_DEVICE_CONFIG_LPCIP3511HS)
	POWER_DisablePD(kPDRUNCFG_PD_USB1_PHY);
	/* enable usb1 host clock */
	CLOCK_EnableClock(kCLOCK_Usbh1);
	/*According to reference mannual, device mode setting has to be set by access usb host register */
	*((uint32_t *) (USBHSH_BASE + 0x50)) |= USBHSH_PORTMODE_DEV_ENABLE_MASK;
	/* enable usb1 host clock */
	CLOCK_DisableClock(kCLOCK_Usbh1);
#endif
#if (defined USB_DEVICE_CONFIG_LPCIP3511FS) && (USB_DEVICE_CONFIG_LPCIP3511FS)
	POWER_DisablePD(kPDRUNCFG_PD_USB0_PHY); /*< Turn on USB Phy */
	CLOCK_SetClkDiv(kCLOCK_DivUsb0Clk, 1, false);
	CLOCK_AttachClk(kFRO_HF_to_USB0_CLK);
	/* enable usb0 host clock */
	CLOCK_EnableClock(kCLOCK_Usbhsl0);
	/*According to reference mannual, device mode setting has to be set by access usb host register */
	*((uint32_t *)(USBFSH_BASE + 0x5C)) |= USBFSH_PORTMODE_DEV_ENABLE_MASK;
	/* disable usb0 host clock */
	CLOCK_DisableClock(kCLOCK_Usbhsl0);
#endif

	APPInit();

	while (1) {

		APPTask();
	}
}

int tx_enqueue(struct gs_host_frame *frame) {
	tx_frames_index++;
	memcpy(&tx_frames[tx_frames_index], frame, sizeof(struct gs_host_frame));
	return 0;
}

int rx_enqueue(uint8_t channel, mcan_rx_buffer_frame_t *frame) {
	uint32_t primask;

	primask = DisableGlobalIRQ();

	// echo id -1 for normal rx
	rx_frames[rx_frames_index].echo_id = -1;
	rx_frames[rx_frames_index].channel = channel;
	if (frame->xtd) {
		// extended ID, add extended bit
		rx_frames[rx_frames_index].can_id = (frame->id) | 0x80000000;
	} else {
		// standard ID, bit shift
		rx_frames[rx_frames_index].can_id = (frame->id) >> STDID_OFFSET;
	}
	rx_frames[rx_frames_index].can_dlc = frame->dlc;
	memcpy(rx_frames[rx_frames_index].data, frame->data, frame->dlc);
	rx_frames_index++;

	EnableGlobalIRQ(primask);

	return 0;
}

int rx_enqueue_echo(struct gs_host_frame *frame) {
	uint32_t primask;

	primask = DisableGlobalIRQ();

	memcpy(&rx_frames[rx_frames_index], frame, sizeof(struct gs_host_frame));
	rx_frames_index++;

	EnableGlobalIRQ(primask);

	return 0;
}

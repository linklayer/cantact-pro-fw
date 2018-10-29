/*
 * gs_usb_class.c
 *
 *  Created on: Oct 27, 2018
 *      Author: eric
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

#include "gs_usb.h"
#include "can.h"
#include "usb_device_descriptor.h"
#include "gs_usb_class.h"
#include "main.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/

/* Data structure of gs_usb device */
static usb_can_t s_usb_can;

// data structure for can device
static struct gs_usb_dev can_dev;

// device information
static const struct gs_device_config device_config = { 0, // reserved 1
		0, // reserved 2
		0, // reserved 3
		1, // interface count (0=1, 1=2..)
		2, // software version
		1  // hardware version
		};

// bit timing constraints
static const struct gs_device_bt_const can_dev_bt_const = {
GS_CAN_FEATURE_LISTEN_ONLY  // supported features
| GS_CAN_FEATURE_LOOP_BACK | GS_CAN_FEATURE_HW_TIMESTAMP
		| GS_CAN_FEATURE_IDENTIFY, 48000000, // can timing base clock
		1, // tseg1 min
		16, // tseg1 max
		1, // tseg2 min
		8, // tseg2 max
		4, // sjw max
		1, // brp min
		1024, //brp_max
		1, // brp increment;
		};

/* Data buffer for receiving and sending*/
USB_DMA_INIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE)
static uint8_t s_currRecvBuf[DATA_BUFF_SIZE];
static uint32_t s_usbBulkMaxPacketSize = FS_GS_USB_BULK_OUT_PACKET_SIZE;

// buffer for incoming setup control requests
static uint8_t setup_out[CAN_CMD_PACKET_SIZE];

extern usb_device_class_struct_t g_UsbDeviceConfig;
static usb_device_class_config_struct_t s_usbClassConfig[1] = { {
		USB_ClassCallback, 0, &g_UsbDeviceConfig, } };
static usb_device_class_config_list_struct_t s_usbClassConfigList = {
		s_usbClassConfig, USB_DeviceCallback, 1, };

/*******************************************************************************
 * Code
 ******************************************************************************/
#if (defined(USB_DEVICE_CONFIG_LPCIP3511FS) && (USB_DEVICE_CONFIG_LPCIP3511FS > 0U))
void USB0_IRQHandler(void)
{
	USB_DeviceLpcIp3511IsrFunction(s_usb_can.deviceHandle);
}
#endif
#if (defined(USB_DEVICE_CONFIG_LPCIP3511HS) && (USB_DEVICE_CONFIG_LPCIP3511HS > 0U))
void USB1_IRQHandler(void) {
	USB_DeviceLpcIp3511IsrFunction(s_usb_can.deviceHandle);
}
#endif

void USB_DeviceClockInit(void) {
#if defined(USB_DEVICE_CONFIG_LPCIP3511FS) && (USB_DEVICE_CONFIG_LPCIP3511FS > 0U)
	/* enable USB IP clock */
	CLOCK_EnableUsbfs0DeviceClock(kCLOCK_UsbSrcFro, CLOCK_GetFroHfFreq());
#if defined(FSL_FEATURE_USB_USB_RAM) && (FSL_FEATURE_USB_USB_RAM)
	for (int i = 0; i < FSL_FEATURE_USB_USB_RAM; i++)
	{
		((uint8_t *)FSL_FEATURE_USB_USB_RAM_BASE_ADDRESS)[i] = 0x00U;
	}
#endif

#endif
#if defined(USB_DEVICE_CONFIG_LPCIP3511HS) && (USB_DEVICE_CONFIG_LPCIP3511HS > 0U)
	/* enable USB IP clock */
	CLOCK_EnableUsbhs0DeviceClock(kCLOCK_UsbSrcUsbPll, 0U);
#if defined(FSL_FEATURE_USBHSD_USB_RAM) && (FSL_FEATURE_USBHSD_USB_RAM)
	for (int i = 0; i < FSL_FEATURE_USBHSD_USB_RAM; i++) {
		((uint8_t *) FSL_FEATURE_USBHSD_USB_RAM_BASE_ADDRESS)[i] = 0x00U;
	}
#endif
#endif
}
void USB_DeviceIsrEnable(void) {
	uint8_t irqNumber;
#if defined(USB_DEVICE_CONFIG_LPCIP3511FS) && (USB_DEVICE_CONFIG_LPCIP3511FS > 0U)
	uint8_t usbDeviceIP3511Irq[] = USB_IRQS;
	irqNumber = usbDeviceIP3511Irq[CONTROLLER_ID - kUSB_ControllerLpcIp3511Fs0];
#endif
#if defined(USB_DEVICE_CONFIG_LPCIP3511HS) && (USB_DEVICE_CONFIG_LPCIP3511HS > 0U)
	uint8_t usbDeviceIP3511Irq[] = USBHSD_IRQS;
	irqNumber = usbDeviceIP3511Irq[CONTROLLER_ID - kUSB_ControllerLpcIp3511Hs0];
#endif
	/* Install isr, set priority, and enable IRQ. */
	NVIC_SetPriority((IRQn_Type) irqNumber, USB_DEVICE_INTERRUPT_PRIORITY);
	EnableIRQ((IRQn_Type) irqNumber);
}
#if USB_DEVICE_CONFIG_USE_TASK
void USB_DeviceTaskFn(void *deviceHandle)
{
#if defined(USB_DEVICE_CONFIG_LPCIP3511FS) && (USB_DEVICE_CONFIG_LPCIP3511FS > 0U)
	USB_DeviceLpcIp3511TaskFunction(deviceHandle);
#endif
#if defined(USB_DEVICE_CONFIG_LPCIP3511HS) && (USB_DEVICE_CONFIG_LPCIP3511HS > 0U)
	USB_DeviceLpcIp3511TaskFunction(deviceHandle);
#endif
}
#endif

usb_status_t USB_ClassCallback(class_handle_t handle, uint32_t event,
		void *param) {
	return kStatus_USB_Success;
}

/*!
 * @brief Vendor specific callback function.
 *
 * This function handles the vendor specific requests.
 *
 * @param handle The USB device handle.
 * @param setup The pointer to the setup packet.
 * @param length The pointer to the length of the data buffer.
 * @param buffer The pointer to the address of setup packet data buffer.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceProcessVendorRequest(usb_device_handle handle,
		usb_device_control_request_struct_t *control_request) {
	struct gs_device_bittiming *bit_timing;
	struct gs_device_mode *mode;
	mcan_timing_config_t mcan_timing_config;
	uint32_t tmp_32;

	usb_status_t error = kStatus_USB_Success;
	usb_setup_struct_t setup = *control_request->setup;
	// wValue contains CAN channel to operate on
	uint8_t channel = setup.wValue;

	// host to device request with no buffer, provide buffer for request
	if ((setup.bmRequestType & 0x80) == 0
			&& (control_request->buffer == NULL)) {
		control_request->buffer = setup_out;
		return error;
	}

	switch (setup.bRequest) {
	// requests setting device state
	case GS_USB_BREQ_HOST_FORMAT:
		memcpy((uint8_t *) &can_dev.host_config, control_request->buffer,
				sizeof(can_dev.host_config));
		break;
	case GS_USB_BREQ_BITTIMING:
		bit_timing = (struct gs_device_bittiming *) control_request->buffer;

		// for NXP driver, seg1 = phase_seg1 + prop_seg
		mcan_timing_config.seg1 = bit_timing->phase_seg1 + bit_timing->prop_seg;
		mcan_timing_config.seg2 = bit_timing->phase_seg2;
		mcan_timing_config.preDivider = bit_timing->brp;
		mcan_timing_config.rJumpwidth = bit_timing->sjw;

		if (can_set_timing(channel, &mcan_timing_config) != 0) {
			error = kStatus_USB_InvalidParameter;
		}
		break;
	case GS_USB_BREQ_MODE:
		mode = (struct gs_device_mode *) control_request->buffer;
		if (mode->mode == GS_CAN_MODE_RESET) {
			if (can_stop(channel) != 0) {
				error = kStatus_USB_InvalidParameter;
			}
		} else if (mode->mode == GS_CAN_MODE_START) {
			if (can_start(channel) != 0) {
				error = kStatus_USB_InvalidParameter;
			}
		}
		break;
	case GS_USB_BREQ_IDENTIFY:
		memcpy(&tmp_32, control_request->buffer, sizeof(tmp_32));
		// TODO identify
		break;
	case GS_USB_BREQ_BERR:
		// TODO ?
		break;

		// requests reading device state
	case GS_USB_BREQ_DEVICE_CONFIG:
		control_request->buffer = (uint8_t *) &device_config;
		control_request->length = sizeof(device_config);
		break;
	case GS_USB_BREQ_BT_CONST:
		control_request->buffer = (uint8_t *) &can_dev_bt_const;
		control_request->length = sizeof(can_dev_bt_const);
		break;
	case GS_USB_BREQ_TIMESTAMP:
		control_request->buffer = (uint8_t *) &can_dev.sof_timestamp_us;
		control_request->length = sizeof(can_dev.sof_timestamp_us);
		break;

	default:
		// unknown bRequest value
		error = kStatus_USB_InvalidRequest;
		break;
	}

	return error;
}

/*!
 * @brief Bulk in pipe callback function.
 *
 * This function serves as the callback function for bulk in pipe.
 *
 * @param handle The USB device handle.
 * @param message The endpoint callback message
 * @param callbackParam The parameter of the callback.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceBulkIn(usb_device_handle handle,
		usb_device_endpoint_callback_message_struct_t *message,
		void *callbackParam) {
	usb_status_t error = kStatus_USB_Success;

	if ((message->length != 0)
			&& (!(message->length % s_usbBulkMaxPacketSize))) {
		/* If the last packet is the size of endpoint, then send also zero-ended packet,
		 ** meaning that we want to inform the host that we do not have any additional
		 ** data, so it can flush the output.
		 */
		USB_DeviceSendRequest(handle, GSUSB_ENDPOINT_IN, NULL, 0);
	} else if ((message->buffer != NULL)
			|| ((message->buffer == NULL) && (message->length == 0))) {
		/* User: add your own code for send complete event */
		/* Schedule buffer for next receive event */
		USB_DeviceRecvRequest(handle, GSUSB_ENDPOINT_OUT, s_currRecvBuf,
				s_usbBulkMaxPacketSize);
#if defined(FSL_FEATURE_USB_KHCI_KEEP_ALIVE_ENABLED) && (FSL_FEATURE_USB_KHCI_KEEP_ALIVE_ENABLED > 0U) && \
defined(USB_DEVICE_CONFIG_KEEP_ALIVE_MODE) && (USB_DEVICE_CONFIG_KEEP_ALIVE_MODE > 0U) &&             \
defined(FSL_FEATURE_USB_KHCI_USB_RAM) && (FSL_FEATURE_USB_KHCI_USB_RAM > 0U)
		s_waitForDataReceive = 1;
		USB0->INTEN &= ~USB_INTEN_SOFTOKEN_MASK;
#endif
	}

	return error;
}

/*!
 * @brief Bulk out pipe callback function.
 *
 * This function serves as the callback function for bulk out pipe.
 *
 * @param handle The USB device handle.
 * @param message The endpoint callback message
 * @param callbackParam The parameter of the callback.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceBulkOut(usb_device_handle handle,
		usb_device_endpoint_callback_message_struct_t *message,
		void *callbackParam) {
	usb_status_t error = kStatus_USB_Success;

	struct gs_host_frame *frame = (struct gs_host_frame *) message->buffer;
	//tx_enqueue(frame);
	can_send(frame->channel, frame->echo_id, frame->can_id, frame->can_dlc,
			frame->data);
	// echo the frame back to the host
	rx_enqueue_echo(frame);

	/* Schedule buffer for next receive event */
	USB_DeviceRecvRequest(handle, GSUSB_ENDPOINT_OUT, s_currRecvBuf,
			s_usbBulkMaxPacketSize);

#if defined(FSL_FEATURE_USB_KHCI_KEEP_ALIVE_ENABLED) && (FSL_FEATURE_USB_KHCI_KEEP_ALIVE_ENABLED > 0U) && \
    defined(USB_DEVICE_CONFIG_KEEP_ALIVE_MODE) && (USB_DEVICE_CONFIG_KEEP_ALIVE_MODE > 0U) &&             \
    defined(FSL_FEATURE_USB_KHCI_USB_RAM) && (FSL_FEATURE_USB_KHCI_USB_RAM > 0U)
	s_waitForDataReceive = 0;
	USB0->INTEN |= USB_INTEN_SOFTOKEN_MASK;
#endif

#if defined(FSL_FEATURE_USB_KHCI_KEEP_ALIVE_ENABLED) && (FSL_FEATURE_USB_KHCI_KEEP_ALIVE_ENABLED > 0U) && \
defined(USB_DEVICE_CONFIG_KEEP_ALIVE_MODE) && (USB_DEVICE_CONFIG_KEEP_ALIVE_MODE > 0U) &&             \
defined(FSL_FEATURE_USB_KHCI_USB_RAM) && (FSL_FEATURE_USB_KHCI_USB_RAM > 0U)
	s_waitForDataReceive = 1;
	USB0->INTEN &= ~USB_INTEN_SOFTOKEN_MASK;
#endif
	return error;
}

/*!
 * @brief USB device callback function.
 *
 * This function handles the usb device specific requests.
 *
 * @param handle          The USB device handle.
 * @param event           The USB device event type.
 * @param param           The parameter of the device specific request.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceCallback(usb_device_handle handle, uint32_t event,
		void *param) {
	usb_status_t error = kStatus_USB_Success;
	uint8_t *temp8 = (uint8_t *) param;

	switch (event) {
	case kUSB_DeviceEventBusReset: {
		//USB_DeviceControlPipeInit(s_usb_can.deviceHandle);
		s_usb_can.attach = 0;
#if (defined(USB_DEVICE_CONFIG_EHCI) && (USB_DEVICE_CONFIG_EHCI > 0U)) || \
    (defined(USB_DEVICE_CONFIG_LPCIP3511HS) && (USB_DEVICE_CONFIG_LPCIP3511HS > 0U))
		/* Get USB speed to configure the device, including max packet size and interval of the endpoints. */
		if (kStatus_USB_Success
				== USB_DeviceGetStatus(s_usb_can.deviceHandle,
						kUSB_DeviceStatusSpeed, &s_usb_can.speed)) {
			USB_DeviceSetSpeed(handle, s_usb_can.speed);
		}
#endif
	}
		break;
	case kUSB_DeviceEventSetConfiguration:
		if (param) {
			s_usb_can.attach = 1;
			s_usb_can.currentConfiguration = *temp8;
			if (USB_GS_USB_CONFIGURE_INDEX == (*temp8)) {
				usb_device_endpoint_init_struct_t epInitStruct;
				usb_device_endpoint_callback_struct_t endpointCallback;

				/* initialize endpoints for bulk pipe */
				endpointCallback.callbackFn = USB_DeviceBulkIn;
				endpointCallback.callbackParam = handle;

				epInitStruct.zlt = 0;
				epInitStruct.transferType = USB_ENDPOINT_BULK;
				epInitStruct.endpointAddress = GSUSB_ENDPOINT_IN;

				if (USB_SPEED_HIGH == s_usb_can.speed) {
					epInitStruct.maxPacketSize = HS_GS_USB_BULK_IN_PACKET_SIZE;
				} else {
					epInitStruct.maxPacketSize = FS_GS_USB_BULK_IN_PACKET_SIZE;
				}

				USB_DeviceInitEndpoint(s_usb_can.deviceHandle, &epInitStruct,
						&endpointCallback);

				endpointCallback.callbackFn = USB_DeviceBulkOut;
				endpointCallback.callbackParam = handle;

				epInitStruct.zlt = 0;
				epInitStruct.transferType = USB_ENDPOINT_BULK;
				epInitStruct.endpointAddress = GSUSB_ENDPOINT_OUT;
				if (USB_SPEED_HIGH == s_usb_can.speed) {
					epInitStruct.maxPacketSize = HS_GS_USB_BULK_OUT_PACKET_SIZE;
				} else {
					epInitStruct.maxPacketSize = FS_GS_USB_BULK_OUT_PACKET_SIZE;
				}

				USB_DeviceInitEndpoint(s_usb_can.deviceHandle, &epInitStruct,
						&endpointCallback);

				if (USB_SPEED_HIGH == s_usb_can.speed) {
					s_usbBulkMaxPacketSize = HS_GS_USB_BULK_OUT_PACKET_SIZE;
				} else {
					s_usbBulkMaxPacketSize = FS_GS_USB_BULK_OUT_PACKET_SIZE;
				}
				/* Schedule buffer for receive */
				USB_DeviceRecvRequest(handle, GSUSB_ENDPOINT_OUT, s_currRecvBuf,
						s_usbBulkMaxPacketSize);
			}
		}
		break;
	case kUSB_DeviceEventGetDeviceDescriptor:
		if (param) {
			error = USB_DeviceGetDeviceDescriptor(handle,
					(usb_device_get_device_descriptor_struct_t *) param);
		}
		break;
	case kUSB_DeviceEventGetConfigurationDescriptor:
		if (param) {
			error = USB_DeviceGetConfigurationDescriptor(handle,
					(usb_device_get_configuration_descriptor_struct_t *) param);
		}
		break;
	case kUSB_DeviceEventGetStringDescriptor:
		if (param) {
			/* Get device string descriptor request */
			error = USB_DeviceGetStringDescriptor(handle,
					(usb_device_get_string_descriptor_struct_t *) param);
		}
		break;
	case kUSB_DeviceEventVendorRequest:
		USB_DeviceProcessVendorRequest(handle,
				(usb_device_control_request_struct_t *) param);
		break;
	default:
		usb_echo("idk %d\n", event);
		break;
	}

	return error;
}

void usb_send(uint8_t *buf, uint32_t len) {
	USB_DeviceSendRequest(s_usb_can.deviceHandle, GSUSB_ENDPOINT_IN, buf, len);
}

void gs_usb_init() {
	USB_DeviceClockInit();
#if (defined(FSL_FEATURE_SOC_SYSMPU_COUNT) && (FSL_FEATURE_SOC_SYSMPU_COUNT > 0U))
	SYSMPU_Enable(SYSMPU, 0);
#endif /* FSL_FEATURE_SOC_SYSMPU_COUNT */

	s_usb_can.speed = USB_SPEED_FULL;
	s_usb_can.attach = 0;
	s_usb_can.deviceHandle = NULL;

	if (kStatus_USB_Success
			!= USB_DeviceClassInit(CONTROLLER_ID, &s_usbClassConfigList,
					&s_usb_can.deviceHandle)) {
		usb_echo("USB device init failed\r\n");
	} else {
		usb_echo("CANtact Pro USB started\r\n");
	}

	USB_DeviceIsrEnable();

	USB_DeviceRun(s_usb_can.deviceHandle);
}

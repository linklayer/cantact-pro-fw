/*
 * The Clear BSD License
 * Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016 NXP
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
#include "usb_device_config.h"
#include "usb.h"
#include "usb_device.h"

#include "usb_device_class.h"
#include "usb_device_cdc_acm.h"

#include "usb_device_descriptor.h"

/*******************************************************************************
* Variables
******************************************************************************/
/* Define endpoint for communication class */
usb_device_endpoint_struct_t g_UsbDeviceCdcVcomCicEndpoints[USB_CDC_VCOM_ENDPOINT_CIC_COUNT] = {
    {
        USB_CDC_VCOM_INTERRUPT_IN_ENDPOINT | (USB_IN << 7U), USB_ENDPOINT_INTERRUPT,
        FS_CDC_VCOM_INTERRUPT_IN_PACKET_SIZE,
    },
};

/* Define endpoint for data class */
usb_device_endpoint_struct_t g_UsbDeviceCdcVcomDicEndpoints[USB_CDC_VCOM_ENDPOINT_DIC_COUNT] = {
    {
    	GSUSB_ENDPOINT_IN | (USB_IN << 7U), USB_ENDPOINT_BULK, CAN_DATA_MAX_PACKET_SIZE,
    },
    {
    	GSUSB_ENDPOINT_OUT | (USB_OUT << 7U), USB_ENDPOINT_BULK, CAN_DATA_MAX_PACKET_SIZE,
    }};

/* Define interface for data class */
usb_device_interface_struct_t g_UsbDeviceCdcVcomDataInterface[] = {
    {0,
     {
         USB_CDC_VCOM_ENDPOINT_DIC_COUNT, g_UsbDeviceCdcVcomDicEndpoints,
     },
     NULL}};

/* Define interfaces for virtual com */
usb_device_interfaces_struct_t g_UsbDeviceCdcVcomInterfaces[USB_CDC_VCOM_INTERFACE_COUNT] = {
    {USB_DEVICE_CLASS, USB_DEVICE_SUBCLASS, USB_DEVICE_PROTOCOL, 1,
     g_UsbDeviceCdcVcomDataInterface, sizeof(g_UsbDeviceCdcVcomDataInterface) / sizeof(usb_device_interfaces_struct_t)},
};

/* Define configurations for virtual com */
usb_device_interface_list_t g_UsbDeviceCdcVcomInterfaceList[USB_DEVICE_CONFIGURATION_COUNT] = {
    {
        USB_CDC_VCOM_INTERFACE_COUNT, g_UsbDeviceCdcVcomInterfaces,
    },
};

/* Define class information for virtual com */
usb_device_class_struct_t g_UsbDeviceConfig = {
    g_UsbDeviceCdcVcomInterfaceList, kUSB_DeviceClassTypeCdc, USB_DEVICE_CONFIGURATION_COUNT,
};

/* Define device descriptor */
USB_DMA_INIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE)
uint8_t g_UsbDeviceDescriptor[] = {
    /* Size of this descriptor in bytes */
    USB_DESCRIPTOR_LENGTH_DEVICE,
    /* DEVICE Descriptor Type */
    USB_DESCRIPTOR_TYPE_DEVICE,
    /* USB Specification Release Number in Binary-Coded Decimal (i.e., 2.10 is 210H). */
    USB_SHORT_GET_LOW(USB_DEVICE_SPECIFIC_BCD_VERSION), USB_SHORT_GET_HIGH(USB_DEVICE_SPECIFIC_BCD_VERSION),
    /* Class code (assigned by the USB-IF). */
    USB_DEVICE_CLASS,
    /* Subclass code (assigned by the USB-IF). */
    USB_DEVICE_SUBCLASS,
    /* Protocol code (assigned by the USB-IF). */
    USB_DEVICE_PROTOCOL,
    /* Maximum packet size for endpoint zero (only 8, 16, 32, or 64 are valid) */
    USB_CONTROL_MAX_PACKET_SIZE,
    /* Vendor ID (assigned by the USB-IF) */
	USB_SHORT_GET_LOW(USB_CAN_VID), USB_SHORT_GET_HIGH(USB_CAN_VID),
    /* Product ID (assigned by the manufacturer) */
	USB_SHORT_GET_LOW(USB_CAN_PID), USB_SHORT_GET_HIGH(USB_CAN_PID),
    /* Device release number in binary-coded decimal */
    USB_SHORT_GET_LOW(USB_DEVICE_DEMO_BCD_VERSION), USB_SHORT_GET_HIGH(USB_DEVICE_DEMO_BCD_VERSION),
    /* Index of string descriptor describing manufacturer */
    0x01,
    /* Index of string descriptor describing product */
    0x02,
    /* Index of string descriptor describing the device's serial number */
    0x00,
    /* Number of possible configurations */
    USB_DEVICE_CONFIGURATION_COUNT,
};

/* Define configuration descriptor */
USB_DMA_INIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE)
uint8_t g_UsbDeviceConfigurationDescriptor[] = {
    /* Size of this descriptor in bytes */
    USB_DESCRIPTOR_LENGTH_CONFIGURE,
    /* CONFIGURATION Descriptor Type */
    USB_DESCRIPTOR_TYPE_CONFIGURE,
    /* Total length of data returned for this configuration. */
    USB_SHORT_GET_LOW(USB_DESCRIPTOR_LENGTH_CONFIGURE + USB_DESCRIPTOR_LENGTH_INTERFACE +
                      USB_DESCRIPTOR_LENGTH_ENDPOINT + USB_DESCRIPTOR_LENGTH_ENDPOINT),
    USB_SHORT_GET_HIGH(USB_DESCRIPTOR_LENGTH_CONFIGURE + USB_DESCRIPTOR_LENGTH_INTERFACE +
                       USB_DESCRIPTOR_LENGTH_ENDPOINT + USB_DESCRIPTOR_LENGTH_ENDPOINT),
    /* Number of interfaces supported by this configuration */
	USB_GS_USB_INTERFACE_COUNT,
    /* Value to use as an argument to the SetConfiguration() request to select this configuration */
	USB_GS_USB_CONFIGURE_INDEX,
    /* Index of string descriptor describing this configuration */
    0,
    /* Configuration characteristics D7: Reserved (set to one) D6: Self-powered D5: Remote Wakeup D4...0: Reserved
       (reset to zero) */
    (USB_DESCRIPTOR_CONFIGURE_ATTRIBUTE_D7_MASK) |
        (USB_DEVICE_CONFIG_SELF_POWER << USB_DESCRIPTOR_CONFIGURE_ATTRIBUTE_SELF_POWERED_SHIFT) |
        (USB_DEVICE_CONFIG_REMOTE_WAKEUP << USB_DESCRIPTOR_CONFIGURE_ATTRIBUTE_REMOTE_WAKEUP_SHIFT),
    /* Maximum power consumption of the USB * device from the bus in this specific * configuration when the device is
       fully * operational. Expressed in 2 mA units *  (i.e., 50 = 100 mA).  */
    USB_DEVICE_MAX_POWER,

    /* Communication Interface Descriptor */
    USB_DESCRIPTOR_LENGTH_INTERFACE, USB_DESCRIPTOR_TYPE_INTERFACE, USB_GS_USB_INTERFACE_INDEX, 0x00,
	USB_GS_USB_ENDPOINT_COUNT, USB_DEVICE_CLASS, USB_DEVICE_SUBCLASS, USB_DEVICE_PROTOCOL,
    0x00, /* Interface Description String Index*/

    /* EP1 Descriptor */
	USB_DESCRIPTOR_LENGTH_GS_USB_EP,				/* bLength */
	USB_DESCRIPTOR_TYPE_ENDPOINT,  					/* bDescriptorType */
	GSUSB_ENDPOINT_IN,                				/* bEndpointAddress */
	0x02,                             				/* bmAttributes: bulk */
	USB_SHORT_GET_LOW(CAN_DATA_MAX_PACKET_SIZE),	/* wMaxPacketSize */
	USB_SHORT_GET_HIGH(CAN_DATA_MAX_PACKET_SIZE),
	0x00,                             				/* bInterval */

	/* EP2 Descriptor */
	USB_DESCRIPTOR_LENGTH_GS_USB_EP,				/* bLength */
	USB_DESCRIPTOR_TYPE_ENDPOINT,  					/* bDescriptorType */
	GSUSB_ENDPOINT_OUT,                				/* bEndpointAddress */
	0x02,                             				/* bmAttributes: bulk */
	USB_SHORT_GET_LOW(CAN_DATA_MAX_PACKET_SIZE),	/* wMaxPacketSize */
	USB_SHORT_GET_HIGH(CAN_DATA_MAX_PACKET_SIZE),
	0x00,                             				/* bInterval */
};

/* Define string descriptor */
USB_DMA_INIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE)
uint8_t g_UsbDeviceString0[] = {2U + 2U, USB_DESCRIPTOR_TYPE_STRING, 0x09, 0x04};

USB_DMA_INIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE)
uint8_t g_UsbDeviceString1[] = {
    2U + 2U * 14U, USB_DESCRIPTOR_TYPE_STRING,
    'L',           0x00U,
    'i',           0x00U,
    'n',           0x00U,
    'k',           0x00U,
    'L',           0x00U,
    'a',           0x00U,
    'y',           0x00U,
    'e',           0x00U,
    'r',           0x00U,
    ' ',           0x00U,
    'L',           0x00U,
    'a',           0x00U,
    'b',           0x00U,
    's',           0x00U,
};

USB_DMA_INIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE)
uint8_t g_UsbDeviceString2[] = {2U + 2U * 11U, USB_DESCRIPTOR_TYPE_STRING,
                                'C',           0,
                                'A',           0,
                                'N',           0,
                                't',           0,
                                'a',           0,
                                'c',           0,
                                't',           0,
                                ' ',           0,
                                'P',           0,
                                'r',           0,
                                'o',           0};

uint8_t *g_UsbDeviceStringDescriptorArray[USB_DEVICE_STRING_COUNT] = {g_UsbDeviceString0, g_UsbDeviceString1,
                                                                      g_UsbDeviceString2};

/* Define string descriptor size */
uint32_t g_UsbDeviceStringDescriptorLength[USB_DEVICE_STRING_COUNT] = {
    sizeof(g_UsbDeviceString0), sizeof(g_UsbDeviceString1), sizeof(g_UsbDeviceString2)};
usb_language_t g_UsbDeviceLanguage[USB_DEVICE_LANGUAGE_COUNT] = {{
    g_UsbDeviceStringDescriptorArray, g_UsbDeviceStringDescriptorLength, (uint16_t)0x0409,
}};

usb_language_list_t g_UsbDeviceLanguageList = {
    g_UsbDeviceString0, sizeof(g_UsbDeviceString0), g_UsbDeviceLanguage, USB_DEVICE_LANGUAGE_COUNT,
};

/*******************************************************************************
* Code
******************************************************************************/
/*!
 * @brief USB device get device descriptor function.
 *
 * This function gets the device descriptor of the USB device.
 *
 * @param handle The USB device handle.
 * @param deviceDescriptor The pointer to the device descriptor structure.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceGetDeviceDescriptor(usb_device_handle handle,
                                           usb_device_get_device_descriptor_struct_t *deviceDescriptor)
{
    deviceDescriptor->buffer = g_UsbDeviceDescriptor;
    deviceDescriptor->length = USB_DESCRIPTOR_LENGTH_DEVICE;
    return kStatus_USB_Success;
}

/*!
 * @brief USB device get configuration descriptor function.
 *
 * This function gets the configuration descriptor of the USB device.
 *
 * @param handle The USB device handle.
 * @param configurationDescriptor The pointer to the configuration descriptor structure.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceGetConfigurationDescriptor(
    usb_device_handle handle, usb_device_get_configuration_descriptor_struct_t *configurationDescriptor)
{
    if (USB_CDC_VCOM_CONFIGURE_INDEX > configurationDescriptor->configuration)
    {
        configurationDescriptor->buffer = g_UsbDeviceConfigurationDescriptor;
        configurationDescriptor->length = USB_DESCRIPTOR_LENGTH_CONFIGURATION_ALL;
        return kStatus_USB_Success;
    }
    return kStatus_USB_InvalidRequest;
}

/*!
 * @brief USB device get string descriptor function.
 *
 * This function gets the string descriptor of the USB device.
 *
 * @param handle The USB device handle.
 * @param stringDescriptor Pointer to the string descriptor structure.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceGetStringDescriptor(usb_device_handle handle,
                                           usb_device_get_string_descriptor_struct_t *stringDescriptor)
{
    if (stringDescriptor->stringIndex == 0U)
    {
        stringDescriptor->buffer = (uint8_t *)g_UsbDeviceLanguageList.languageString;
        stringDescriptor->length = g_UsbDeviceLanguageList.stringLength;
    }
    else
    {
        uint8_t languageId = 0U;
        uint8_t languageIndex = USB_DEVICE_STRING_COUNT;

        for (; languageId < USB_DEVICE_LANGUAGE_COUNT; languageId++)
        {
            if (stringDescriptor->languageId == g_UsbDeviceLanguageList.languageList[languageId].languageId)
            {
                if (stringDescriptor->stringIndex < USB_DEVICE_STRING_COUNT)
                {
                    languageIndex = stringDescriptor->stringIndex;
                }
                break;
            }
        }

        if (USB_DEVICE_STRING_COUNT == languageIndex)
        {
            return kStatus_USB_InvalidRequest;
        }
        stringDescriptor->buffer = (uint8_t *)g_UsbDeviceLanguageList.languageList[languageId].string[languageIndex];
        stringDescriptor->length = g_UsbDeviceLanguageList.languageList[languageId].length[languageIndex];
    }
    return kStatus_USB_Success;
}

/*!
 * @brief USB device set speed function.
 *
 * This function sets the speed of the USB device.
 *
 * Due to the difference of HS and FS descriptors, the device descriptors and configurations need to be updated to match
 * current speed.
 * As the default, the device descriptors and configurations are configured by using FS parameters for both EHCI and
 * KHCI.
 * When the EHCI is enabled, the application needs to call this fucntion to update device by using current speed.
 * The updated information includes endpoint max packet size, endpoint interval, etc.
 *
 * @param handle The USB device handle.
 * @param speed Speed type. USB_SPEED_HIGH/USB_SPEED_FULL/USB_SPEED_LOW.
 *
 * @return A USB error code or kStatus_USB_Success.
 */
usb_status_t USB_DeviceSetSpeed(usb_device_handle handle, uint8_t speed)
{
    usb_descriptor_union_t *ptr1;
    usb_descriptor_union_t *ptr2;

    ptr1 = (usb_descriptor_union_t *)(&g_UsbDeviceConfigurationDescriptor[0]);
    ptr2 = (usb_descriptor_union_t *)(&g_UsbDeviceConfigurationDescriptor[USB_DESCRIPTOR_LENGTH_CONFIGURATION_ALL - 1]);

    while (ptr1 < ptr2)
    {
        if (ptr1->common.bDescriptorType == USB_DESCRIPTOR_TYPE_ENDPOINT)
        {
            if (USB_SPEED_HIGH == speed)
            {
                if (((ptr1->endpoint.bEndpointAddress & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK) ==
                     USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_IN) &&
                     (GSUSB_ENDPOINT_IN == ptr1->endpoint.bEndpointAddress))
                {
                    ptr1->endpoint.bInterval = HS_CDC_VCOM_INTERRUPT_IN_INTERVAL;
                    USB_SHORT_TO_LITTLE_ENDIAN_ADDRESS(CAN_DATA_MAX_PACKET_SIZE,
                                                       ptr1->endpoint.wMaxPacketSize);
                }
                else if (((ptr1->endpoint.bEndpointAddress & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK) ==
                     USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_OUT) &&
                         (GSUSB_ENDPOINT_OUT == ptr1->endpoint.bEndpointAddress))
                {
                    USB_SHORT_TO_LITTLE_ENDIAN_ADDRESS(CAN_DATA_MAX_PACKET_SIZE, ptr1->endpoint.wMaxPacketSize);
                }
                else
                {
                }
            }
            else
            {
                if (((ptr1->endpoint.bEndpointAddress & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK) ==
                     USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_IN) &&
                     (GSUSB_ENDPOINT_IN == ptr1->endpoint.bEndpointAddress))
                {
                    ptr1->endpoint.bInterval = FS_CDC_VCOM_INTERRUPT_IN_INTERVAL;
                    USB_SHORT_TO_LITTLE_ENDIAN_ADDRESS(CAN_DATA_MAX_PACKET_SIZE,
                                                       ptr1->endpoint.wMaxPacketSize);
                }
                else if (((ptr1->endpoint.bEndpointAddress & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK) ==
                     USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_OUT) &&
                         (GSUSB_ENDPOINT_OUT == ptr1->endpoint.bEndpointAddress))
                {
                    USB_SHORT_TO_LITTLE_ENDIAN_ADDRESS(CAN_DATA_MAX_PACKET_SIZE, ptr1->endpoint.wMaxPacketSize);
                }
                else
                {
                }
            }
        }
        ptr1 = (usb_descriptor_union_t *)((uint8_t *)ptr1 + ptr1->common.bLength);
    }

    for (int i = 0; i < USB_CDC_VCOM_ENDPOINT_CIC_COUNT; i++)
    {
        if (USB_SPEED_HIGH == speed)
        {
            g_UsbDeviceCdcVcomCicEndpoints[i].maxPacketSize = CAN_DATA_MAX_PACKET_SIZE;
        }
        else
        {
            g_UsbDeviceCdcVcomCicEndpoints[i].maxPacketSize = CAN_DATA_MAX_PACKET_SIZE;
        }
    }
    for (int i = 0; i < USB_CDC_VCOM_ENDPOINT_DIC_COUNT; i++)
    {
        if (USB_SPEED_HIGH == speed)
        {
            if (g_UsbDeviceCdcVcomDicEndpoints[i].endpointAddress & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK)
            {
                g_UsbDeviceCdcVcomDicEndpoints[i].maxPacketSize = CAN_DATA_MAX_PACKET_SIZE;
            }
            else
            {
                g_UsbDeviceCdcVcomDicEndpoints[i].maxPacketSize = CAN_DATA_MAX_PACKET_SIZE;
            }
        }
        else
        {
            if (g_UsbDeviceCdcVcomDicEndpoints[i].endpointAddress & USB_DESCRIPTOR_ENDPOINT_ADDRESS_DIRECTION_MASK)
            {
                g_UsbDeviceCdcVcomDicEndpoints[i].maxPacketSize = CAN_DATA_MAX_PACKET_SIZE;
            }
            else
            {
                g_UsbDeviceCdcVcomDicEndpoints[i].maxPacketSize = CAN_DATA_MAX_PACKET_SIZE;
            }
        }
    }

    return kStatus_USB_Success;
}

/*
 * gs_usb.h
 */

#ifndef GS_USB_H_
#define GS_USB_H_

#include "stdint.h"

/* Device specific constants */
enum gs_usb_breq {
	GS_USB_BREQ_HOST_FORMAT = 0,
	GS_USB_BREQ_BITTIMING,
	GS_USB_BREQ_MODE,
	GS_USB_BREQ_BERR,
	GS_USB_BREQ_BT_CONST,
	GS_USB_BREQ_DEVICE_CONFIG,
	GS_USB_BREQ_TIMESTAMP,
	GS_USB_BREQ_IDENTIFY,
};

enum gs_can_mode {
	/* reset a channel. turns it off */
	GS_CAN_MODE_RESET = 0,
	/* starts a channel */
	GS_CAN_MODE_START
};

enum gs_can_state {
	GS_CAN_STATE_ERROR_ACTIVE = 0,
	GS_CAN_STATE_ERROR_WARNING,
	GS_CAN_STATE_ERROR_PASSIVE,
	GS_CAN_STATE_BUS_OFF,
	GS_CAN_STATE_STOPPED,
	GS_CAN_STATE_SLEEPING
};

enum gs_can_identify_mode {
	GS_CAN_IDENTIFY_OFF = 0,
	GS_CAN_IDENTIFY_ON
};

/* data types passed between host and device */
struct gs_host_config {
	uint32_t byte_order;
};
/* All data exchanged between host and device is exchanged in host byte order,
 * thanks to the struct gs_host_config byte_order member, which is sent first
 * to indicate the desired byte order.
 */

struct gs_device_config {
	uint8_t reserved1;
	uint8_t reserved2;
	uint8_t reserved3;
	uint8_t icount;
	uint32_t sw_version;
	uint32_t hw_version;
};

#define GS_CAN_MODE_NORMAL               0
#define GS_CAN_MODE_LISTEN_ONLY          BIT(0)
#define GS_CAN_MODE_LOOP_BACK            BIT(1)
#define GS_CAN_MODE_TRIPLE_SAMPLE        BIT(2)
#define GS_CAN_MODE_ONE_SHOT             BIT(3)

struct gs_device_mode {
	uint32_t mode;
	uint32_t flags;
};

struct gs_device_state {
	uint32_t state;
	uint32_t rxerr;
	uint32_t txerr;
};

struct gs_device_bittiming {
	uint32_t prop_seg;
	uint32_t phase_seg1;
	uint32_t phase_seg2;
	uint32_t sjw;
	uint32_t brp;
};

struct gs_identify_mode {
	uint32_t mode;
};

#define BIT(X) (1<<X)
#define GS_CAN_FEATURE_LISTEN_ONLY      BIT(0)
#define GS_CAN_FEATURE_LOOP_BACK        BIT(1)
#define GS_CAN_FEATURE_TRIPLE_SAMPLE    BIT(2)
#define GS_CAN_FEATURE_ONE_SHOT         BIT(3)
#define GS_CAN_FEATURE_HW_TIMESTAMP     BIT(4)
#define GS_CAN_FEATURE_IDENTIFY         BIT(5)

struct gs_device_bt_const {
	uint32_t feature;
	uint32_t fclk_can;
	uint32_t tseg1_min;
	uint32_t tseg1_max;
	uint32_t tseg2_min;
	uint32_t tseg2_max;
	uint32_t sjw_max;
	uint32_t brp_min;
	uint32_t brp_max;
	uint32_t brp_inc;
};

#define GS_CAN_FLAG_OVERFLOW 1

struct gs_host_frame {
	uint32_t echo_id;
	uint32_t can_id;

	uint8_t can_dlc;
	uint8_t channel;
	uint8_t flags;
	uint8_t reserved;

	uint8_t data[8];
};

struct gs_usb_dev {
	struct gs_host_config host_config;
	uint32_t sof_timestamp_us;
};

#endif /* GS_USB_H_ */

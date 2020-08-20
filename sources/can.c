/*
 * can.c
 *
 */

#include "can.h"
#include "gpio.h"
#include "main.h"

#define MCAN0_CLK_FREQ CLOCK_GetFreq(kCLOCK_MCAN0)
#define MCAN1_CLK_FREQ CLOCK_GetFreq(kCLOCK_MCAN1)

#define CAN_RAM_BASE 			0x20010000U
#define CAN0_STD_FILTER_OFS 	0x00
#define CAN0_RX_FIFO0_OFS 		0x10
#define CAN0_TX_BUFFER_OFS 		0x20
#define CAN1_STD_FILTER_OFS 	0x100
#define CAN1_RX_FIFO0_OFS 		0x110
#define CAN1_TX_BUFFER_OFS 		0x120

static CAN_Type* can_devs[CAN_NUM_CHANNELS] = {
CAN0,
CAN1 };
static mcan_timing_config_t timing_configs[CAN_NUM_CHANNELS];
static uint8_t channel_enabled[CAN_NUM_CHANNELS];
static uint8_t monitor_mode[CAN_NUM_CHANNELS];
static uint8_t identify[CAN_NUM_CHANNELS];
static uint64_t rx_count[CAN_NUM_CHANNELS];
static uint64_t tx_count[CAN_NUM_CHANNELS];

void CAN0_IRQ0_IRQHandler(void) {
	mcan_rx_buffer_frame_t rx_frame;

	if (MCAN_GetStatusFlag(CAN0, CAN_IR_RF0N_MASK)) {
		MCAN_ClearStatusFlag(CAN0, CAN_IR_RF0N_MASK);
		MCAN_ReadRxFifo(CAN0, 0, &rx_frame);
		rx_enqueue(0, &rx_frame);
		rx_count[0]++;
	} else if (MCAN_GetStatusFlag(CAN0, CAN_IR_TC_MASK)) {
		MCAN_ClearStatusFlag(CAN0, CAN_IR_TC_MASK);
	}
}

void CAN1_IRQ0_IRQHandler(void) {
	mcan_rx_buffer_frame_t rx_frame;

	if (MCAN_GetStatusFlag(CAN1, CAN_IR_RF0N_MASK)) {
		MCAN_ClearStatusFlag(CAN1, CAN_IR_RF0N_MASK);
		MCAN_ReadRxFifo(CAN1, 0, &rx_frame);
		rx_enqueue(1, &rx_frame);
		rx_count[1]++;
	} else if (MCAN_GetStatusFlag(CAN1, CAN_IR_TC_MASK)) {
		MCAN_ClearStatusFlag(CAN1, CAN_IR_TC_MASK);
	}
}

uint64_t can_get_rx_count(uint8_t channel) {
	if (channel >= CAN_NUM_CHANNELS) {
		return 0;
	}
	return rx_count[channel];
}

uint64_t can_get_tx_count(uint8_t channel) {
	if (channel >= CAN_NUM_CHANNELS) {
		return 0;
	}
	return tx_count[channel];
}

uint8_t can_get_enabled(uint8_t channel) {
	if (channel >= CAN_NUM_CHANNELS) {
		return 0;
	}
	return channel_enabled[channel];
}

void can_set_identify(uint8_t channel, uint8_t enable) {
	if (channel >= CAN_NUM_CHANNELS) {
		return;
	}
	identify[channel] = enable;
}
uint8_t can_get_identify(uint8_t channel) {
	if (channel >= CAN_NUM_CHANNELS) {
		return 0;
	}
	return identify[channel];
}

uint8_t can_get_monitor_mode(uint8_t channel) {
	if (channel >= CAN_NUM_CHANNELS) {
		return 0;
	}
	return monitor_mode[channel];
}

int can_set_timing(uint8_t channel, mcan_timing_config_t *timing_config) {
	if (channel >= CAN_NUM_CHANNELS) {
		return -1;
	}
	memcpy(&timing_configs[channel], timing_config, sizeof(mcan_timing_config_t));

	// hardware expects these to be one less than value
	timing_configs[channel].preDivider -= 1;
	timing_configs[channel].seg1 -= 1;
	timing_configs[channel].seg2 -= 1;
	timing_configs[channel].rJumpwidth -= 1;

	return 0;
}

int can_start(uint8_t channel, uint32_t flags) {
	mcan_config_t config;
	mcan_rx_fifo_config_t rxFifo0;
	mcan_tx_buffer_config_t txBuffer;
	CAN_Type *can_dev;

	if (channel >= CAN_NUM_CHANNELS) {
		return -1;
	}
	can_dev = can_devs[channel];

	MCAN_GetDefaultConfig(&config);
	monitor_mode[channel] = 0;
	if (flags & GS_CAN_FEATURE_LISTEN_ONLY) {
		config.enableBusMon = true;
		monitor_mode[channel] = 1;
	}
	if (flags & GS_CAN_FEATURE_LOOP_BACK) {
		config.enableLoopBackExt = true;
	}

	if (can_dev == CAN0) {
		MCAN_Init(can_dev, &config, MCAN0_CLK_FREQ);
	} else if (can_dev == CAN1) {
		MCAN_Init(can_dev, &config, MCAN1_CLK_FREQ);
	}

	channel_enabled[channel] = 1;

	// set timing configuration
	// this must happen after MCAN_Init or it will be overridden
	MCAN_SetArbitrationTimingConfig(can_dev, &timing_configs[channel]);
	MCAN_SetDataTimingConfig(can_dev, &timing_configs[channel]);

	/* Set Message RAM base address and clear to avoid BEU/BEC error. */
	MCAN_SetMsgRAMBase(can_dev, CAN_RAM_BASE);
	if (can_dev == CAN0) {
		uint32_t *p = (uint32_t *) (CAN_RAM_BASE);
		memset(p, 0, 0x100U);
	} else if (can_dev == CAN1) {
		uint32_t *p = (uint32_t *) (CAN_RAM_BASE + CAN1_STD_FILTER_OFS);
		memset(p, 0, 0x100);
	}

	/* RX fifo0 config. */
	if (can_dev == CAN0) {
		rxFifo0.address = CAN0_RX_FIFO0_OFS;
	} else {
		rxFifo0.address = CAN1_RX_FIFO0_OFS;
	}
	rxFifo0.elementSize = 1U;
	rxFifo0.watermark = 0;
	rxFifo0.opmode = kMCAN_FifoBlocking;
	rxFifo0.datafieldSize = kMCAN_8ByteDatafield;
	MCAN_SetRxFifo0Config(can_dev, &rxFifo0);

	/* TX buffer config */
	if (can_dev == CAN0) {
		txBuffer.address = CAN0_TX_BUFFER_OFS;
	} else {
		txBuffer.address = CAN1_TX_BUFFER_OFS;
	}
	txBuffer.dedicatedSize = 16U;
	txBuffer.fqSize = 16U;
	txBuffer.datafieldSize = kMCAN_8ByteDatafield;
	MCAN_SetTxBufferConfig(can_dev, &txBuffer);

	/* Enable RX fifo0 new message interrupt using interrupt line 0. */
	MCAN_EnableInterrupts(can_dev, 0, CAN_IE_RF0NE_MASK);
	if (can_dev == CAN0) {
		EnableIRQ(CAN0_IRQ0_IRQn);
	} else if (can_dev == CAN1) {
		EnableIRQ(CAN1_IRQ0_IRQn);
	}

	/* Enter normal mode. */
	MCAN_EnterNormalMode(can_dev);

	return 0;
}

int can_stop(uint8_t channel) {
	CAN_Type* can_dev;

	if (channel >= CAN_NUM_CHANNELS) {
		return -1;
	}
	can_dev = can_devs[channel];
	channel_enabled[channel] = 0;

	MCAN_Deinit(can_dev);

	return 0;
}

int can_send(uint8_t channel, uint8_t buf, uint32_t can_id, uint8_t can_dlc,
		uint8_t *can_data) {
	CAN_Type* can_dev;
	mcan_handle_t handle;
	mcan_buffer_transfer_t xfer;
	mcan_tx_buffer_frame_t frame;

	if (channel >= CAN_NUM_CHANNELS) {
		return -1;
	}
	if (!channel_enabled[channel]) {
		return -1;
	}
	can_dev = can_devs[channel];

	MCAN_TransferCreateHandle(can_dev, &handle, NULL, NULL);

	// check extended bit
	if (can_id & 0x80000000) {
		// extended frame, remove extended bit
		frame.id = can_id & 0x7FFFFFFF;
		frame.xtd = kMCAN_FrameIDExtend;
	} else {
		// standard ID, bit shift for MCAN FIFO
		frame.id = can_id << STDID_OFFSET;
		frame.xtd = kMCAN_FrameIDStandard;
	}
	// check RTR bit
	if (can_id & 0x40000000) {
		// RTR frame, remove RTR bit
		frame.rtr = kMCAN_FrameTypeRemote;
		frame.id = frame.id & 0x3FFFFFFF;
	} else {
		frame.rtr = kMCAN_FrameTypeData;
	}

	frame.dlc = can_dlc;
	frame.size = can_dlc;
	frame.data = can_data;

	frame.fdf = 0;
	frame.brs = 0;

	xfer.frame = &frame;
	xfer.bufferIdx = buf;

	MCAN_TransferSendNonBlocking(can_dev, &handle, &xfer);

	tx_count[channel]++;

	return 0;
}

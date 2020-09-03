/*
 * can.h
 *
 */

#ifndef CAN_H_
#define CAN_H_

#include "fsl_mcan.h"

#define CAN_NUM_CHANNELS 2
#define STDID_OFFSET 		18U

uint64_t can_get_rx_count(uint8_t channel);
uint64_t can_get_tx_count(uint8_t channel);
uint8_t can_get_enabled(uint8_t channel);
void can_set_identify(uint8_t channel, uint8_t enable);
uint8_t can_get_identify(uint8_t channel);
uint8_t can_get_monitor_mode(uint8_t channel);
int can_set_timing(uint8_t index, mcan_timing_config_t *timing_config);
int can_set_data_timing(uint8_t index, mcan_timing_config_t *timing_config);
int can_len2dlc(uint8_t len);
int can_dlc2len(uint8_t dlc);
int can_start(uint8_t index, uint32_t flags);
int can_stop(uint8_t index);
int can_send(uint8_t channel, uint8_t buf, uint32_t can_id, uint8_t flags, uint8_t can_dlc,
		uint8_t *can_data);

#endif /* CAN_H_ */

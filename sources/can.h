/*
 * can.h
 *
 */

#ifndef CAN_H_
#define CAN_H_

#include "fsl_mcan.h"

#define CAN_NUM_CHANNELS 2
#define STDID_OFFSET 		18U

int can_set_timing(uint8_t index, mcan_timing_config_t *timing_config);
int can_start(uint8_t index);
int can_stop(uint8_t index);

#endif /* CAN_H_ */

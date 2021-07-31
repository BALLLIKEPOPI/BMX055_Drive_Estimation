#ifndef _BMX055_H_
#define _BMX055_H_

#include "bma2x2.h"
#include "bmg160.h"
#include "bmm150.h"
#include "bmm050.h"

void bmx_055_init(void);
s32 bma2x2_data_readout(struct bma2x2_accel_data *xyzt);
s32 bmm050_data_readout(struct bmm050_mag_data_s16_t *data_s32);
s32 bmg160_data_readout(struct bmg160_data_t *gyro_xyzi_data);

#endif

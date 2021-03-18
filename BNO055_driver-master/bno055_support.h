#ifndef __BNO055_SUPPORT_H__
#define __BNO055_SUPPORT_H__
#include "bno055.h"

s32 bno055_data_readout_template(void);
s8 I2C_routine(void);
s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
void BNO055_delay_msek(u32 msek);

#endif

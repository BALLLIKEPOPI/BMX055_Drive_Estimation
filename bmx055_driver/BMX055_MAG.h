/* Code written by Chia Jiun Wei @ 28 Aug 2017
 * <J.W.Chia@tudelft.nl>
 
 * BMX055: a library to provide high level APIs to interface with 
 * the Bosch Absolute Orientation Sensors magnetometer. It is  
 * possible to use this library in Energia (the Arduino port for 
 * MSP microcontrollers) or in other toolchains.
 
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License 
 * version 3, both as published by the Free Software Foundation.
  
 */
 
#ifndef BMX055_MAG_H
#define BMX055_MAG_H

#include <Energia.h>
#include <DSPI.h>

#ifdef __cplusplus
extern "C"
{
#endif

// C header here
#include "bmm050.h"

#ifdef __cplusplus
}
#endif
	
	//Read write function interface with Bosch Magnetometer bmm050 API
	s8 readRegister(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 size);
	s8 writeRegister(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 size);
	void delay_msek(u32 msek);
	
class BMX055_MAG

{
protected:	
	
	bmm050_t bmm050;
	
public:
	
	BMX055_MAG(DSPI *spi, unsigned char pin);
	virtual ~BMX055_MAG( ) {};
	
	void init();
	signed char ping();
	void get_mag(s16 *data_array);
	
private:	
	
};


#endif  // BMX055_MAG_H
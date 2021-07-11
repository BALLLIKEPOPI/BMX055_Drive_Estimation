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
 
#include "BMX055_MAG.h"	

DSPI *line;
	
/**  BMX055 Magnetometer class creator function
 *
 *   Parameters:
 *   DSPI *spi             SPI object
 *	 unsigned char pin	   Chip select GPIO pin
 *
 */
BMX055_MAG::BMX055_MAG(DSPI *spi, unsigned char pin)
{
	line = spi;
	bmm050.bus_write = writeRegister;
	bmm050.bus_read = readRegister;
	bmm050.delay_msec = delay_msek;
	bmm050.dev_addr = pin;	//SPI: CS GPIO, I2C: device address
}

/**  Initialise BMX055 Magnetometer
 *	 Initialise to active and NORMAL mode, interrupt disable, ODR default 10Hz
 *
 */
void BMX055_MAG::init()
{	
	pinMode(bmm050.dev_addr, OUTPUT);	//Chip select GPIO as output 
	
	bmm050_init(&bmm050);
	bmm050_set_functional_state(BMM050_NORMAL_MODE);
}

/**  Verify if BMX055 is present
 *
 *   Returns:
 *   true                  BMX055 is present
 *   false                 otherwise
 *
 */
signed char BMX055_MAG::ping()
{	
	return (line->transfer(BMM050_CHIP_ID) == 0x32);
}

/**  Get magnetometer measurement
 *
 *   Parameters:
 *   s16 *data_array       Array of 3 to store magnetometer measurement
 *
 */
void BMX055_MAG::get_mag(s16 *data_array)
{	
	bmm050_mag_data_s16_t data;
	
	bmm050_read_mag_data_XYZ(&data);
	
	data.datax = data_array[0];
	data.datay = data_array[1];
	data.dataz = data_array[2];
}


/*** Bosch bmm050 interface function ***/

/**  Read sensor register
 *		
 *	 Parameters:
 *	 u8 dev_addr		I2C: device address, SPI: Chip select GPIO pin
 *   u8 reg_addr		register address
 *   u8 *reg_data		buffer to store read data
 *   u8 size			total bytes to be read
 *
 *   Returns:
 *   Does not return anything, s8 return is implemented to match with bmm050 function
 *
 */
s8 readRegister(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 size)
{
	digitalWrite(dev_addr, LOW);
	line->transfer(reg_addr);
	
	for (unsigned int i = 0; i < size; i++)
		*(reg_data + i) = line->transfer(0x00);
	
	digitalWrite(dev_addr, HIGH);	
}

/**  Write sensor register
 *		
 *	 Parameters:
 *	 u8 dev_addr		I2C: device address, SPI: Chip select GPIO pin
 *   u8 reg_addr		register address
 *   u8 *reg_data		buffer to store data to be written
 *   u8 size			total bytes to be write
 *
 *   Returns:
 *   Does not return anything, s8 return is implemented to match with bmm050 function
 *
 */
s8 writeRegister(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 size)
{
	digitalWrite(dev_addr, LOW);	
	line->transfer(reg_addr);
	
	for (unsigned int i = 0; i < size; i++)		
		line->transfer(*(reg_data + i));
	
	digitalWrite(dev_addr, HIGH);	
}

/**  Delay milli-second
 *		
 *	 Parameters:
 *   u32 msek			milli-second
 *
 */
void delay_msek(u32 msek)
{
	delay(msek);
}

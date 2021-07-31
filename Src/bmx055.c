#include "bmx055.h"
#include "spi.h"
#include "stm32f1xx_hal.h"

#define	I2C_BUFFER_LEN 8
#define SPI_BUFFER_LEN 5
#define BMA2x2_BUS_READ_WRITE_ARRAY_INDEX	1
#define BMA2x2_SPI_BUS_WRITE_CONTROL_BYTE	0x7F
#define BMA2x2_SPI_BUS_READ_CONTROL_BYTE	0x80

/*----------------------------------------------------------------------------*
*	The following functions are used for reading and writing of
*	sensor data using I2C or SPI communication
*----------------------------------------------------------------------------*/
/* \Brief: The function is used as SPI bus write
 * \Return : Status of the SPI write
 * \param dev_addr : The device address of the sensor
 * \param reg_addr : Address of the first register,
 *      will data is going to be written
 * \param reg_data : It is a value hold in the array,
 *	will be used for write the value into the register
 * \param cnt : The no of byte of data to be write
 */
s8 BMA2x2_SPI_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
/* \Brief: The function is used as SPI bus read
 * \Return : Status of the SPI read
 * \param dev_addr : The device address of the sensor
 * \param reg_addr : Address of the first register,
 *   will data is going to be read
 * \param reg_data : This data read from the sensor, which is hold in an array
 * \param cnt : The no of byte of data to be read */
s8 BMA2x2_SPI_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
/*
 * \Brief: SPI/I2C init routine
*/
s8 SPI_routine(void);
s8 BMM050_SPI_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s8 BMM050_SPI_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s8 BMG160_SPI_bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t length);
s8 BMG160_SPI_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
void BMG160_delay_msek(u32 msek);
/********************End of I2C/SPI function declarations*******************/
/*	Brief : The delay routine
 *	\param : delay in ms
 */
void BMA2x2_delay_msek(u32 msek);
void BMM050_delay_msek(u32 msek);

/*----------------------------------------------------------------------------*
*  struct bma2x2_t parameters can be accessed by using bma2x2
 *	bma2x2_t having the following parameters
 *	Bus write function pointer: BMA2x2_WR_FUNC_PTR
 *	Bus read function pointer: BMA2x2_RD_FUNC_PTR
 *	Burst read function pointer: BMA2x2_BRD_FUNC_PTR
 *	Delay function pointer: delay_msec
 *	I2C address: dev_addr
 *	Chip id of the sensor: chip_id
 *---------------------------------------------------------------------------*/
struct bma2x2_t bma2x2;
struct bmm150_dev bmm150;
struct bmm050 bmm050;
struct bmg160_t bmg160;

/*----------------------------------------------------------------------------*
*  V_BMA2x2RESOLUTION_u8R used for selecting the accelerometer resolution
 *	12 bit
 *	14 bit
 *	10 bit
*----------------------------------------------------------------------------*/
extern u8 V_BMA2x2RESOLUTION_u8R;

void bmx_055_init(void)
{
	// u8 bw_value_u8 = BMA2x2_INIT_VALUE;
	// u8 banwid = BMA2x2_INIT_VALUE;
	u8 v_data_rate_value_u8 = BMM050_ZERO_U8X;

	bma2x2.bus_write = BMA2x2_SPI_bus_write;
	bma2x2.bus_read = BMA2x2_SPI_bus_read;
	bma2x2.delay_msec = BMA2x2_delay_msek;
	bma2x2_init(&bma2x2);
	HAL_Delay(2);
	bma2x2_set_range(BMA2x2_RANGE_2G);
	bma2x2_set_bw(BMA2x2_BW_62_50HZ);
	// bma2x2_set_bw(0x08);
	// bma2x2_set_power_mode(BMA2x2_MODE_NORMAL);
	// bw_value_u8 = 0x08;/* set bandwidth of 7.81Hz*/
	// bma2x2_set_bw(bw_value_u8);
	// bma2x2_get_bw(&banwid);
	// bma2x2_set_range(BMA2x2_RANGE_2G);

	bmm050.bus_write = BMM050_SPI_bus_write;
	bmm050.bus_read = BMM050_SPI_bus_read;
	bmm050.delay_msec = BMM050_delay_msek;
	bmm050_init(&bmm050);
	HAL_Delay(2);
	bmm050_set_presetmode(BMM050_PRESETMODE_REGULAR);	
	bmm050_set_functional_state(BMM050_FORCED_MODE);
	v_data_rate_value_u8 = BMM050_DATA_RATE_30HZ;/* set data rate of 30Hz*/
	bmm050_set_data_rate(v_data_rate_value_u8);

	bmg160.bus_write = BMG160_SPI_bus_write;
	bmg160.bus_read = BMG160_SPI_bus_read;
	bmg160.delay_msec = BMG160_delay_msek;
	bmg160_init(&bmg160);
	HAL_Delay(2);
	bmg160_set_range_reg(0x02);
	bmg160_set_bw(C_BMG160_BW_230HZ_U8X);
}
s32 bma2x2_data_readout(struct bma2x2_accel_data *xyz)
{
	/*Local variables for reading accel x, y and z data*/
	s16	accel_x_s16, accel_y_s16, accel_z_s16 = BMA2x2_INIT_VALUE;
	/* bma2x2acc_data structure used to read accel xyz data*/
	// struct bma2x2_accel_data sample_xyz;
	/* bma2x2acc_data_temp structure used to read
		accel xyz and temperature data*/
	// struct bma2x2_accel_data_temp sample_xyzt;
	/* Local variable used to assign the bandwidth value*/
	// u8 bw_value_u8 = BMA2x2_INIT_VALUE;
	/* Local variable used to set the bandwidth value*/
	// u8 banwid = BMA2x2_INIT_VALUE;
	/* status of communication*/
	s32 com_rslt = ERROR_BMX;
	
	com_rslt += bma2x2_set_power_mode(BMA2x2_MODE_NORMAL);

	/* Read the accel X data*/
	com_rslt += bma2x2_read_accel_x(&accel_x_s16);
	/* Read the accel Y data*/
	com_rslt += bma2x2_read_accel_y(&accel_y_s16);
	/* Read the accel Z data*/
	com_rslt += bma2x2_read_accel_z(&accel_z_s16);

	/* accessing the bma2x2acc_data parameter by using sample_xyz*/
	/* Read the accel XYZ data*/
	com_rslt += bma2x2_read_accel_xyz(xyz);

	/* accessing the bma2x2acc_data_temp parameter by using sample_xyzt*/
	/* Read the accel XYZT data*/
	// com_rslt += bma2x2_read_accel_xyzt(xyzt);
	com_rslt += bma2x2_set_power_mode(BMA2x2_MODE_DEEP_SUSPEND);

	return com_rslt;
}

s32 bmm050_data_readout(struct bmm050_mag_data_s16_t *data)
{
	/* Structure used for read the mag xyz data*/
	// struct bmm050_mag_data_s16_t data;
	/* Structure used for read the mag xyz data with 32 bit output*/
	// struct bmm050_mag_s32_data_t data_s32;
	/* Structure used for read the mag xyz data with float output*/
	// struct bmm050_mag_data_float_t data_float;
	/* Variable used to get the data rate*/
	// u8 v_data_rate_u8 = BMM050_ZERO_U8X;
	/* Variable used to set the data rate*/
	// u8 v_data_rate_value_u8 = BMM050_ZERO_U8X;
	/* result of communication results*/
	s32 com_rslt = ERROR_BMX;

/*---------------------------------------------------------------------------*
 *********************** START INITIALIZATION ************************
 *--------------------------------------------------------------------------*/
 /*	Based on the user need configure I2C or SPI interface.
  *	It is sample code to explain how to use the bmm050 API*/
	// #ifdef BMM050_API
	// I2C_routine();
	// /*SPI_routine(); */
	// #endif
/*--------------------------------------------------------------------------*
 *  This function used to assign the value/reference of
 *	the following parameters
 *	I2C address
 *	Bus Write
 *	Bus read
 *	company_id
*-------------------------------------------------------------------------*/
	// com_rslt = bmm050_init(&bmm050_t);

/*	For initialization it is required to set the mode of
 *	the sensor as "NORMAL"
 *	but before set the mode needs to configure the power control bit
 *	in the register 0x4B bit BMM050_ZERO_U8X should be enabled
 *	This bit is enabled by calling bmm050_init function
 *	For the Normal data acquisition/read/write is possible in this mode
 *	by using the below API able to set the power mode as NORMAL*/
	/* Set the power mode as NORMAL*/
	// com_rslt += bmm050_set_functional_state(BMM050_NORMAL_MODE);
/*--------------------------------------------------------------------------*
************************* END INITIALIZATION *************************
*---------------------------------------------------------------------------*/

/*------------------------------------------------------------------------*
************************* START GET and SET FUNCTIONS DATA ****************
*---------------------------------------------------------------------------*/
	/* This API used to Write the data rate of the sensor, input
	value have to be given
	data rate value set from the register 0x4C bit 3 to 5*/
	// v_data_rate_value_u8 = BMM050_DATA_RATE_30HZ;/* set data rate of 30Hz*/
	// com_rslt += bmm050_set_data_rate(v_data_rate_value_u8);

	/* This API used to read back the written value of data rate*/
	// com_rslt += bmm050_get_data_rate(&v_data_rate_u8);
/*-----------------------------------------------------------------*
************************* END GET and SET FUNCTIONS ****************
*-------------------------------------------------------------------*/
/*------------------------------------------------------------------*
************************* START READ SENSOR DATA(X,Y and Z axis) ********
*------------------------------------------------------------------*/
	/* accessing the bmm050_mdata parameter by using data*/
	com_rslt += bmm050_read_mag_data_XYZ(data);/* Reads the mag x y z data*/


	/* accessing the bmm050_mdata_float parameter by using data_float*/
	// com_rslt += bmm050_read_mag_data_XYZ_float(&data_float);/* Reads mag xyz data output as 32bit value*/

	/* accessing the bmm050_mdata_s32 parameter by using data_s32*/
	// com_rslt += bmm050_read_mag_data_XYZ_s32(&data_s32);/* Reads mag xyz data output as float value*/

/*--------------------------------------------------------------------*
************************* END READ SENSOR DATA(X,Y and Z axis) ************
*-------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------*
************************* START DE-INITIALIZATION ***********************
*-------------------------------------------------------------------------*/
/*	For de-initialization it is required to set the mode of
 *	the sensor as "SUSPEND"
 *	the SUSPEND mode set from the register 0x4B bit BMM050_ZERO_U8X should be disabled
 *	by using the below API able to set the power mode as SUSPEND*/
	/* Set the power mode as SUSPEND*/
	// com_rslt += bmm050_set_functional_state(BMM050_SUSPEND_MODE);
	bmm050_set_functional_state(BMM050_FORCED_MODE);
/*---------------------------------------------------------------------*
************************* END DE-INITIALIZATION **********************
*---------------------------------------------------------------------*/
return com_rslt;
}
s32 bmg160_data_readout(struct bmg160_data_t *gyro_xyzi_data)
{
    /* Gyro */
    /* variable used for read the sensor data*/
    s16 v_gyro_datax_s16, v_gyro_datay_s16, v_gyro_dataz_s16 = BMG160_INIT_VALUE;

    /* structure used for read the sensor data - xyz*/
    // struct bmg160_data_t data_gyro;

    /* structure used for read the sensor data - xyz and interrupt status*/
    // struct bmg160_data_t gyro_xyzi_data;

    /* variable used for read the gyro bandwidth data*/
    // u8 v_gyro_value_u8 = BMG160_INIT_VALUE;

    /* variable used for set the gyro bandwidth data*/
    // u8 v_bw_u8;

    /* result of communication results*/
    s32 com_rslt;

    /*-------------------------------------------------------------------------*
    *********************** START INITIALIZATION ***********************
    *-------------------------------------------------------------------------*/

    /*  Based on the user need configure I2C or SPI interface.
     *  It is example code to explain how to use the bmg160 API
     */
// #ifdef BMG160_API
//     I2C_routine();

//     /*SPI_routine(); */
// #endif

    /*--------------------------------------------------------------------------*
     *  This function used to assign the value/reference of
     *  the following parameters
     *  Gyro I2C address
     *  Bus Write
     *  Bus read
     *  Gyro Chip id
     *----------------------------------------------------------------------------*/
    // com_rslt = bmg160_init(&bmg160);

    /*----------------------------------------------------------------------------*/

    /*  For initialization it is required to set the mode of the sensor as "NORMAL"
     *  data acquisition/read/write is possible in this mode
     *  by using the below API able to set the power mode as NORMAL
     *  NORMAL mode set from the register 0x11 and 0x12
     *  While sensor in the NORMAL mode idle time of at least 2us(micro seconds)
     *  is required to write/read operations
     *  0x11 -> bit 5,7 -> set value as BMG160_INIT_VALUE
     *  0x12 -> bit 6,7 -> set value as BMG160_INIT_VALUE
     *  Note:
     *      If the sensor is in the fast power up mode idle time of least
     *      450us(micro seconds) required for write/read operations
     */

    /*-------------------------------------------------------------------------*/
    /* Set the gyro power mode as NORMAL*/
    com_rslt += bmg160_set_power_mode(BMG160_MODE_NORMAL);

    /*--------------------------------------------------------------------------*
    ************************* END INITIALIZATION ******************************
    *--------------------------------------------------------------------------*/

    /*------------------------------------------------------------------------*
     ************************* START GET and SET FUNCTIONS DATA ***************
     *--------------------------------------------------------------------------*/

    /* This API used to Write the bandwidth of the gyro sensor
     * input value have to be give 0x10 bit BMG160_INIT_VALUE to 3
     * The bandwidth set from the register
     */
    // v_bw_u8 = C_BMG160_BW_230HZ_U8X; /* set gyro bandwidth of 230Hz*/
    // com_rslt += bmg160_set_bw(v_bw_u8);

    /* This API used to read back the written value of bandwidth for gyro*/
    // com_rslt += bmg160_get_bw(&v_gyro_value_u8);

    /*---------------------------------------------------------------------*
     ************************* END GET and SET FUNCTIONS ********************
     *----------------------------------------------------------------------*/

    /*---------------------------------------------------------------------*
     ************************* START READ SENSOR DATA(X,Y and Z axis) *********
     *-------------------------------------------------------------------------*/

    /******************* Read Gyro data xyz**********************/
    com_rslt += bmg160_get_data_x(&v_gyro_datax_s16); /* Read the gyro X data*/
    com_rslt += bmg160_get_data_y(&v_gyro_datay_s16); /* Read the gyro Y data*/
    com_rslt += bmg160_get_data_z(&v_gyro_dataz_s16); /* Read the gyro Z data*/
    /* accessing the  bmg160_data_t parameter by using data_gyro*/
    com_rslt += bmg160_get_data_xyz(gyro_xyzi_data); /* Read the gyro XYZ data*/
    /* accessing the bmg160_data_t parameter by using gyro_xyzi_data*/
    /* Read the gyro XYZ data and interrupt status*/
    // com_rslt += bmg160_get_data_xyzi(gyro_xyzi_data);

    /*--------------------------------------------------------------------------
     ************************* END READ SENSOR DATA(X,Y and Z axis) *************
     *----------------------------------------------------------------------------*/

    /*---------------------------------------------------------------------------*
     *********************** START DE-INITIALIZATION *****************************
     *--------------------------------------------------------------------------*/

    /*  For de-initialization it is required to set the mode of
     *  the sensor as "DEEPSUSPEND"
     *  the device reaches the lowest power consumption only
     *  interface selection is kept alive
     *  No data acquisition is performed
     *  The DEEPSUSPEND mode set from the register 0x11 bit 5
     *  by using the below API able to set the power mode as DEEPSUSPEND
     *  For the read/ write operation it is required to provide least 450us
     *  micro second delay
     */
    com_rslt += bmg160_set_power_mode(BMG160_MODE_DEEPSUSPEND);

    /*--------------------------------------------------------------------------*
     *********************** END DE-INITIALIZATION **************************
     *---------------------------------------------------------------------------*/
    return com_rslt;
}
/* This function is an example for reading sensor data
 *	\param: None
 *	\return: communication result
 */
// s32 bma2x2_data_readout(void)
// {
// 	/*Local variables for reading accel x, y and z data*/
// 	s16	accel_x_s16, accel_y_s16, accel_z_s16 = BMA2x2_INIT_VALUE;

// 	/* bma2x2acc_data structure used to read accel xyz data*/
// 	struct bma2x2_accel_data sample_xyz;
// 	/* bma2x2acc_data_temp structure used to read
// 		accel xyz and temperature data*/
// 	struct bma2x2_accel_data_temp sample_xyzt;
// 	/* Local variable used to assign the bandwidth value*/
// 	u8 bw_value_u8 = BMA2x2_INIT_VALUE;
// 	/* Local variable used to set the bandwidth value*/
// 	u8 banwid = BMA2x2_INIT_VALUE;
// 	/* status of communication*/
// 	s32 com_rslt = ERROR_BMX;


// /*********************** START INITIALIZATION ************************
//   *	Based on the user need configure I2C or SPI interface.
//   *	It is example code to explain how to use the bma2x2 API*/
// 	#ifdef BMA2x2_API
// 	// I2C_routine();
// 	//SPI_routine(); 
// 	#endif
//  /*--------------------------------------------------------------------------*
//  *  This function used to assign the value/reference of
//  *	the following parameters
//  *	I2C address
//  *	Bus Write
//  *	Bus read
//  *	Chip id
//  *-------------------------------------------------------------------------*/
// 	com_rslt = bma2x2_init(&bma2x2);

// /*	For initialization it is required to set the mode of
//  *	the sensor as "NORMAL"
//  *	NORMAL mode is set from the register 0x11 and 0x12
//  *	0x11 -> bit 5,6,7 -> set value as 0
//  *	0x12 -> bit 5,6 -> set value as 0
//  *	data acquisition/read/write is possible in this mode
//  *	by using the below API able to set the power mode as NORMAL
//  *	For the Normal/standby/Low power 2 mode Idle time
// 		of at least 2us(micro seconds)
//  *	required for read/write operations*/
// 	/* Set the power mode as NORMAL*/
// 	com_rslt += bma2x2_set_power_mode(BMA2x2_MODE_NORMAL);
// /*	Note:
// 	* For the Suspend/Low power1 mode Idle time of
// 		at least 450us(micro seconds)
// 	* required for read/write operations*/

// /************************* END INITIALIZATION *************************/

// /*------------------------------------------------------------------------*
// ************************* START GET and SET FUNCTIONS DATA ****************
// *---------------------------------------------------------------------------*/
// 	/* This API used to Write the bandwidth of the sensor input
// 	value have to be given
// 	bandwidth is set from the register 0x10 bits from 1 to 4*/
// 	bw_value_u8 = 0x08;/* set bandwidth of 7.81Hz*/
// 	com_rslt += bma2x2_set_bw(bw_value_u8);

// 	/* This API used to read back the written value of bandwidth*/
// 	com_rslt += bma2x2_get_bw(&banwid);
//     bma2x2_set_range(BMA2x2_RANGE_2G);
// /*-----------------------------------------------------------------*
// ************************* END GET and SET FUNCTIONS ****************
// *-------------------------------------------------------------------*/
// /*------------------------------------------------------------------*
// ************************* START READ SENSOR DATA(X,Y and Z axis) ********
// *---------------------------------------------------------------------*/
// 	/* Read the accel X data*/
// 	com_rslt += bma2x2_read_accel_x(&accel_x_s16);
// 	/* Read the accel Y data*/
// 	com_rslt += bma2x2_read_accel_y(&accel_y_s16);
// 	/* Read the accel Z data*/
// 	com_rslt += bma2x2_read_accel_z(&accel_z_s16);

// 	/* accessing the bma2x2acc_data parameter by using sample_xyz*/
// 	/* Read the accel XYZ data*/
// 	com_rslt += bma2x2_read_accel_xyz(&sample_xyz);

// 	/* accessing the bma2x2acc_data_temp parameter by using sample_xyzt*/
// 	/* Read the accel XYZT data*/
// 	com_rslt += bma2x2_read_accel_xyzt(&sample_xyzt);

// /*--------------------------------------------------------------------*
// ************************* END READ SENSOR DATA(X,Y and Z axis) ************
// *-------------------------------------------------------------------------*/
// /*-----------------------------------------------------------------------*
// ************************* START DE-INITIALIZATION ***********************
// *-------------------------------------------------------------------------*/
// /*	For de-initialization it is required to set the mode of
//  *	the sensor as "DEEP SUSPEND"
//  *	DEEP SUSPEND mode is set from the register 0x11
//  *	0x11 -> bit 5 -> set value as 1
//  *	the device reaches the lowest power consumption only
//  *	interface selection is kept alive
//  *	No data acquisition is performed
//  *	by using the below API able to set the power mode as DEEPSUSPEND*/
//  /* Set the power mode as DEEPSUSPEND*/
// 	com_rslt += bma2x2_set_power_mode(BMA2x2_MODE_DEEP_SUSPEND);
// /*---------------------------------------------------------------------*
// ************************* END DE-INITIALIZATION **********************
// *---------------------------------------------------------------------*/
// return com_rslt;
// }


/*	\Brief: The function is used as SPI bus read
 *	\Return : Status of the SPI read
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register,
 *          will data is going to be read
 *	\param reg_data : This data read from the sensor,
 *          which is hold in an array
 *	\param cnt : The no of byte of data to be read */



/*	\Brief: The function is used as SPI bus write
 *	\Return : Status of the SPI write
 *	\param dev_addr : The device address of the sensor
 *	\param reg_addr : Address of the first register,
*               will data is going to be written
 *	\param reg_data : It is a value hold in the array,
 *		will be used for write the value into the register
 *	\param cnt : The no of byte of data to be write
 */

int8_t BMA2x2_SPI_bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t length)
{
	HAL_StatusTypeDef bma_write_status;
	HAL_GPIO_WritePin(CSB1_GPIO_Port,CSB1_Pin,GPIO_PIN_RESET);//片选

	bma_write_status = HAL_SPI_Transmit(&hspi1,&reg_addr,1,10);
	bma_write_status = HAL_SPI_Transmit(&hspi1,reg_data,length,10);

	HAL_GPIO_WritePin(CSB1_GPIO_Port,CSB1_Pin,GPIO_PIN_RESET);//取消片选

	if (bma_write_status != HAL_OK)
	{
		/* code */
		return -1;
	}
	
	return 0;
}

s8 BMG160_SPI_bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t length)
{
	HAL_StatusTypeDef bmg_write_status;
	HAL_GPIO_WritePin(CSB2_GPIO_Port,CSB2_Pin,GPIO_PIN_RESET);//片选

	bmg_write_status = HAL_SPI_Transmit(&hspi1,&reg_addr,1,10);
	bmg_write_status = HAL_SPI_Transmit(&hspi1,reg_data,length,10);

	HAL_GPIO_WritePin(CSB2_GPIO_Port,CSB2_Pin,GPIO_PIN_RESET);//取消片选

	if (bmg_write_status != HAL_OK)
	{
		/* code */
		return -1;
	}
	
	return 0;
}

s8 BMM050_SPI_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	HAL_StatusTypeDef bmm_write_state;
	HAL_GPIO_WritePin(CSB3_GPIO_Port,CSB3_Pin,GPIO_PIN_RESET);
	bmm_write_state = HAL_SPI_Transmit(&hspi1,&reg_addr,1,10);
	if (bmm_write_state != HAL_OK)
	{
		return -1;
	}
	bmm_write_state = HAL_SPI_Transmit(&hspi1,reg_data,cnt,10);
	HAL_GPIO_WritePin(CSB3_GPIO_Port,CSB3_Pin,GPIO_PIN_SET);
	if (bmm_write_state != HAL_OK)
	{
		return -1;
	}
	return 0;
}

s8 BMM050_SPI_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	HAL_StatusTypeDef bmm_read_state;
	reg_addr = reg_addr | 0x80;
	HAL_GPIO_WritePin(CSB3_GPIO_Port,CSB3_Pin,GPIO_PIN_RESET);
	bmm_read_state = HAL_SPI_Transmit(&hspi1,&reg_addr,1,10);
	bmm_read_state = HAL_SPI_Receive(&hspi1,reg_data,cnt,10);
	HAL_GPIO_WritePin(CSB3_GPIO_Port,CSB3_Pin,GPIO_PIN_SET);
	if (bmm_read_state != HAL_OK)
	{
		return -1;
	}
	return 0;
}

// s8 BMM050_SPI_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
// {
// 	s32 iError=BMM050_ZERO_U8X;
// 	u8 array[SPI_BUFFER_LEN]={0xFF};
// 	u8 stringpos;
// 	/*	For the SPI mode only 7 bits of register addresses are used.
// 	The MSB of register address is declared the bit what functionality it is
// 	read/write (read as C_BMM050_ONE_U8X/write as BMM050_ZERO_U8X)*/
// 	array[BMM050_ZERO_U8X] = reg_addr|0x80;/*read routine is initiated register address is mask with 0x80*/
// 	/*
// 	* Please take the below function as your reference for
// 	* read the data using SPI communication
// 	* " IERROR = SPI_READ_WRITE_STRING(ARRAY, ARRAY, CNT+C_BMM050_ONE_U8X)"
// 	* add your SPI read function here
// 	* iError is an return value of SPI read function
// 	* Please select your valid return value
// 	* In the driver SUCCESS defined as BMM050_ZERO_U8X
//     * and FAILURE defined as -1
// 	* Note :
// 	* This is a full duplex operation,
// 	* The first read data is discarded, for that extra write operation
// 	* have to be initiated. For that cnt+C_BMM050_ONE_U8X operation done in the SPI read
// 	* and write string function
// 	* For more information please refer data sheet SPI communication:
// 	*/
// 	iError=HAL_SPI_Transmit(&hspi1,array,1,10);
// 	for (stringpos = BMM050_ZERO_U8X; stringpos < cnt; stringpos++)
// 	{
// 		iError=HAL_SPI_Receive(&hspi1,&array[stringpos+C_BMM050_ONE_U8X],1,10);
// 	}
	
// 	iError=HAL_SPI_Receive(&hspi1,array,1,10);
// 	for (stringpos = BMM050_ZERO_U8X; stringpos < cnt; stringpos++) {
// 		*(reg_data + stringpos) = array[stringpos+C_BMM050_ONE_U8X];
// 	}
// 	return (s8)iError;
// }

s8 BMA2x2_SPI_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	HAL_StatusTypeDef bma_read_state;
	reg_addr = reg_addr | 0x80;
	HAL_GPIO_WritePin(CSB1_GPIO_Port,CSB1_Pin,GPIO_PIN_RESET);
	bma_read_state = HAL_SPI_Transmit(&hspi1,&reg_addr,1,10);
	bma_read_state = HAL_SPI_Receive(&hspi1,reg_data,cnt,10);
	HAL_GPIO_WritePin(CSB1_GPIO_Port,CSB1_Pin,GPIO_PIN_SET);
	if (bma_read_state != HAL_OK)
	{
		return -1;
	}
	return 0;
}

s8 BMG160_SPI_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	HAL_StatusTypeDef bmg_read_state;
	reg_addr = reg_addr | 0x80;
	HAL_GPIO_WritePin(CSB2_GPIO_Port,CSB2_Pin,GPIO_PIN_RESET);
	bmg_read_state = HAL_SPI_Transmit(&hspi1,&reg_addr,1,10);
	bmg_read_state = HAL_SPI_Receive(&hspi1,reg_data,cnt,10);
	HAL_GPIO_WritePin(CSB2_GPIO_Port,CSB2_Pin,GPIO_PIN_SET);
	if (bmg_read_state != HAL_OK)
	{
		return -1;
	}
	return 0;
}

void BMA2x2_delay_msek(u32 msek)
{
	/*Here you can write your own delay routine*/
	HAL_Delay(msek);
}

void BMG160_delay_msek(u32 msek)
{
	/*Here you can write your own delay routine*/
	HAL_Delay(msek);
}

void BMM050_delay_msek(u32 msek)
{
	/*Here you can write your own delay routine*/
	HAL_Delay(msek);
}

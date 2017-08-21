#include <kernel.h>
#include <t_syslog.h>
#include <t_stdlib.h>
#include "syssvc/serial.h"
#include "syssvc/syslog.h"
#include "kernel_cfg.h"

#include "stm32f4xx_hal.h"
#include "stm32f401xe.h"

#include "rpr0521.h"
#include "rpr0521_driver.h"

uint32_t timeout = 1000;	//TODO 適当にしてる

unsigned short _als_data0_gain;
unsigned short _als_data1_gain;
unsigned short _als_measure_time;


void write_register(I2C_HandleTypeDef *hi2c, uint16_t reg, uint8_t data) {
    char data_to_send[2];
    HAL_StatusTypeDef status;

    data_to_send[0] = reg;
    data_to_send[1] = data;
    status = HAL_I2C_Master_Transmit(hi2c, (uint16_t)(RPR0521_DEVICE_ADRESS << 1), &data_to_send[0], 2, timeout);
    if(status != HAL_OK){
    	//送信エラー
    	syslog(LOG_NOTICE,"HAL_I2C_Master_Transmit error stats: %d\n\r", status);
    }else{
    	syslog(LOG_NOTICE,"-- HAL_I2C_Master_Transmit OK\n\r");
    }
}

uint8_t read_register(I2C_HandleTypeDef *hi2c, uint16_t reg, uint8_t* buf, uint16_t buf_len) {
    uint8_t received_bytes;
    int read_ok;
    HAL_StatusTypeDef status;

	syslog(LOG_NOTICE,"-- read_register2 start\n\r");
    status = HAL_I2C_Master_Transmit(hi2c, (uint16_t)(RPR0521_DEVICE_ADRESS << 1), buf, buf_len, timeout);
    if(status != HAL_OK){
    	//送信エラー
    	syslog(LOG_NOTICE,"read2 HAL_I2C_Master_Transmit error stats: %d\n\r", status);
    }else{
    	syslog(LOG_NOTICE,"-- read2 HAL_I2C_Master_Transmit OK\n\r");
    }
    status = HAL_I2C_Master_Receive(hi2c, (uint16_t)(RPR0521_DEVICE_ADRESS << 1), buf, buf_len, timeout);

    if( status == HAL_OK ){     //0 == success(ack)
        received_bytes = buf_len;
    	syslog(LOG_NOTICE,"--read2 HAL_I2C_Master_Receive ok\n\r", status);
        }
    else{                   //non0 == fail (nack)
        received_bytes = 0;
    	syslog(LOG_NOTICE,"--read2 HAL_I2C_Master_Receive error status: %d\n\r", status);
        }
    return( received_bytes );
}


void rpr0521_print_one_value(I2C_HandleTypeDef *hi2c){
	bool_t error;
    uint16_t data[3];

    error = rpr0521_read_data(hi2c, &data[0]);
    if (!error) {
    	syslog(LOG_NOTICE,"PS[%4u], Als0[%4u], Als1[%4u]\n\r", data[0], data[1], data[2]);
        }
    else {
    	syslog(LOG_NOTICE,"\n\r");
        }
}


uint8_t rpr0521_readId(I2C_HandleTypeDef *hi2c){
    uint8_t id = RPR0521_MANUFACT;
    uint8_t read_bytes;

    read_bytes = read_register(hi2c, RPR0521_DEVICE_ADRESS, &id, 1);
    if ( read_bytes > 0 ){
        uint8_t partid = RPR0521_SYSTEM_CONTROL;

    	syslog(LOG_NOTICE, "Manufacturer: %u\n\r", id);
        read_bytes = read_register(hi2c, RPR0521_DEVICE_ADRESS, &partid, 1);
        if ( read_bytes > 0 ){
        	syslog(LOG_NOTICE,"Part ID: %u\n\r", (partid & 0b00111111) );
        return(partid);
        }
        else{
        	syslog(LOG_NOTICE,"Part ID read failed.\n\r");
            return 255;
        }
    }
    else{
    	syslog(LOG_NOTICE,"Manufacturer read failed.\n\r");
        return 255;
    }
}


void rpr0521_wait_until_found(I2C_HandleTypeDef *hi2c){
    uint8_t id;

    id = rpr0521_readId(hi2c);
    while (id == 255){
        tslp_tsk(10000);	//org wate100
        id = rpr0521_readId(hi2c);
        }
    return;
}

/*
void rpr0521_soft_reset(){
    write_register(SAD, RPR0521_SYSTEM_CONTROL, RPR0521_SYSTEM_CONTROL_SW_RESET_START);
}

void rpr0521_clear_interrupt(){
    write_register(SAD, RPR0521_SYSTEM_CONTROL, RPR0521_SYSTEM_CONTROL_INT_PIN_HI_Z);
}
*/
void rpr0521_initial_setup(I2C_HandleTypeDef *hi2c){

	write_register(hi2c, RPR0521_ALS_PS_CONTROL,
        (RPR0521_ALS_PS_CONTROL_ALS_DATA0_GAIN_X1 |
         RPR0521_ALS_PS_CONTROL_ALS_DATA1_GAIN_X1 |
         RPR0521_ALS_PS_CONTROL_LED_CURRENT_25MA)
        );
	syslog(LOG_NOTICE,"rpr0521_initial_setup. -1- \n\r");
	write_register(hi2c, RPR0521_PS_CONTROL,
        (RPR0521_PS_CONTROL_PS_GAIN_X1 |
         RPR0521_PS_CONTROL_PERSISTENCE_DRDY )
        );
	syslog(LOG_NOTICE,"rpr0521_initial_setup. -2- \n\r");
	write_register(hi2c, RPR0521_MODE_CONTROL,
        (RPR0521_MODE_CONTROL_ALS_EN_TRUE | RPR0521_MODE_CONTROL_PS_EN_TRUE |
         RPR0521_MODE_CONTROL_PS_PULSE_200US | RPR0521_MODE_CONTROL_PS_OPERATING_MODE_NORMAL |
         RPR0521_MODE_CONTROL_MEASUREMENT_TIME_100MS_100MS)
        );
	syslog(LOG_NOTICE,"rpr0521_initial_setup. -OK- \n\r");

}


/* input param: data16, pointer to 3*16bit memory
   return: error, true/false */

bool_t rpr0521_read_data(I2C_HandleTypeDef *hi2c, uint16_t* data16){
    #define RPR0521_DATA_LEN 6

    uint8_t data[RPR0521_DATA_LEN];
    uint8_t read_bytes;
    data[0] = RPR0521_PS_DATA_LSBS;

    read_bytes = read_register(hi2c, RPR0521_DEVICE_ADRESS, &data[0], RPR0521_DATA_LEN);
    if (read_bytes == RPR0521_DATA_LEN){
        data16[0] = (data[0]) | (data[1] << 8); //ps_data
        data16[1] = (data[2]) | (data[3] << 8); //als_data0
        data16[2] = (data[4]) | (data[5] << 8); //als_data1
        return false;
        }
    else{
    	syslog(LOG_NOTICE,"Read error. Read %d bytes\n\r", read_bytes);
        return true;
        }
    }

//--
unsigned char get_rawpsalsval(I2C_HandleTypeDef *hi2c, unsigned char *data)
{
	bool_t result;
//    uint16_t data[3];

    syslog(LOG_ERROR, "get_rawpsalsval called");
//	int result = read(RPR0521RS_PS_DATA_LSB, data, 6);
//	rpr0521_read_data
//	rpr0521_print_one_value(&hi2c1);
    result = rpr0521_read_data(hi2c, data);
    if (!result) {
    	syslog(LOG_NOTICE,"PS[%4u], Als0[%4u], Als1[%4u]\n\r", data[0], data[1], data[2]);
        }
    else {
    	syslog(LOG_NOTICE,"\n\r");
        }

	return (result);
}

float convert_lx(unsigned short *data)
{
  float lx;
  float d0, d1, d1_d0;

  if (_als_data0_gain == 0) {
    return (RPR0521RS_ERROR);
  }

  if (_als_data1_gain == 0) {
    return (RPR0521RS_ERROR);
  }

  if (_als_measure_time == 0) {
    return (RPR0521RS_ERROR);
  } else if (_als_measure_time == 50) {
    if ((data[0] & 0x8000) == 0x8000) {
      data[0] = 0x7FFF;
    }
    if ((data[1] & 0x8000) == 0x8000) {
      data[1] = 0x7FFF;
    }
  }

  d0 = (float)data[0] * (100 / _als_measure_time) / _als_data0_gain;
  d1 = (float)data[1] * (100 / _als_measure_time) / _als_data1_gain;

  if (d0 == 0) {
    lx = 0;
    return (lx);
  }

  d1_d0 = d1 / d0;

  if (d1_d0 < 0.595) {
    lx = (1.682 * d0 - 1.877 * d1);
  } else if (d1_d0 < 1.015) {
    lx = (0.644 * d0 - 0.132 * d1);
  } else if (d1_d0 < 1.352) {
    lx = (0.756 * d0 - 0.243 * d1);
  } else if (d1_d0 < 3.053) {
    lx = (0.766 * d0 - 0.25 * d1);
  } else {
    lx = 0;
  }

  return (lx);
}


unsigned char get_psalsval(I2C_HandleTypeDef *hi2c, unsigned short *ps, float *als)
{
	unsigned char rc;
	unsigned char val[6];
	unsigned short rawps;
	unsigned short rawals[2];

	syslog(LOG_ERROR, "get_psalsval called");
	rc = get_rawpsalsval(hi2c, val);
	if (rc != 0) {
		return (rc);
	}

	rawps     = ((unsigned short)val[1] << 8) | val[0];
	rawals[0] = ((unsigned short)val[3] << 8) | val[2];
	rawals[1] = ((unsigned short)val[5] << 8) | val[4];

	*ps  = rawps;
	*als = convert_lx(rawals);

	return (rc);
}



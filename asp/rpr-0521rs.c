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
#include "rpr-0521rs.h"

#define SAD 0x38


#define I2C_WRITE 0
#define I2C_READ  1

uint32_t timeout = 1000;

//void write_register(uint8_t sad, uint8_t reg, uint8_t data) {
void write_register(I2C_HandleTypeDef *hi2c, uint16_t reg, uint8_t data) {
    char data_to_send[2];
    HAL_StatusTypeDef status;

    data_to_send[0] = reg;
    data_to_send[1] = data;
//    i2c.write( (int)((sad << 1) | I2C_WRITE ), &data_to_send[0], 2);
    status = HAL_I2C_Master_Transmit(hi2c, RPR0521_DEVICE_ADRESS, &data_to_send[0], 2, timeout);
    if(status != HAL_OK){
    	//送信エラー
    	syslog(LOG_NOTICE,"HAL_I2C_Master_Transmit error stats: %d\n\r", status);
    }else{
    	syslog(LOG_NOTICE,"-- HAL_I2C_Master_Transmit OK\n\r");
    }
}

//uint8_t read_register(uint8_t sad, uint8_t reg, uint8_t* buf, uint8_t buf_len) {
uint8_t read_register(I2C_HandleTypeDef *hi2c, uint16_t reg, uint8_t* buf, uint16_t buf_len) {
    uint8_t received_bytes;
    int read_ok;
    HAL_StatusTypeDef status;

//    i2c.write( (int)((sad << 1) | I2C_WRITE), (char*)&reg, (int)1 );
    //I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout
    status = HAL_I2C_Master_Transmit(hi2c, RPR0521_DEVICE_ADRESS, buf, buf_len, timeout);
//    status = HAL_I2C_Master_Transmit(hi2c, reg, buf, buf_len, timeout);
    if(status != HAL_OK){
    	//送信エラー
    	syslog(LOG_NOTICE,"HAL_I2C_Master_Transmit error stats: %d\n\r", status);
    }else{
    	syslog(LOG_NOTICE,"-- HAL_I2C_Master_Transmit OK\n\r");
    }
    //    read_ok = i2c.read( (int)((sad << 1) | I2C_READ), (char*)buf, (int)buf_len);
    //I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout
    status = HAL_I2C_Master_Receive(hi2c, RPR0521_DEVICE_ADRESS, buf, buf_len, timeout);

    if( status == HAL_OK ){     //0 == success(ack)
        received_bytes = buf_len;
        }
    else{                   //non0 == fail (nack)
        received_bytes = 0;
        }
    return( received_bytes );
}

uint8_t read_register2(I2C_HandleTypeDef *hi2c, uint16_t reg, uint8_t* buf, uint16_t buf_len) {
    uint8_t received_bytes;
    int read_ok;
    HAL_StatusTypeDef status;

//    i2c.write( (int)((sad << 1) | I2C_WRITE), (char*)&reg, (int)1 );
    //I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout
    status = HAL_I2C_Master_Transmit(hi2c, RPR0521_DEVICE_ADRESS, buf, buf_len, timeout);
//    status = HAL_I2C_Master_Transmit(hi2c, reg, buf, buf_len, timeout);
    if(status != HAL_OK){
    	//送信エラー
    	syslog(LOG_NOTICE,"HAL_I2C_Master_Transmit error stats: %d\n\r", status);
    }else{
    	syslog(LOG_NOTICE,"-- HAL_I2C_Master_Transmit OK\n\r");
    }
    //    read_ok = i2c.read( (int)((sad << 1) | I2C_READ), (char*)buf, (int)buf_len);
    //I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout
    status = HAL_I2C_Master_Receive(hi2c, RPR0521_DEVICE_ADRESS, buf, buf_len, timeout);

    if( status == HAL_OK ){     //0 == success(ack)
        received_bytes = buf_len;
        }
    else{                   //non0 == fail (nack)
        received_bytes = 0;
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
    uint8_t id;
    uint8_t read_bytes;

//    read_bytes = read_register(SAD, RPR0521_MANUFACT, &id, 1);
    read_bytes = read_register2(hi2c, RPR0521_MANUFACT, &id, 1);
    if ( read_bytes > 0 ){
        uint8_t partid;

    	syslog(LOG_NOTICE, "Manufacturer: %u\n\r", id);
//        read_bytes = read_register(SAD, RPR0521_SYSTEM_CONTROL, &partid, 1);
        read_bytes = read_register(hi2c, RPR0521_SYSTEM_CONTROL, &partid, 1);
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

    return 255;	//dummy
}


void rpr0521_wait_until_found(I2C_HandleTypeDef *hi2c){
    uint8_t id;

    id = rpr0521_readId(hi2c);
//    while (id == 255){
//        tslp_tsk(10000);	//org wate100
//        id = rpr0521_readId(hi2c);
//        }
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

//    write_register(SAD, RPR0521_ALS_PS_CONTROL,
	write_register(hi2c, RPR0521_ALS_PS_CONTROL,
        (RPR0521_ALS_PS_CONTROL_ALS_DATA0_GAIN_X1 |
         RPR0521_ALS_PS_CONTROL_ALS_DATA1_GAIN_X1 |
         RPR0521_ALS_PS_CONTROL_LED_CURRENT_25MA)
        );
	syslog(LOG_NOTICE,"rpr0521_initial_setup. -1- \n\r");
//    write_register(SAD, RPR0521_PS_CONTROL,
	write_register(hi2c, RPR0521_PS_CONTROL,
        (RPR0521_PS_CONTROL_PS_GAIN_X1 |
         RPR0521_PS_CONTROL_PERSISTENCE_DRDY )
        );
	syslog(LOG_NOTICE,"rpr0521_initial_setup. -2- \n\r");
//    write_register(SAD, RPR0521_MODE_CONTROL,
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

//    read_bytes = read_register(SAD, RPR0521_PS_DATA_LSBS, &data[0], RPR0521_DATA_LEN);
    read_bytes = read_register(hi2c, RPR0521_PS_DATA_LSBS, &data[0], RPR0521_DATA_LEN);
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




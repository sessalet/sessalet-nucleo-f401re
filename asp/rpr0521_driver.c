/*   Copyright 2016 Rohm Semiconductor

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/
// #include "../../rohm-sensor-hal/rohm-sensor-hal/rohm_hal.h"         //types, DEBUG_print*
// #include "../../rohm-sensor-hal/rohm-sensor-hal/I2CCommon.h"        //read_register, write_register, change_bits

#include <kernel.h>
#include <t_syslog.h>
#include <t_stdlib.h>

#include "rpr0521.h"      //RPR0521_* register definitions
#include "rpr0521_driver.h"
#define SAD 0x38

#define I2C_WRITE 0
#define I2C_READ  1



void aquire() {
    if (_owner != this) {
        i2c_frequency(&_i2c, _hz);
        _owner = this;
    }
}

int write(int address, const char* data, int length, bool repeated) {
    aquire();

    int stop = (repeated) ? 0 : 1;
    int written = i2c_write(&_i2c, address, data, length, stop);

    return length != written;
}



//-- from I2CCommonMbedHardwareLib.cpp
/* i2c common functions */
uint8_t read_register(uint8_t sad, uint8_t reg, uint8_t* buf, uint8_t buf_len) {
    uint8_t received_bytes;
    int read_ok;

    i2c.write( (int)((sad << 1) | I2C_WRITE), (char*)&reg, (int)1 );
    read_ok = i2c.read( (int)((sad << 1) | I2C_READ), (char*)buf, (int)buf_len);

    if( read_ok == 0 ){     //0 == success(ack)
        received_bytes = buf_len;
        }
    else{                   //non0 == fail (nack)
        received_bytes = 0;
        }
    return( received_bytes );
}

void write_register(uint8_t sad, uint8_t reg, uint8_t data) {
    char data_to_send[2];

    data_to_send[0] = reg;
    data_to_send[1] = data;
    i2c.write( (int)((sad << 1) | I2C_WRITE ), &data_to_send[0], 2);
}

//----




/* rpr0521 driver*/

uint8_t rpr0521_readId(){
    uint8_t id;
    uint8_t read_bytes;

    read_bytes = read_register(SAD, RPR0521_MANUFACT, &id, 1);
    if ( read_bytes > 0 ){
        uint8_t partid;

        syslog(LOG_NOTICE, "Manufacturer: %u\n\r", id);
        read_bytes = read_register(SAD, RPR0521_SYSTEM_CONTROL, &partid, 1);
        if ( read_bytes > 0 ){
        	syslog(LOG_NOTICE, "Part ID: %u\n\r", (partid & 0b00111111) );
        return(partid);
        }
        else{
            DEBUG_print("Part ID read failed.\n\r");
            return 255;
        }
    }
    else{
        DEBUG_print("Manufacturer read failed.\n\r");
        return 255;
    }
}

void rpr0521_wait_until_found(){
    uint8_t id;

    id = rpr0521_readId();
    while (id == 255){
        wait(100);
        id = rpr0521_readId();
        }
    return;
}

void rpr0521_soft_reset(){
    write_register(SAD, RPR0521_SYSTEM_CONTROL, RPR0521_SYSTEM_CONTROL_SW_RESET_START);
}

void rpr0521_clear_interrupt(){
    write_register(SAD, RPR0521_SYSTEM_CONTROL, RPR0521_SYSTEM_CONTROL_INT_PIN_HI_Z);
}

void rpr0521_initial_setup(){
    write_register(SAD, RPR0521_ALS_PS_CONTROL,
        (RPR0521_ALS_PS_CONTROL_ALS_DATA0_GAIN_X1 |
         RPR0521_ALS_PS_CONTROL_ALS_DATA1_GAIN_X1 |
         RPR0521_ALS_PS_CONTROL_LED_CURRENT_25MA)
        );
    write_register(SAD, RPR0521_PS_CONTROL,
        (RPR0521_PS_CONTROL_PS_GAIN_X1 |
         RPR0521_PS_CONTROL_PERSISTENCE_DRDY )
        );
    write_register(SAD, RPR0521_MODE_CONTROL,
        (RPR0521_MODE_CONTROL_ALS_EN_TRUE | RPR0521_MODE_CONTROL_PS_EN_TRUE |
         RPR0521_MODE_CONTROL_PS_PULSE_200US | RPR0521_MODE_CONTROL_PS_OPERATING_MODE_NORMAL |
         RPR0521_MODE_CONTROL_MEASUREMENT_TIME_100MS_100MS)
        );
}

/* input param: data16, pointer to 3*16bit memory
   return: error, true/false */
bool_t rpr0521_read_data(uint16_t* data16){
    #define RPR0521_DATA_LEN 6

    uint8_t data[RPR0521_DATA_LEN];
    uint8_t read_bytes;

    read_bytes = read_register(SAD, RPR0521_PS_DATA_LSBS, &data[0], RPR0521_DATA_LEN);
    if (read_bytes == RPR0521_DATA_LEN){
        data16[0] = (data[0]) | (data[1] << 8); //ps_data
        data16[1] = (data[2]) | (data[3] << 8); //als_data0
        data16[2] = (data[4]) | (data[5] << 8); //als_data1
        return false;
	}
else{
	syslog(LOG_NOTICE, "Read error. Read %d bytes\n\r", read_bytes);
	return true;
	}

}

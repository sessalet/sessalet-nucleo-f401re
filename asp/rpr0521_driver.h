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
#ifndef rpr0521_driver_h
#define rpr0521_driver_h

#define RPR0521_DEVICE_ADRESS 0x38

#define RPR0521RS_ERROR                            (-1)


/* rpr0521 driver*/

uint8_t rpr0521_readId(I2C_HandleTypeDef *hi2c);
void rpr0521_wait_until_found(I2C_HandleTypeDef *hi2c);
bool_t rpr0521_read_data(I2C_HandleTypeDef *hi2c, uint16_t* data16);
void rpr0521_initial_setup(I2C_HandleTypeDef *hi2c);
void rpr0521_print_one_value(I2C_HandleTypeDef *hi2c);
void rpr0521_soft_reset();
void rpr0521_clear_interrupt();
unsigned char get_psalsval(I2C_HandleTypeDef *hi2c, unsigned short *ps, float *als);
unsigned char get_rawpsalsval(I2C_HandleTypeDef *hi2c, unsigned char *data);


#endif

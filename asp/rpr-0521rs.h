/*
 * rpr-0521rs.h
 *
 *  Created on: 2017/08/15
 *      Author: yuri
 */

#ifndef RPR_0521RS_H_
#define RPR_0521RS_H_

#define RPR0521_DEVICE_ADRESS 0x38

    int init(void) ;
    unsigned char get_rawpsalsval(unsigned char *data);
    unsigned char get_psalsval(unsigned short *ps, float *als);
    unsigned char check_near_far(unsigned short data);
    float convert_lx(unsigned short *data);
    int write(unsigned char command, unsigned char *data, unsigned char size);
    int read(unsigned char command, unsigned char *data, int size);

//  private:
    unsigned short _als_data0_gain;
    unsigned short _als_data1_gain;
    unsigned short _als_measure_time;



#endif /* RPR_0521RS_H_ */

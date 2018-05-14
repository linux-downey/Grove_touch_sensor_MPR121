/*
 * Seeed_MPR121_driver.h
 * Driver for DIGITAL I2C HUMIDITY AND TEMPERATURE SENSOR
 *  
 * Copyright (c) 2018 Seeed Technology Co., Ltd.
 * Website    : www.seeed.cc
 * Author     : downey
 * Create Time: May 2018
 * Change Log :
 *
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#ifndef SEEED_MPR121_DRIVER_H
#define SEEED_MPR121_DRIVER_H


#include <Wire.h>
#include <Arduino.h>

#define DEFUALT_MPR121_ADDR   0X5B

#define TOUCH_THRESHOLD_MAX   0X50       

/****************************************************Sensor register address!!***********************************************/
/****************************************************************************************************************************/


#define CHANNEL_NUM   12

/*These two registers indicate the detected touch/release status of all of the 12 sensing input channels*/
#define TOUCH_STATUS_REG_ADDR_L             0X00
#define TOUCH_STATUS_REG_ADDR_H             0X01

/*The MPR121 provides filtered electrode output data for all 12 channels*/
/*Total 26 registers,for 12 channel dlectrode out put data.Each channel corresponding two registers:high byte and low byte*/
/*0x04~0x1d*/
#define FILTERED_DATA_REG_START_ADDR_L      0X04
#define FILTERED_DATA_REG_START_ADDR_H      0X05

/*Along with the 10-bit electrode filtered data output, each channel also has a 10-bit baseline value*/
/*0X1E~0X2A*/
#define BASELINE_VALUE_REG_START_ADDR             0X1E


/*All12 of the electrode baseline values are controlled by the same set of filtering control registers, 0x2B ~ 0x35*/
/*The 13th channel ELEPROX is controlled by registers 0x36 ~ 0x40*/
#define BASELINE_FILTERING_CONTROL_REG_START_ADDR    0X2B  


/*Each of the 12 channels can be set with its own set of touch and release thresholds. Touch and release are detected by
comparing the electrode filtered data to the baseline value. typically in the range 0x04~0x10*/
/*Touch condition: Baseline - Electrode filtered data > Touch threshold
  Release condition: Baseline - Electrode filtered data < Release threshold*/
#define THRESHOLD_REG_START_ADDR            0X41

/*All 12 channels use the same set of touch and release debounce numbers.*/
#define DEBOUNCE_REG_ADDR                   0X5B

/*These two registers set the global AFE settings. This includes global electrode charge/discharge current CDC, global charge/
discharge time CDT, as well as a common filtering setting (FFI, SFI, ESI) for all 12 channels, including the 13th Eleprox channel*/
#define FILTER_AND_GLOBAL_CDC_CFG_ADDR      0X5C
#define FILTER_AND_GLOBAL_CDT_CFG_ADDR      0X5D

/*0X5F-0X6B*/
#define ELEC_CHARGE_CURRENT_REG_START_ADDR  0X5F

/*0X6C-0X72*/
#define ELEC_CHARGE_TIME_REG_START_ADDR  0X6C

/*The Electrode Configuration Register (ECR) determines if the MPR121 is in Run Mode or Stop Mode*/
/*Default is 0 to stop mode*/
#define ELEC_CFG_REG_ADDR                    0X5E

using u8 = unsigned char;
using u16 = unsigned short;
using u32 =long unsigned int;
using s32 = int;

typedef enum
{
    STOP_MODE,
    START_PROXIMITY_ENABLE_MODE,
    START_PROXIMITY_DISABLE_MODE,
}sensor_mode_t;




class Mpr121
{
    public:
        Mpr121(u8 addr=DEFUALT_MPR121_ADDR);
        ~Mpr121(){}
        s32 begin();
        u16 check_status_register();
        s32 select_mode(sensor_mode_t mode);
        void set_debounce(u8 debounce);
        void set_threshold(u16 threshold);
        void set_globle_param(u16 value);
        void get_filtered_reg_data(u16 *elecs_stat,u16* elecs_filtered_data);
        void get_baseline_data(u16 elecs_stat,u8* base_line_data);
    private:        
        u8 _IIC_ADDR;
        void IIC_read_bytes(u8 reg,u8* bytes,u32 bytes_len);
        void IIC_read_byte(u8 reg,u8* byte);
        s32 IIC_write_byte(u8 reg,u8 byte);
        void IIC_write_bytes(u8 reg,u8 bytes[],u32 bytes_len);
        s32 sensor_stop();
        s32 sensor_start_proximity_enable();
        s32 sensor_start_proximity_disable();
};



#endif

/*
 * Seeed_MPR121_driver.cpp
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

#include "Seeed_MPR121_driver.h"

Mpr121::Mpr121(u8 addr)
{
    _IIC_ADDR=addr;
}

s32  Mpr121::begin()
{
    s32 ret=0;
    Wire.begin();
    ret=select_mode(STOP_MODE);
    if(0!=ret)
    {
        return -1;
    }
    delay(100);
    /*FFI-6,CDC-16UA,CDT-0.5US,SFI-4,ESI-4MS*/
    /**/
    set_globle_param(0x2310);
    /*Touch debounce =2*SFI*ESI,Realese debounce=2*SFI*ESI */
    set_debounce(0X22);
    /*Set touch and realese threshold 0x08.*/
    //set_threshold(0x0808);
    select_mode(START_PROXIMITY_DISABLE_MODE);
    //select_mode(START_PROXIMITY_ENABLE_MODE);
    return ret;
}


/**@brief Set sensor mode:STOP,detection,proximity.
 * @param mode
 * 
 * */
s32 Mpr121::select_mode(sensor_mode_t mode)
{
    switch(mode)
    {
        case STOP_MODE:
        return sensor_stop();
        break;
        case START_PROXIMITY_ENABLE_MODE:
        return sensor_start_proximity_enable();
        break;
        case START_PROXIMITY_DISABLE_MODE:
        return sensor_start_proximity_disable();
        break;
        default:break;
    }
}



/*debounce_touch=(bit0~bit2)*EFI*SSI   debounce_release=(bit4~bit6)*ESI*SFI */
/*Default ESI=16ms,SFI=4,This two parameters is configurable,see set_globle_param()*/
/*ESI--Electrode Sample Interval,SFI--Second Filter Iterations*/

/**@brief Set debounce of touch 
 * @param debounce 
 * 
 * */
void Mpr121::set_debounce(u8 debounce)
{
   IIC_write_byte(DEBOUNCE_REG_ADDR,debounce); 
}


/*  0x5d:
    bit0~bit2:ESI
    bit3~bit4:SFI
    bit5~bit7:CDT
    0x5c:
    bit0~bit5:CDC
    bit6~bit7:FFI
    0x5c-low byte,0x5d-high byte
*/
/**@brief set globle parameters.FFI(First Filter Iterations ),CDC(Charge Discharge Current)
 * @brief CDT(Charge Discharge Time),SFI(Second Filter Iterations),ESI(Electrode Sample Interval)
 * @param 16bit value
 * 
 * */
void Mpr121::set_globle_param(u16 value)
{
    u8 val_l=(u8)value;
    u8 val_h=(u8)(value>>8);
    IIC_write_byte(FILTER_AND_GLOBAL_CDC_CFG_ADDR,val_l);
    IIC_write_byte(FILTER_AND_GLOBAL_CDT_CFG_ADDR,val_h);
}


/**@brief Set threshold  
 * @brief Touch condition: Baseline - Electrode filtered data > Touch threshold
 * @brief Release condition: Baseline - Electrode filtered data < Release threshold
 * @brief It's not very necessary to set this param in simple use case.
 * @param Threshold for sensor
 * */
void Mpr121::set_threshold(u16 threshold)
{
    u8 thres_touch=(u8)threshold;
    u8 thres_realese=(u8)(threshold>>8);
    for(int i=0;i<CHANNEL_NUM;i++)
    {
        IIC_write_byte(THRESHOLD_REG_START_ADDR+2*i,thres_touch);
        IIC_write_byte(THRESHOLD_REG_START_ADDR+2*i+1,thres_realese);
    }
}



/**@brief Total 12 channels for touch electrode,bit0-bit11 of 8+8 bits data indicate if there is a channel triggered.
 * @brief Bit0-bit11 correponding to channel 0-11
 * @return 16 bits indicate data.
 * */
u16 Mpr121::check_status_register()
{
    u16 val=0;
    u8 val_l,val_h;
    IIC_read_byte(TOUCH_STATUS_REG_ADDR_L,&val_l);
    IIC_read_byte(TOUCH_STATUS_REG_ADDR_H,&val_h);
    val=val_h<<8|val_l;
    return val;
}

/**@brief When key is pressed,The sensor not just ouput 0/1
 * @brief The MPR121 provides filtered electrode output data for all 13 channels,
 * @param elecs_stat.Judge which channel is triggered according to this param 
 * @param elecs_filtered_data.Get corresponding channel data.
 * */
void Mpr121::get_filtered_reg_data(u16 *elecs_stat,u16* elecs_filtered_data)
{
    u16 value=0;
    u8 data_l,data_h;
    for(int i=0;i<CHANNEL_NUM;i++)
    {
        if((*elecs_stat)&(1<<i))
        {
            IIC_read_byte(FILTERED_DATA_REG_START_ADDR_L+2*i,&data_l);
            IIC_read_byte(FILTERED_DATA_REG_START_ADDR_L+2*i+1,&data_h);
            elecs_filtered_data[i]=(u16)data_h<<8|data_l;
            if(TOUCH_THRESHOLD_MAX<elecs_filtered_data[i])
            {
                (*elecs_stat)&=~(1<<i);
            }
            //Serial.print("press value= 0X");
            //Serial.println(elecs_filtered_data[i],HEX);
        }
    }
}



void Mpr121::get_baseline_data(u16 elecs_stat,u8* base_line_data)
{
    u16 value=0;
    u8 data=0;
    for(int i=0;i<CHANNEL_NUM;i++)
    {
        if(elecs_stat&(1<<i))
        {
            IIC_read_byte(BASELINE_FILTERING_CONTROL_REG_START_ADDR+i,&data);
            Serial.print("base line value= 0X");
            Serial.println(base_line_data[i],HEX);
        }
    }
}


/**@brief Set stop mode by set ELEC_CFG_REG_ADDR to 0x0 
 * 
 * */
s32 Mpr121::sensor_stop()
{
    return IIC_write_byte(ELEC_CFG_REG_ADDR,0);
}

/**@brief Set start mode with proximity enable by set ELEC_CFG_REG_ADDR to 0x0 
 * 
 * */
s32 Mpr121::sensor_start_proximity_enable()
{
    IIC_write_byte(ELEC_CFG_REG_ADDR,0x3c);
}

/**@brief Set start mode with proximity disable by set ELEC_CFG_REG_ADDR to 0x0 
 * 
 * */
s32 Mpr121::sensor_start_proximity_disable()
{
    IIC_write_byte(ELEC_CFG_REG_ADDR,0x3c);
}


/*****************************************IIC operation interface!!*****************************************/
/************************************************************************************************************/
s32 Mpr121::IIC_write_byte(u8 reg,u8 byte)
{
    Wire.beginTransmission(_IIC_ADDR);
    Wire.write(reg);
    Wire.write(byte);
    return Wire.endTransmission();
}

void Mpr121::IIC_write_bytes(u8 reg,u8 bytes[],u32 bytes_len)
{
    Wire.beginTransmission(_IIC_ADDR);
    Wire.write(reg);
    for(int i=0;i<bytes_len;i++)
    {
        Wire.write(bytes[i]);
    }
    Wire.endTransmission();
}

void Mpr121::IIC_read_byte(u8 reg,u8* byte)
{
    Wire.beginTransmission(_IIC_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);

    Wire.requestFrom(_IIC_ADDR,(u8)1);
    while(1!=Wire.available());
    *byte=Wire.read();
    
}

void Mpr121::IIC_read_bytes(u8 start_reg,u8* bytes,u32 bytes_len)
{
    Wire.beginTransmission(_IIC_ADDR);
    Wire.write(start_reg);
    Wire.endTransmission(false);

    Wire.requestFrom(_IIC_ADDR,(u8)bytes_len);
     while(bytes_len!=Wire.available());
    for(int i=0;i<bytes_len;i++)
    {
        bytes[i]=Wire.read();
    }
}


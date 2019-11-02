#ifndef  __TOF10120_H__
#define  __TOF10120_H__

#include  "common.h"
#include  "MK60_gpio.h"
#include  "MK60_SysTick.h"

#define TOF_SCL    PTA13
#define TOF_SDA    PTA12

#define TOF10120_DEV_ADDR         0x52

typedef enum TOF       //DACÄ£¿é
{
    TOF_IIC,
    TOF_SCCB
} type;


extern uint8 dis;


void TOF_start(void);
void TOF_stop(void);
void TOF_delay(void);
void TOF_send_ch(uint8 c);
uint8 TOF_read_ch(void);
void  TOF_init(void);
void TOF_Measurement(uint8 *data);
uint8 read_reg(uint8 dev_add, uint8 reg, type type);

#endif
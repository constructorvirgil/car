#include "encoder.h"
#include "common.h"
#include "MK60_ftm.h"
#include "lcd.h"

int16 car_speed;
int16 speed_left;
int16 speed_right;
void encoder_init()
{
    LCD_Init();
    ftm_quad_init(FTM1);
    ftm_quad_init(FTM2);
}

void speed_measure(void)
{

    speed_right = -ftm_quad_get(ENCODER_LEFT);  // 获取FTM 正交解码 的脉冲数(负数表示反方向)
    ftm_quad_clean(ENCODER_LEFT);
    speed_left  = ftm_quad_get(ENCODER_RIGHT);  // 获取FTM 正交解码 的脉冲数(负数表示反方向)
    ftm_quad_clean(ENCODER_RIGHT);
    car_speed=(speed_left+speed_right)/2;
}

void speed_display()
{
    LCD_CLS();
    char str[24]={0};
    sprintf(str,"speed:%d",car_speed); 
    LCD_P6x8Str(24,3,(byte *)str);

    //display left
    memset(str,0,24);
    sprintf(str,"left:%d",speed_left);
    LCD_P6x8Str(24,4,(byte *)str);

    //display right
    memset(str,0,24);
    sprintf(str,"right:%d",speed_right);
    LCD_P6x8Str(24,5,(byte *)str);
}
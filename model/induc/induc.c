#include "induc.h"
#include "CirQueue.h"
#include "common.h"
#include "MK60_adc.h"
#include "lcd.h"
#include "stdio.h"

//按顺序是左中右,左上,右上
//static ADCn_Ch_e adc_ch[INDUC_N]={ADC0_SE10,ADC0_SE18,ADC0_SE17,ADC1_SE15,ADC1_SE14};
static ADCn_Ch_e adc_ch[INDUC_N]={ADC0_SE10,ADC0_SE18,ADC0_SE17};

CirQueue * cq[INDUC_N];
static int induc_sum[INDUC_N] = {0};
uint8 induc_val[INDUC_N] = {0};


void induc_init()
{
    LCD_Init();
    //circle queue init
    for(int i=0;i<INDUC_N;i++)
    {
        cq[i] = Q_Init();
    }
    //adc init
    for(int i=0;i<INDUC_N;i++)
    {
        adc_init(adc_ch[i]);
    }
}

void induc_update_no_filter()
{
    for(int i=0;i<INDUC_N;i++)
    {
        induc_val[i] = adc_ave(adc_ch[i], ADC_10bit, 5) >> 2;
    }
}

void induc_update()
{
    static int first = 1;

    //如果第一次滤波,那么先把缓冲区采满
    if(first == 1)
    {
        first = 0;
        for(int i=0;i<CIRQUESIZE;i++)
        {
            for(int j=0;j<INDUC_N;j++)
            {
                Q_Put(cq[j],adc_ave(adc_ch[j], ADC_10bit, 5) >> 2);
            }
        }
    }else{
        for(int i=0;i<INDUC_N;i++)
        {
            Q_Poll(cq[i]);
            Q_Put(cq[i],adc_ave(adc_ch[i], ADC_10bit, 5) >> 2);
        }
    }


    for(int j=0;j<INDUC_N;j++)
    {
        for (int i=0;i<CIRQUESIZE;i++) 
        {
            induc_sum[j] += (CIRQUESIZE - i) * cq[j]->base[i];
        }
    }
    

    for(int i=0;i<INDUC_N;i++)
    {
        induc_val[i] = 2 * induc_sum[i] / (CIRQUESIZE + 1) / CIRQUESIZE;
    }
    
}

void induc_display()
{
    char str[INDUC_N][8];
    LCD_CLS();
    for(int i=0;i<INDUC_N;i++)
    {
        sprintf(str[i],"val:%d",induc_val[i]);
    }
    for(int i=0;i<INDUC_N;i++)
    {
        LCD_P6x8Str(24,2+i,(byte *)str[i]);
    }
}
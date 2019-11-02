#include "Induc.h"
#include "CirQueue.h"
#include "common.h"
#include "stdio.h"
#include "include.h"
#include "lcd.h"
#include "control.h"
#include "TOF10120.h"

static int first = 1;
static ADCn_Ch_e adc_ch[IND_NUM]={ADC1_SE4a,ADC1_SE5a,ADC1_SE6a,ADC0_SE14,ADC0_SE15};
CirQueue * cq[IND_NUM];
static int sum[IND_NUM] = {0};
uint8 indVal[IND_NUM] = {0};

void INducUpdate()
{
    //注意缓冲区的长度问题
    for(int i=0;i<IND_NUM;i++)
    {
        cq[i] = Q_Init();
    }

    //如果第一次滤波,那么先把缓冲区采满
    if(first ==1)
    {
        first = 0;
        for(int i=0;i<CIRQUESIZE;i++)
        {
            for(int j=0;j<IND_NUM;j++)
            {
                Q_Put(cq[i],adc_ave(adc_ch[i], ADC_10bit, 5) >> 2);
            }
        }
    }else{
        for(int i=0;i<IND_NUM;i++)
        {
            Q_Poll(cq[i]);
            Q_Put(cq[i],adc_ave(adc_ch[i], ADC_10bit, 5) >> 2);
        }
    }


    for(int j=0;j<IND_NUM;j++)
    {
        for (int i=0;i<CIRQUESIZE;i++) 
        {
            sum[j] += (CIRQUESIZE - i) * cq[j]->base[i];
        }
    }
    

    for(int i=0;i<IND_NUM;i++)
    {
        indVal[i] = 2 * sum[i] / (CIRQUESIZE + 1) / CIRQUESIZE;
    }
    
}
/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,ɽ��Ƽ�
 *     All rights reserved.
 *     �������ۣ�ɽ����̳ http://www.vcan123.com
 *
 *     ��ע�������⣬�����������ݰ�Ȩ����ɽ��Ƽ����У�δ����������������ҵ��;��
 *     �޸�����ʱ���뱣��ɽ��Ƽ��İ�Ȩ������
 *
 * @file       VCAN_OV7725_Eagle.c
 * @brief      ӥ��ov7725��������
 * @author     ɽ��Ƽ�
 * @version    v5.0
 * @date       2013-09-07
 */

#include "common.h"
#include "MK60_gpio.h"
#include "MK60_port.h"
#include "MK60_dma.h"
#include "VCAN_camera.h"



#define OV7725_EAGLE_Delay_ms(time)  DELAY_MS(time)


uint8   *ov7725_eagle_img_buff;

uint8  THRESHOLD_VALUE;
   
volatile IMG_STATUS_e      ov7725_eagle_img_flag = IMG_FINISH;   //ͼ��״̬

//�ڲ���������
static uint8 ov7725_eagle_reg_init(uint8 threshold);
static void ov7725_eagle_port_init();


/*!
 *  @brief      ӥ��ov7725��ʼ��
 *  @since      v5.0
 */


uint8 ov7725_eagle_init(uint8 *imgaddr , uint8 threshold) // ov7725_eagle_init(uint8 *imgaddr)
{
    ov7725_eagle_img_buff = imgaddr;
    //THRESHOLD_VALUE =  threshold;  
    while(ov7725_eagle_reg_init(threshold) == 0);
    ov7725_eagle_port_init();
    return 0;
}

/*!
 *  @brief      ӥ��ov7725�ܽų�ʼ�����ڲ����ã�
 *  @since      v5.0
 */
void ov7725_eagle_port_init()
{
    //DMAͨ��0��ʼ����PTA27����Դ(Ĭ��������)��Դ��ַΪPTB_B0_IN��Ŀ�ĵ�ַΪ��IMG_BUFF��ÿ�δ���1Byte
    dma_portx2buff_init(CAMERA_DMA_CH, 
								(void *)&PTB_B0_IN, 
								(void *)ov7725_eagle_img_buff, 
								PTA27,
								DMA_BYTE1,
								CAMERA_DMA_NUM,
								DADDR_KEEPON);

    DMA_DIS(CAMERA_DMA_CH);
    disable_irq(PORTA_IRQn);                        //�ر�PTA���ж�
    DMA_IRQ_CLEAN(CAMERA_DMA_CH);                   //���ͨ�������жϱ�־λ
    DMA_IRQ_EN(CAMERA_DMA_CH);

    port_init(PTA27, ALT1 | DMA_FALLING | PULLDOWN );         //PCLK

    port_init(PTA29, ALT1 | IRQ_RISING  | PULLDOWN | PF);     //���жϣ��������Ͻ��ش����жϣ����˲�
	
	port_init(PTA28, ALT1 | IRQ_RISING  | PULLDOWN | PF);     //���жϣ��������Ͻ��ش����жϣ����˲�,    ������ģ��

}

/*!
 *  @brief      ӥ��ov7725���жϷ�����
 *  @since      v5.0
 */
void ov7725_eagle_vsync(void)
{

    //���ж���Ҫ�ж��ǳ��������ǳ���ʼ
    if(ov7725_eagle_img_flag == IMG_START)                   //��Ҫ��ʼ�ɼ�ͼ��
    {
        ov7725_eagle_img_flag = IMG_GATHER;                  //���ͼ��ɼ���
        disable_irq(PORTA_IRQn);

#if 1

        PORTA_ISFR = 1 <<  PT27;            //���PCLK��־λ

        DMA_EN(CAMERA_DMA_CH);                  //ʹ��ͨ��CHn Ӳ������
        PORTA_ISFR = 1 <<  PT27;            //���PCLK��־λ
        DMA_DADDR(CAMERA_DMA_CH) = (uint32)ov7725_eagle_img_buff;    //�ָ���ַ

#else
        PORTA_ISFR = 1 <<  PT27;            //���PCLK��־λ
        dma_repeat(CAMERA_DMA_CH, (void *)&PTB_B0_IN, (void *)ov7725_eagle_img_buff,CAMERA_DMA_NUM);
#endif
    }
    else                                        //ͼ��ɼ�����
    {
        disable_irq(PORTA_IRQn);                        //�ر�PTA���ж�
        ov7725_eagle_img_flag = IMG_FAIL;                    //���ͼ��ɼ�ʧ��
    }
}

/*!
 *  @brief      ӥ��ov7725 DMA�жϷ�����
 *  @since      v5.0
 */
void ov7725_eagle_dma()
{
    ov7725_eagle_img_flag = IMG_FINISH ;
    DMA_IRQ_CLEAN(CAMERA_DMA_CH);           //���ͨ�������жϱ�־λ
}

/*!
 *  @brief      ӥ��ov7725�ɼ�ͼ�񣨲ɼ��������ݴ洢�� ��ʼ��ʱ���õĵ�ַ�ϣ�
 *  @since      v5.0
 */
void ov7725_eagle_get_img()
{
    ov7725_eagle_img_flag = IMG_START;                   //��ʼ�ɼ�ͼ��
    PORTA_ISFR = ~0;                        //д1���жϱ�־λ(����ģ���Ȼ�ص���һ���жϾ����ϴ����ж�)
    enable_irq(PORTA_IRQn);                         //����PTA���ж�
    while(ov7725_eagle_img_flag != IMG_FINISH)           //�ȴ�ͼ��ɼ����
    {
        if(ov7725_eagle_img_flag == IMG_FAIL)            //����ͼ��ɼ����������¿�ʼ�ɼ�
        {
            ov7725_eagle_img_flag = IMG_START;           //��ʼ�ɼ�ͼ��
            PORTA_ISFR = ~0;                //д1���жϱ�־λ(����ģ���Ȼ�ص���һ���жϾ����ϴ����ж�)
            enable_irq(PORTA_IRQn);                 //����PTA���ж�
        }
    }
}

/*OV7725��ʼ�����ñ�*/
/*reg_s ov7725_eagle_reg[] =
{
    //�Ĵ������Ĵ���ֵ��
   // {OV7725_COM4         , 0xc1},  //50 0xc1 0x02   75 0x41 0x00    112 0x81 0x00  150  0xc1 0x00

 
    {OV7725_COM4         , 0xc1},
    {OV7725_CLKRC        , 0x00},
    {OV7725_COM2         , 0x03},
    {OV7725_COM3         , 0xD0},
    {OV7725_COM7         , 0x40},
    {OV7725_HSTART       , 0x3F},
    {OV7725_HSIZE        , 0x50},
    {OV7725_VSTRT        , 0x03},
    {OV7725_VSIZE        , 0x78},
    {OV7725_HREF         , 0x00},
    {OV7725_SCAL0        , 0x0A},
    {OV7725_AWB_Ctrl0    , 0xE0},
    {OV7725_DSPAuto      , 0xff},
    {OV7725_DSP_Ctrl2    , 0x0C},
    {OV7725_DSP_Ctrl3    , 0x00},
    {OV7725_DSP_Ctrl4    , 0x00},

#if (CAMERA_W == 80)
    {OV7725_HOutSize     , 0x14},
#elif (CAMERA_W == 160)
    {OV7725_HOutSize     , 0x28},
#elif (CAMERA_W == 240)
    {OV7725_HOutSize     , 0x3c},
#elif (CAMERA_W == 320)
    {OV7725_HOutSize     , 0x50},
#else

#endif

#if (CAMERA_H == 60 )
    {OV7725_VOutSize     , 0x1E},
#elif (CAMERA_H == 120 )
    {OV7725_VOutSize     , 0x3c},
#elif (CAMERA_H == 180 )
    {OV7725_VOutSize     , 0x5a},
#elif (CAMERA_H == 240 )
    {OV7725_VOutSize     , 0x78},
#else

#endif

    {OV7725_EXHCH        , 0x00},
    {OV7725_GAM1         , 0x0c},
    {OV7725_GAM2         , 0x16},
    {OV7725_GAM3         , 0x2a},
    {OV7725_GAM4         , 0x4e},
    {OV7725_GAM5         , 0x61},
    {OV7725_GAM6         , 0x6f},
    {OV7725_GAM7         , 0x7b},
    {OV7725_GAM8         , 0x86},
    {OV7725_GAM9         , 0x8e},
    {OV7725_GAM10        , 0x97},
    {OV7725_GAM11        , 0xa4},
    {OV7725_GAM12        , 0xaf},
    {OV7725_GAM13        , 0xc5},
    {OV7725_GAM14        , 0xd7},
    {OV7725_GAM15        , 0xe8},
    {OV7725_SLOP         , 0x20},
    {OV7725_LC_RADI      , 0x00},
    {OV7725_LC_COEF      , 0x13},
    {OV7725_LC_XC        , 0x08},
    {OV7725_LC_COEFB     , 0x14},
    {OV7725_LC_COEFR     , 0x17},
    {OV7725_LC_CTR       , 0x05},
    {OV7725_BDBase       , 0x99},
    {OV7725_BDMStep      , 0x03},
    {OV7725_SDE          , 0x04},
    {OV7725_BRIGHT       , 0x00},
    {OV7725_CNST         , 0x40},
    {OV7725_SIGN         , 0x06},
    {OV7725_UVADJ0       , 0x11},
    {OV7725_UVADJ1       , 0x02},


};
*/

reg_s ov7725_eagle_reg[] =
{
    //�Ĵ������Ĵ���ֵ��
   // {OV7725_COM4         , 0xc1},  //50 0xc1 0x02   75 0x41 0x00    112 0x81 0x00  150  0xc1 0x00

 
    {OV7725_COM4         , 0xc1},
    {OV7725_CLKRC        , 0x00},
    {OV7725_COM2         , 0x03},
    {OV7725_COM3         , 0xD0},
    {OV7725_COM7         , 0x40},
    {OV7725_HSTART       , 0x3F},
    {OV7725_HSIZE        , 0x50},
    {OV7725_VSTRT        , 0x03},
    {OV7725_VSIZE        , 0x78},
    {OV7725_HREF         , 0x00},
    {OV7725_SCAL0        , 0x0A},
    {OV7725_AWB_Ctrl0    , 0xE0},
    {OV7725_DSPAuto      , 0xff},
    {OV7725_DSP_Ctrl2    , 0x0C},
    {OV7725_DSP_Ctrl3    , 0x00},
    {OV7725_DSP_Ctrl4    , 0x00},

#if (CAMERA_W == 80)
    {OV7725_HOutSize     , 0x14},
#elif (CAMERA_W == 160)
    {OV7725_HOutSize     , 0x28},
#elif (CAMERA_W == 240)
    {OV7725_HOutSize     , 0x3c},
#elif (CAMERA_W == 320)
    {OV7725_HOutSize     , 0x50},
#else

#endif

#if (CAMERA_H == 60 )
    {OV7725_VOutSize     , 0x1E},
#elif (CAMERA_H == 120 )
    {OV7725_VOutSize     , 0x3c},
#elif (CAMERA_H == 180 )
    {OV7725_VOutSize     , 0x5a},
#elif (CAMERA_H == 240 )
    {OV7725_VOutSize     , 0x78},
#else

#endif

    {OV7725_EXHCH        , 0x00},
    {OV7725_GAM1         , 0x0c},
    {OV7725_GAM2         , 0x16},
    {OV7725_GAM3         , 0x2a},
    {OV7725_GAM4         , 0x4e},
    {OV7725_GAM5         , 0x61},
    {OV7725_GAM6         , 0x6f},
    {OV7725_GAM7         , 0x7b},
    {OV7725_GAM8         , 0x86},
    {OV7725_GAM9         , 0x8e},
    {OV7725_GAM10        , 0x97},
    {OV7725_GAM11        , 0xa4},
    {OV7725_GAM12        , 0xaf},
    {OV7725_GAM13        , 0xc5},
    {OV7725_GAM14        , 0xd7},
    {OV7725_GAM15        , 0xe8},
    {OV7725_SLOP         , 0x20},
    {OV7725_LC_RADI      , 0x00},
    {OV7725_LC_COEF      , 0x13},
    {OV7725_LC_XC        , 0x08},
    {OV7725_LC_COEFB     , 0x14},
    {OV7725_LC_COEFR     , 0x17},
    {OV7725_LC_CTR       , 0x05},
    {OV7725_BDBase       , 0x99},
    {OV7725_BDMStep      , 0x03},
    {OV7725_SDE          , 0x04},
    {OV7725_BRIGHT       , 0x00},
    /*{OV7725_CNST         , 0x18},
    
#if (THRESHOLD_VALUE == 10 )
    {OV7725_CNST     , 10},
#elif (THRESHOLD_VALUE == 11 )
    {OV7725_CNST     , 11},
#elif (THRESHOLD_VALUE == 12 )
    {OV7725_CNST     , 12},
#elif (THRESHOLD_VALUE == 13 )
    {OV7725_CNST     , 13},
#elif (THRESHOLD_VALUE == 14 )
    {OV7725_CNST     , 14},
#elif (THRESHOLD_VALUE == 15 )
    {OV7725_CNST     , 15},
#elif   (THRESHOLD_VALUE == 16 )
    {OV7725_CNST     , 16},
#elif (THRESHOLD_VALUE == 17 )
    {OV7725_CNST     , 17},
#elif (THRESHOLD_VALUE == 18 )
    {OV7725_CNST     , 18},
#elif (THRESHOLD_VALUE == 19 )
    {OV7725_CNST     , 19},
#elif (THRESHOLD_VALUE == 20 )
    {OV7725_CNST     , 20},
#elif (THRESHOLD_VALUE == 21 )
    {OV7725_CNST     , 21},
#elif (THRESHOLD_VALUE == 22 )
    {OV7725_CNST     , 22},
#elif (THRESHOLD_VALUE == 23 )
    {OV7725_CNST     , 23},
#elif (THRESHOLD_VALUE == 24 )
    {OV7725_CNST     , 24},
#elif (THRESHOLD_VALUE == 25 )
    {OV7725_CNST     , 25},
#elif (THRESHOLD_VALUE == 26 )
    {OV7725_CNST     , 26},
#elif (THRESHOLD_VALUE == 27 )
    {OV7725_CNST     , 27},
#elif (THRESHOLD_VALUE == 28 )
    {OV7725_CNST     , 28},
#elif (THRESHOLD_VALUE == 29 )
    {OV7725_CNST     , 29},
#elif (THRESHOLD_VALUE == 30 )
    {OV7725_CNST     , 30},
#elif (THRESHOLD_VALUE == 31 )
    {OV7725_CNST     , 31},
#elif (THRESHOLD_VALUE == 32 )
    {OV7725_CNST     , 32},
#elif (THRESHOLD_VALUE == 33 )
    {OV7725_CNST     , 33},
#elif (THRESHOLD_VALUE == 34 )
    {OV7725_CNST     , 34},
#elif (THRESHOLD_VALUE == 35 )
    {OV7725_CNST     , 35},
#elif (THRESHOLD_VALUE == 36 )
    {OV7725_CNST     , 36},
#elif (THRESHOLD_VALUE == 37 )
    {OV7725_CNST     , 37},
#elif (THRESHOLD_VALUE == 38 )
    {OV7725_CNST     , 38},
#elif (THRESHOLD_VALUE == 39 )
    {OV7725_CNST     , 39},
#elif (THRESHOLD_VALUE == 40 )
    {OV7725_CNST     , 40},
#elif (THRESHOLD_VALUE == 41 )
    {OV7725_CNST     , 41},
#elif (THRESHOLD_VALUE == 42 )
    {OV7725_CNST     , 42},
#elif (THRESHOLD_VALUE == 43 )
    {OV7725_CNST     , 43},
#elif (THRESHOLD_VALUE == 44 )
    {OV7725_CNST     , 44},
#elif (THRESHOLD_VALUE == 45 )
    {OV7725_CNST     , 45},
#elif (THRESHOLD_VALUE == 46 )
    {OV7725_CNST     , 46},
#elif (THRESHOLD_VALUE == 47 )
    {OV7725_CNST     , 47},
#elif (THRESHOLD_VALUE == 48 )
    {OV7725_CNST     , 48},
#elif (THRESHOLD_VALUE == 49 )
    {OV7725_CNST     , 49},
#elif (THRESHOLD_VALUE == 50 )
    {OV7725_CNST     , 50},
#else

#endif*/
    {OV7725_SIGN         , 0x06},
    {OV7725_UVADJ0       , 0x11},
    {OV7725_UVADJ1       , 0x02},
};


uint8 ov7725_eagle_cfgnum = ARR_SIZE( ov7725_eagle_reg ) ; /*�ṹ�������Ա��Ŀ*/


/*!
 *  @brief      ӥ��ov7725�Ĵ��� ��ʼ��
 *  @return     ��ʼ�������0��ʾʧ�ܣ�1��ʾ�ɹ���
 *  @since      v5.0
 *  13���ű��޸ģ����һ�����ݲ���(uint8 threshold)��
 *  ���ܣ�������ֵ��ֱ�Ӱ����޸ģ�SCCB_WriteByte(OV7725_CNST ,threshold);
 */
uint8 ov7725_eagle_reg_init(uint8 threshold)
{
    uint16 i = 0;
    uint8 Sensor_IDCode = 0;
    SCCB_GPIO_init();

    //OV7725_Delay_ms(50);
    if( 0 == SCCB_WriteByte ( OV7725_COM7, 0x80 ) ) /*��λsensor */
    {
        DEBUG_PRINTF("\n����:SCCBд���ݴ���");
        return 0 ;
    }

    OV7725_EAGLE_Delay_ms(50);

    if( 0 == SCCB_ReadByte( &Sensor_IDCode, 1, OV7725_VER ) )    /* ��ȡsensor ID��*/
    {
        DEBUG_PRINTF("\n����:��ȡIDʧ��");
        return 0;
    }
    DEBUG_PRINTF("\nGet ID success��SENSOR ID is 0x%x", Sensor_IDCode);
    DEBUG_PRINTF("\nConfig Register Number is %d ", ov7725_eagle_cfgnum);
    if(Sensor_IDCode == OV7725_ID)
    {
      SCCB_WriteByte(OV7725_CNST ,threshold);
        for( i = 0 ; i < ov7725_eagle_cfgnum ; i++ )
        {
            if( 0 == SCCB_WriteByte(ov7725_eagle_reg[i].addr, ov7725_eagle_reg[i].val) )
            {
                DEBUG_PRINTF("\n����:д�Ĵ���0x%xʧ��", ov7725_eagle_reg[i].addr);
                return 0;
            }
        }
    }
    else
    {
        return 0;
    }
    DEBUG_PRINTF("\nOV7725 Register Config Success!");
    return 1;
}

//***************************LQ_MT9V034****************************************//
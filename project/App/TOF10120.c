#include "TOF10120.h"

#define CHECK_SDA             gpio_get (TOF_SDA)
#define SDA_H()          gpio_set (TOF_SDA, 1)		//IO������ߵ�ƽ
#define SDA_L()          gpio_set (TOF_SDA, 0)		//IO������͵�ƽ
#define SCL_H()          gpio_set (TOF_SCL, 1)		//IO������ߵ�ƽ
#define SCL_L()          gpio_set (TOF_SCL, 0)		//IO������͵�ƽ
#define TOF_DIR_OUT()       gpio_ddr (TOF_SDA, GPO)    //�������
#define TOF_DIR_IN()        gpio_ddr (TOF_SDA, GPI)    //���뷽��

#define TOF_ack 1      //��Ӧ��
#define TOF_no_ack 0   //��Ӧ��

//**************************************�ڲ����ú���************************************//

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ģ��IIC��ʱ
//  @return     void						
//  @since      v1.0
//  Sample usage:				���IICͨѶʧ�ܿ��Գ�������j��ֵ
//-------------------------------------------------------------------------------------------------------------------
void TOF_delay(void)
{
  //64Ϊ100K������(busƵ��Ϊ100M)
  //�ر���ʾOV7725��ͨ�����ʲ���̫�ߣ����50K���ң�j����Ϊ120ͨ������Ϊ60K��60K��ʱ���������ͨ��
  //����IIC����һ�����400K��ͨ������
  uint16 j=400;   
  while(j--);
}

void TOF_start(void)
{
  SDA_H();
  SCL_H();
  TOF_delay();
  SDA_L();
  TOF_delay();
  SCL_L();
}

void TOF_stop(void)
{
  SDA_L();
  SCL_L();
  TOF_delay();
  SCL_H();
  TOF_delay();
  SDA_H();
  TOF_delay();
}

//��Ӧ��(����ack:SDA=0��no_ack:SDA=0)
//�ڲ�ʹ�ã��û��������
void TOF_SendACK(unsigned char ack_dat)
{
  SCL_L();
  TOF_delay();
  if(ack_dat)
    SDA_L();
  else
    SDA_H();
  SCL_H();
  TOF_delay();
  SCL_L();
  TOF_delay();
}

static int WaitAck(void)
{
  SCL_L();
  TOF_DIR_IN();
  TOF_delay();
  
  SCL_H();
  TOF_delay();
  
  if(CHECK_SDA)           //Ӧ��Ϊ�ߵ�ƽ���쳣��ͨ��ʧ��
  {
    TOF_DIR_OUT();
    SCL_L();
    return 0;
  }
  TOF_DIR_OUT();
  SCL_L();
  TOF_delay();
  return 1;
}


//�ֽڷ��ͳ���
//����c(����������Ҳ���ǵ�ַ)���������մ�Ӧ��
//�����Ǵ�Ӧ��λ
//�ڲ�ʹ�ã��û��������
void TOF_send_ch(uint8 c)
{
  uint8 i = 8;
  while(i--)
  {
    if(c & 0x80)
      SDA_H();//SDA �������
    else
      SDA_L();
    c <<= 1;
    TOF_delay();
    SCL_H();                //SCL ���ߣ��ɼ��ź�
    TOF_delay();
    SCL_L();                //SCL ʱ��������
  }
  WaitAck();
}

//�ֽڽ��ճ���
//�����������������ݣ��˳���Ӧ���|��Ӧ����|IIC_ack_main()ʹ��
//�ڲ�ʹ�ã��û��������
uint8 TOF_read_ch(void)
{
  uint8 i;
  uint8 c;
  c=0;
  SCL_L();
  TOF_delay();
  SDA_H();             //��������Ϊ���뷽ʽ
  TOF_DIR_IN();
  for(i=0;i<8;i++)
  {
    TOF_delay();
    SCL_L();         //��ʱ����Ϊ�ͣ�׼����������λ
    TOF_delay();
    SCL_H();         //��ʱ����Ϊ�ߣ�ʹ��������������Ч
    TOF_delay();
    c<<=1;
    if(CHECK_SDA) c+=1;   //������λ�������յ����ݴ�c
  }
  TOF_DIR_OUT();
  SCL_L();
  TOF_delay();
  TOF_SendACK(TOF_no_ack);
  
  return c;
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      ģ��IIC���豸�Ĵ�����ȡ����
//  @param      dev_add			�豸��ַ(����λ��ַ)
//  @param      reg				�Ĵ�����ַ
//  @param      type			ѡ��ͨ�ŷ�ʽ��IIC  ���� SCCB
//  @return     uint8			���ؼĴ���������			
//  @since      v1.0
//  Sample usage:				
//-------------------------------------------------------------------------------------------------------------------
uint8 read_reg(uint8 dev_add, uint8 reg, type type)
{
  uint8 dat;
  TOF_start();
  TOF_send_ch( (dev_add<<1) | 0x00);  //����������ַ��дλ
  TOF_send_ch( reg );   				//���ʹӻ��Ĵ�����ַ
  if(type == TOF_SCCB)
    TOF_stop();
  TOF_start();
  TOF_send_ch( (dev_add<<1) | 0x01);  //����������ַ�Ӷ�λ
  dat = TOF_read_ch();   				//������Ҫд�������
  TOF_stop();
  
  return dat;
}


void TOF_init(void)
{
  gpio_init (TOF_SCL, GPO,1);
  gpio_init (TOF_SDA, GPO,1);
  port_init_NoALT (TOF_SCL, ODO | PULLUP);//ODO
  port_init_NoALT (TOF_SDA, ODO | PULLUP);
}

void TOF_Measurement(uint8 *data)
{
    uint8 TOF_L;
    uint16 TOF_H;
    TOF_H = read_reg(TOF10120_DEV_ADDR, 0x00, TOF_IIC);
    TOF_L = read_reg(TOF10120_DEV_ADDR, 0x00+1, TOF_IIC);
    *data=((TOF_H*256)+TOF_L)/10;//�ϳ�����
}


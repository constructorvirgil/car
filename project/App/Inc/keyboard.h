#ifndef __KEYBOARD_H
#define __KEYBOARD_H

#define CH451_RESET     0x0201             //��λ
#define CH451_LEFTMOV   0x0300             //�����ƶ���ʽ-����
#define CH451_LEFTCYC   0x0301             //�����ƶ���ʽ-��ѭ
#define CH451_RIGHTMOV  0x0302             //�����ƶ���ʽ-����
#define CH451_RIGHTCYC  0x0303             //�����ƶ���ʽ-��ѭ 
#define CH451_SYSOFF    0x0400             //����ʾ�����̡����Ź�
#define CH451_SYSON1    0x0401             //����ʾ
#define CH451_KEYEN     0x0402             //������
#define CH451_SYSON2    0x0403             //����ʾ������
#define CH451_SYSON3    0x0407             //����ʾ�����̡����Ź�����
#define CH451_DSP       0x0500             //����Ĭ����ʾ��ʽ
#define CH451_BCD       0x0580             //����BCD���뷽ʽ
#define CH451_TWINKLE   0x0600             //������˸����
#define CH451_KEY       0x0700             //�����̴���
#define CH451_DIG0      0x0800             //�����λ0��ʾ
#define CH451_DIG1      0x0900             //�����λ1��ʾ 
#define CH451_DIG2      0x0A00             //�����λ2��ʾ
#define CH451_DIG3      0x0B00             //�����λ3��ʾ
#define CH451_DIG4      0x0C00             //�����λ4��ʾ
#define CH451_DIG5      0x0D00             //�����λ5��ʾ 
#define CH451_DIG6      0x0E00             //�����λ6��ʾ 
#define CH451_DIG7      0x0F00             //�����λ7��ʾ
 
#define LOAD   PTA14          // 3    
#define DIN    PTA12          // 2    
#define DCLK   PTA15        // 1    
#define DOUT   PTA13         // 4    
/*�޸Ķ˿�ʱ�����*/

uint8 KeyCode[9]={0x41,0x42,0x43,//1,2,3
                  0x49,0x4A,0x4B,//4,5,6
                  0x51,0x52,0x53,};//7,8,9

uint8 KeyValue;
uint8 GetKeyValue=0;

void CH451_WriteCommand(uint32 Command)
{                                                                  //��CH451д����
  uint8 k=12;
  gpio_set(LOAD,0);                                                //gpio_ctrl(key_port,Load,0); 
  for(k=0;k<12;k++){
	 gpio_set(DIN,Command&1);                                       //gpio_ctrl(key_port,IN,Command&1);         
	 gpio_set(DCLK,0);                                               //gpio_ctrl(key_port,CLK,0);                
	 Command>>=1;                                                     //��������һλ
	 gpio_set(DCLK,1);                                               //gpio_ctrl(key_port,CLK,1);
  }
  gpio_set(LOAD,1);                                               //gpio_ctrl(key_port,Load,1);
}

void CH451_GetKeyValue(void)
{
    uint8 k;
    uint8 Command=0x07;                                            //��ȡ��ֵ����
    GetKeyValue=0x00;                                              //���̴���
                                                                   //CH451_Load=0;
    gpio_set(LOAD,0);                                             //gpio_ctrl(key_port,Load,0);
                                                                   //���ʼ       
    for(k=0;k<4;k++)
    {                                                              //����4λ���ݣ���λ��ǰ
                                                                   //CH451_IN=Command&0x0001; 
    gpio_set(DIN,Command&0X0001);                                //gpio_ctrl(key_port,IN,Command&0X0001);
                                                                   //CH451_CLK=0;
    gpio_set(DCLK,0);                                             //gpio_ctrl(key_port,CLK,0);
      Command>>=1;
                                                                   //CH451_CLK=1;                   //��������Ч
    gpio_set(DCLK,1);                                             // gpio_ctrl(key_port,CLK,1);
    }
                                                                   // CH451_Load=1;                     //�����ؼ�������
   gpio_set(LOAD,1);                                              // gpio_ctrl(key_port,Load,1);    
    for(k=0;k<7;k++)
    {
        GetKeyValue<<=1;
        GetKeyValue|=gpio_get (DOUT);                             //gpio_Get(key_port,Out);//CH451_Out;
                                                                   //CH451_CLK=1;               //�½�����Ч
        gpio_set(DCLK,1);                                          //gpio_ctrl(key_port,CLK,1);
                                                                   //CH451_CLK=0;
        gpio_set(DCLK,0);
    }
                                                                   //���Ҽ��̴����Ӧ�ļ�ֵ
    for(k=0;k<12;k++)
    {                
      if(GetKeyValue==KeyCode[k])
      {  
        KeyValue=k+1;  
        break;
      }
    }
}
void CH451_ini(void)
{
  gpio_set(DIN,1);                                            // gpio_ctrl(key_port,IN,1);
  gpio_set(DIN,0); 
  gpio_set(DIN,1);                                              // gpio_ctrl(key_port,IN,1);
  CH451_WriteCommand(CH451_RESET);                                   //��λCH451
  CH451_WriteCommand(CH451_SYSON2); 
}

void JIANPAN_ini(void)         //���̿��ƶ˿ڳ�ʼ��
{
  gpio_init(DIN,GPO,1);
  gpio_init(DCLK,GPO,1);
  gpio_init(LOAD,GPO,1);
  gpio_init(DOUT,GPI,1);
  
}
#endif
#ifndef	_LCD_H_
#define _LCD_H_

#define RESET  PTC18_OUT
#define DC     PTC19_OUT
#define D1     PTC17_OUT
#define D0     PTC16_OUT

 typedef unsigned char byte;
 typedef unsigned int word;
 extern byte beyond96x64[768];
 
 void LCD_Init(void);
 void LCD_CLS(void);
 void Display(int16 i,uint8 x,uint8 y);
 void DisplayFloat(int16 i,uint8 x,uint8 y);
 void LCD_P6x8Str(byte x,byte y,byte ch[]);
 void LCD_P8x16Str(byte x,byte y,byte ch[]);
 void LCD_P14x16Str(byte x,byte y,byte ch[]);
 void LCD_Print(byte x, byte y, byte ch[]);
 void LCD_PutPixel(uint8 x,uint8 y,uint8 data1);
 void LCD_Rectangle(byte x1,byte y1,byte x2,byte y2,byte gif);
 void Draw_BMP(byte x0,byte y0,byte x1,byte y1,byte bmp[]); 
 void LCD_Fill(byte dat);
 void LCD_P6x8Str1(byte x,byte y,uint8 date);
 void LCD_P6x8Str2(byte x,byte y,uint8 date);
 void LCD_P6x8Str3(byte x,byte y,uint16 date); 
 void LCD_P6x8Str4(byte x,byte y,uint16 date);
 void LCD_P6x8Str5(uint8 x,uint8 y,uint32 date);
 void LCD_P6x8Str_1(byte x,byte y,int8 date);
 void LCD_P6x8Str_2(byte x,byte y,int8 date);
 void LCD_P6x8Str_3(byte x,byte y,int16 date);
 void LCD_P6x8Str_4(byte x,byte y,int16 date);
 void LCD_P6x8Str_5(uint8 x,uint8 y,int32 date);
 void LCD_WrDat(uint8 data);
 void LCD_Set_Pos(byte x, byte y);
 
#endif
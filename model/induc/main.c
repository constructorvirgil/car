/**
* @file main.c
* @brief 电感测试
* @author virgil
* @email 275380431@qq.com
* @version v1.0
* @date 2019-11-05
*/

#include "common.h"
#include "include.h"
#include "a_lcd.h"
#include "induc.h"

//test inc
#include "lcd.h"


int main()
{
    DisableInterrupts;
    
    LCD_Init();
    LCD_CLS();
    LCD_P6x8Str(24,3,"INDUC unit test");
	systick_delay_ms(1000);

	induc_init();

	while(1)
	{
		induc_update_no_filter();
		induc_display();
		systick_delay_ms(100);
	}

	return 0;
}


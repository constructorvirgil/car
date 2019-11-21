/**
* @file main.c
* @brief 编码器测试
* @author virgil
* @email 275380431@qq.com
* @version v1.0
* @date 2019-11-05
*/

#include "common.h"
#include "include.h"
#include "a_lcd.h"
#include "motor.h"
#include "encoder.h"
#include <limits.h>

//test inc
#include "lcd.h"

void run_car()
{
	int c = INT_MAX;
	int duty = -5000;
	while(duty <5000)
	{
		//每隔1s变一次速
		if(c > 20)
		{
			c = 0;
			motor_run(duty);
			duty += 500;
		}		
		speed_measure();
		speed_display();
		c++;
		systick_delay_ms(50);
	}
	while(1);
}


int main()
{
    DisableInterrupts;

    LCD_Init();
    LCD_CLS();
    LCD_P6x8Str(24,3,"Encoder unit test");
	systick_delay_ms(1000);

	//只创建一个菜单,从反向减速到正向加速,测试速度变化
	UNIT* unit_top = create_top("run car");

	add_unit_handler(unit_top, "go", run_car);

	motor_init();
	encoder_init();

	menu_start();

	return 0;
}


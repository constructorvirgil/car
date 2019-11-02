#include "LCD_Menu.h"
#include "common.h"
#include "stdio.h"
#include "include.h"
#include "lcd.h"
#include "control.h"
#include "TOF10120.h"

MENU *topMenu;
MENU *nowMenu;
uint8 first_create = 1;
ITEM items[48];
int pos = 0;

/***************************************************************  
*  @brief     	����һ���˵��Լ�����������һ���˵�
*  @param     
				p_xxxx:ǰ�ĸ�������Ҫ��ʾ���ַ���,˳��,��������,���ҿ���5���ַ�����,�м�4���ַ�,����8���ַ�
				pp_menu:ָ����һ���˵��Ķ���ָ��,�Ա���չ�˵�
				p_bak:ָ����һ���˵���ָ��,���򿪹ذ����м����᷵����һ���˵�
*  @note      	
*  @return 		����ָ��˵���ָ��
**************************************************************/
MENU *CreateMenu(const char *p_left, const char *p_right, const char *p_up, const char *p_down, const char *p_centre, MENU **pp_menu, MENU *p_bak)
{
	MENU *p_menu = (MENU *)malloc(sizeof(MENU));
	if (first_create == 1)
	{
		first_create = 0;
		topMenu = p_menu;
		nowMenu = topMenu;
	}
	else
	{
		*pp_menu = p_menu; //��д��������ָ��
	}
	p_menu->left = p_left;
	p_menu->left_next = NULL;

	p_menu->right = p_right;
	p_menu->right_next = NULL;

	p_menu->up = p_up;
	p_menu->up_next = NULL;

	p_menu->down = p_down;
	p_menu->down_next = NULL;

	p_menu->centre = p_centre;
	p_menu->centre_next = p_bak;

	return p_menu;
}


static void PrintMenu()
{
	LCD_CLS();
	if (nowMenu->left != NULL)
	{
		LCD_P6x8Str(0, 4, (uint8_t *)nowMenu->left);
	}
	if (nowMenu->right != NULL)
	{
		LCD_P6x8Str(90, 4, (uint8_t *)nowMenu->right);
	}
	if (nowMenu->up != NULL)
	{
		LCD_P6x8Str(48, 1, (uint8_t *)nowMenu->up);
	}
	if (nowMenu->down != NULL)
	{
		LCD_P6x8Str(48, 7, (uint8_t *)nowMenu->down);
	}
	if (nowMenu->centre != NULL)
	{
		LCD_P6x8Str(53, 4, (uint8_t *)nowMenu->centre);
	}
}

static void Goto(_DIR dir)
{
	switch (dir)
	{
	case LEFT:
		if (nowMenu->left_next != NULL)
		{
			nowMenu = nowMenu->left_next;
		}
		else
		{
			ExcuteCmd(nowMenu->left);
		}
		break;
	case RIGHT:
		if (nowMenu->right_next != NULL)
		{
			nowMenu = nowMenu->right_next;
		}
		else
		{
			ExcuteCmd(nowMenu->right);
		}
		break;

	case UP:
		if (nowMenu->up_next != NULL)
		{
			nowMenu = nowMenu->up_next;
		}
		else
		{
			ExcuteCmd(nowMenu->up);
		}
		break;

	case DOWN:
		if (nowMenu->down_next != NULL)
		{
			nowMenu = nowMenu->down_next;
		}
		else
		{
			ExcuteCmd(nowMenu->down);
		}
		break;
	case CENTRE:
		if (nowMenu->centre_next != NULL)
		{
			nowMenu = nowMenu->centre_next;
		}
		else
		{
			ExcuteCmd(nowMenu->centre);
		}
		break;
	}
	PrintMenu();
}

/***************************************************************  
*  @brief     	���һ���ַ����Իص�������ӳ��
*  @param     
				str:�ص�������ʶ,�ַ���
				hand_func:�ص�����ָ��
*  @note      	��ѡ�ж�Ӧ���ַ�����ʱ��,���Զ����ö�Ӧ����ַ����Ļص�����
				��Ϊ��ͨ���ַ�������ʽ��ƥ��ص�������,�����ַ�����ʶӦ��Ψһ
**************************************************************/
void AddHandle(const char *str, HD hand_func)
{
	ITEM item;
	strcpy(item.str, str);
	item.p_func = hand_func;
	items[pos] = item;
	pos++;
}

static void ExcuteCmd(const char *str)
{
	if (str == NULL)
	{
		return;
	}

	for (int i = 0; i < 48; i++)
	{
		if (strcmp(items[i].str, str) == 0)
		{
			(*items[i].p_func)();
			break;
		}
	}

	nowMenu = topMenu;
}

/***************************************************************  
*  @brief     	��ʾ�˵�
*  @param     	
*  @note      	
**************************************************************/
void MenuStart(void)
{
	_KEY_CODE key;
	PrintMenu();
	while (1)
	{
		systick_delay_ms(500);
		key = Keyboardscanf();
		switch (key)
		{
		case _KEY_LEFT:
			Goto(LEFT);
			break;
		case _KEY_RIGHT:
			Goto(RIGHT);
			break;
		case _KEY_UP:
			Goto(UP);
			break;
		case _KEY_DOWN:
			Goto(DOWN);
			break;
		case _KEY_CENTRE:
			Goto(CENTRE);
			break;
		default:
			break;
		}
	}
}
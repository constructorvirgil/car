/**
* @file a_lcd.c
* @brief 在lcd基础模块上拓展的lcd应用
* @author virgil
* @email 275380431@qq.com
* @version v1.0
* @date 2019-11-05
*/

#include "a_lcd.h"
#include "common.h"
#include "lcd.h"
#include "key.h"

//** 一屏只显示三个单元 */
char s_labels[3][LABEL_MAXLEN]={"","",""};
//** 记录第一层第一个单元 */
static UNIT* s_top;
/** 记录当前单元 */
static UNIT* s_first;



/**
* @param _label 要显示的字符串
* @return 返回指向新增单元的指针,便于以后继续增加单元
* @note 第一层不设置TYPE_PARAM,TPYE_MOTION型单元(即只有TYPE_OPTION类型) @see UNIT_TYPE
*/
UNIT* create_top(const char* _label)
{
    s_top = (UNIT*)calloc(1,sizeof(UNIT));
    strcpy(s_top->label,_label);
    s_top->type = TYPE_OPTION;

    s_first = s_top;
    return s_top;
}

/**
* @param _unit  要在哪个单元后添加
* @param _label 要显示的字符串
* @param _set_value 选中该单元后,对变量赋什么值
* @param _p_set_value 指向要被复制的变量
* @return 返回指向新增单元的指针,便于以后继续增加单元
* @note 如果_unit是TYPE_OPTION,会在右边添加;如果_unit是TYPE_PARAM,会在下方添加
*/
UNIT* add_unit_param(UNIT* _unit,const char * _label,int _set_value,int* _p_set_value)
{
    //new unit
    UNIT* p ;
    p = (UNIT*)calloc(1,sizeof(UNIT));
    strcpy(p->label,_label);
    p->type = TYPE_PARAM;
    p->set_value = _set_value;
    p->p_set_value = _p_set_value;

    if(_unit->type == TYPE_OPTION)
    {
        p->left = _unit;
        //last unit
        _unit->right = p;
    }else if(_unit->type == TYPE_PARAM)
    {
        p->up = _unit;
        //last unit
        _unit->down = p;
    }

    return p;
}
/**
* @param _unit  要在哪个单元后添加
* @param _label 要显示的字符串
* @param _p_func 选中该单元后,会调用哪个函数执行
* @return 返回指向新增单元的指针,便于以后继续增加单元
*/
UNIT* add_unit_handler(UNIT* _unit,const char * _label,P_FUNC_HANDLE _p_func)
{
    //new unit
    UNIT* p ;
    p = (UNIT*)calloc(1,sizeof(UNIT));
    strcpy(p->label,_label);
    p->type = TYPE_MOTION;
    p->left = _unit;
    p->p_func = _p_func;
    //last unit
    _unit->right = p;
    return p;
}

/**
* @param _unit  要在哪个单元后添加
* @param _label 要显示的字符串
* @return 返回指向新增单元的指针,便于以后继续增加单元
*/
UNIT* add_unit_right(UNIT* _unit,const char * _label)
{
    //new unit
    UNIT* p ;
    p = (UNIT*)calloc(1,sizeof(UNIT));
    strcpy(p->label,_label);
    p->type = TYPE_OPTION;
    p->left = _unit;

    //last unit
    _unit->right = p;
    return p;
}

/**
* @param _unit  要在哪个单元后添加
* @param _label 要显示的字符串
* @return 返回指向新增单元的指针,便于以后继续增加单元
*/
UNIT*  add_unit_down(UNIT* _unit,const char * _label)
{
    //new unit
    UNIT* p ;
    p = (UNIT*)calloc(1,sizeof(UNIT));
    strcpy(p->label,_label);
    p->type = TYPE_OPTION;
    p->up = _unit;

    //last unit
    _unit->down = p;
    return p;

}

/**
* @note move_up,move_down,move_left,move_right均只改变s_first的值
*/ 
static void move_up(void)
{
    if(s_first->up != NULL)
    {
        s_first = s_first->up;
    }
    update_labels();
}

static void move_down(void)
{
    if(s_first->down != NULL)
    {
        s_first = s_first->down;
    }
    update_labels();
}

static void move_left()
{
    if(s_first->left != NULL)
    {
        s_first = s_first->left;
    }
    update_labels();
}


/**
* @note 当前单元被选中
*/
static void move_right(void)
{
    if(s_first->type == TYPE_PARAM)//如果是要设置参数
    {
        *s_first->p_set_value = s_first->set_value;
        s_first = s_top;//菜单回到顶层
    }else if(s_first->type == TYPE_MOTION)//如果是要执行函数
    {
        (*s_first->p_func)();
        s_first = s_top;//菜单回到顶层
    }else{//跳到下一级菜单
        if(s_first->right != NULL)
        {
            s_first = s_first->right;
        }
    }
    update_labels();
}

/**
* @param _unit  要在哪个单元后添加
* @param _label 要显示的字符串
* @return 返回指向新增单元的指针,便于以后继续增加单元
*/
static void update_labels(void)
{
	//无需把整个字符串清空
    s_labels[0][0]='\0';
    s_labels[1][0]='\0';
    s_labels[2][0]='\0';

    //当前一个不为空才打印下一个
    if(s_first != NULL)
    {
        strcpy(s_labels[0],s_first->label);
        if(s_first->down != NULL)
        {
            strcpy(s_labels[1],s_first->down->label);
            if(s_first->down->down != NULL)
            {
                strcpy(s_labels[2],s_first->down->down->label);
            }
        }

    }

}


static void lcd_print_menu(void)
{
    LCD_CLS();
    LCD_P6x8Str(12,3,">>");
    LCD_P6x8Str(36,3,(byte *)s_labels[0]);
    LCD_P6x8Str(36,5,(byte *)s_labels[1]);
    LCD_P6x8Str(36,7,(byte *)s_labels[2]);
}


/**
 * @note 程序会被阻塞在此,根据被选中的单元执行对应操作
 */
void menu_start(void)
{
    //初始化LCD
    LCD_Init();
	LCD_CLS();

    //初始化五向开关
    five_key_init();

    //先刷新一次屏幕
    update_labels();
    lcd_print_menu();

	KEY_CODE_EN key;
    while(1)
    {
        key = key_scanf();
        if(key == KEY_CODE_UP)
        {
            move_up();
        }else if(key == KEY_CODE_DOWN)
        {
            move_down();
        }else if(key == KEY_CODE_LEFT)
        {
            move_left();
        }else if(key == KEY_CODE_RIGHT)
        {
            move_right();
        }else if(key == KEY_CODE_CENTER)
        {
            move_right();
        }
        lcd_print_menu();
        systick_delay_ms(500);//防抖处理
    }
}


/**
* @note 递归的具体实现
*/
static void recur_close(UNIT* top)
{
	UNIT* mainmenu = top;
	UNIT* mainmenu_last;
	//先到主菜单尾
	while (mainmenu->down != NULL)
	{
		mainmenu = mainmenu->down;
	}
	//从主菜单尾部往前逐个回收内存
	while (mainmenu != top)
	{
		mainmenu_last = mainmenu->up;
		if (mainmenu->right != NULL)
		{
			recur_close(mainmenu->right);
		}
		free(mainmenu);//回收头结点
		mainmenu = mainmenu_last;
	}
	free(top);//回收头结点
}

/**
* @note 使用递归的方式回收所有内存
*/
void menu_close(void)
{
    recur_close(s_top);
}
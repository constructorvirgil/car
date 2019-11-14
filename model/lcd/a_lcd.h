/**
 * @file a_lcd.c
 * @brief 在lcd基础模块上拓展的lcd应用
 * @author virgil
 * @email 275380431@qq.com
 * @version v1.0
 * @date 2019-11-05
 */

#ifndef  __A_LCD_H
#define __A_LCD_H

/** 每个选项字符串最大长度 */
#define LABEL_MAXLEN 9

/** 参数void,返回值void型函数指针 */
typedef void(*P_FUNC_HANDLE)(void);

/**
 * @note 每个菜单有多个选项,每个选项都视为一个单元,不同的类型的选项被选中后会有不同的行为,
 * TYPE_OPTION无行为,TYPE_PARAM,会对一个变量赋值,TYPE_MOTION会调用一个函数
 */
typedef enum{
    TYPE_OPTION,
    TYPE_PARAM,
    TYPE_MOTION
}UNIT_TYPE;

/** 存储单元结构体 */
typedef struct __UNIT{
    char label[LABEL_MAXLEN];
    struct __UNIT * up;
    struct __UNIT * down;
    struct __UNIT * left;
    struct __UNIT * right;
    UNIT_TYPE type;
    int set_value;
    int* p_set_value;
    P_FUNC_HANDLE p_func;
}UNIT;

/** 创建第一层菜单的第一个单元 */
UNIT* create_top(const char* _label);
/** 向右/向下添加一个TYPE_PARAM类型的单元 */
UNIT* add_unit_param(UNIT* _unit,const char * _label,int _set_value,int* _p_set_value);
/** 向右添加一个TYPE_MOTION类型的单元 */
UNIT* add_unit_handler(UNIT* _unit,const char * _label,P_FUNC_HANDLE _p_func);
/** 向右添加一个TYPE_OPTION类型的单元 */
UNIT* add_unit_right(UNIT* _unit,const char * _label);
/** 向下添加一个TYPE_OPTION类型的单元 */
UNIT* add_unit_down(UNIT* _unit,const char * _label);
/** 进入菜单(阻塞),检测按键, 更新界面,根据当前单元执行动作 */
void menu_start(void);
/** 销毁菜单,回收内存 */
void menu_close(void);
/** 更新s_first */
static void move_up(void);
/** 更新s_first */
static void move_down(void);
/** 更新s_first */
static void move_down(void);
/** 更新s_first */
static void move_left(void);
/** 更新s_first */
static void move_right(void);
/** 更新s_labels */
static void update_labels(void);
/** 在lcd 上打印界面 */
static void lcd_print_menu(void);
/** 销毁菜单,回收内存 */
void menu_close(void);
/** menu_close 会调用该函数来递归回收内存 */
static void recur_close(UNIT* top);

#endif
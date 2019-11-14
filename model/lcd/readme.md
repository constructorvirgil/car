# LCD菜单显示模块

## 环境

国赛主力车

## 文件说明

`key.c` `key.h` `lcd.c` `lcd.h` 提供了菜单的底层实现

`a_lcd.c` `a_lcd.h` 是对它们的封装

`main.c`  里面有可用的实例代码

## 注

-   移植

    lcd引脚配置在 `lcd.h`  `lcd.c` 中

    五向开关引脚配置在 `key.c`  `key.h中`

    如果移植,按需更改这几个文件

-   `key.c` `key.h`相较以前的代码做了些优化

-   `lcd.c` `lcd.h` 主要变动的是引脚部分


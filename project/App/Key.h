#ifndef __KEY_H
#define __KEY_H

#include "common.h"
#include "stdio.h"
#include "include.h"
#include "lcd.h"
#include "control.h"
#include "TOF10120.h"

typedef enum 
{
    _KEY_NULL,
    _KEY_LEFT,
    _KEY_RIGHT,
    _KEY_UP,
    _KEY_DOWN,
    _KEY_CENTRE
}_KEY_CODE;

_KEY_CODE Keyboardscanf(void);

#endif
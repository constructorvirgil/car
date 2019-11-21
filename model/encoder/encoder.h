#ifndef __ENCODER_H
#define __ENCODER_H


#define ENCODER_LEFT    FTM1
#define ENCODER_RIGHT   FTM2

void encoder_init();
void speed_measure(void);
void speed_display();

#endif
//
// Created by Y on 2024/7/5.
//

#ifndef DBUS_1_SCREW_H
#define DBUS_1_SCREW_H

#include "tim.h"


class Screw
{
public:
    Screw();
    Screw(TIM_HandleTypeDef *htim, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
    ~Screw();

private:
    TIM_HandleTypeDef *htim;
    GPIO_TypeDef* GPIOx;
    uint16_t GPIO_Pin;
    uint16_t nowPos = 0;  //0~2000
    uint16_t targetPos = 0;
    uint16_t PWMnum = 0;
    bool direction = 0;  //0上1下

    bool checkPos(uint16_t pos);
    bool getDirection(uint16_t pos);

public:
    void writePos(uint16_t pos);
    uint16_t readPos();
    void OnHAL_TIM_PWM_PulseFinishedCallback();
};


#endif //DBUS_1_SCREW_H

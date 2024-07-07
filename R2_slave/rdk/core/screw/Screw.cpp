//
// Created by Y on 2024/7/6.
//

#include "Screw.h"

//
// Created by Y on 2024/7/5.
//

#include "Screw.h"

Screw::Screw()
{

}

Screw::Screw(TIM_HandleTypeDef *htim, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    this->htim = htim;
    this->GPIOx = GPIOx;
    this->GPIO_Pin = GPIO_Pin;
}

Screw::~Screw()
{

}

bool Screw::checkPos(uint16_t pos)
{
    bool isOK = 0;
    if(pos <= 10000 && pos >= 0)
    {
        isOK = 1;
    }
    return isOK;
}

bool Screw::getDirection(uint16_t pos)
{
    bool isOK = this->checkPos(pos);
    if(isOK == 0)
    {
        return -1;
    }

    bool nowDirection = 0;
    this->targetPos = pos;
    if(this->targetPos > this->nowPos)
    {
        nowDirection = 1;
    }
    else if(this->targetPos < this->nowPos)
    {
        nowDirection = 0;
    }

    this->direction = nowDirection;
    return this->direction;
}

void Screw::writePos(uint16_t pos)
{
    bool nowDirection = this->getDirection(pos);
    this->targetPos = pos;
    if(this->targetPos - this->nowPos >= 5 || this->targetPos - this->nowPos <= -5)
    {
        if(nowDirection == 1)
        {
            HAL_GPIO_WritePin(this->GPIOx, this->GPIO_Pin, GPIO_PIN_SET);
            this->PWMnum = this->targetPos - this->nowPos;
        }
        else if(nowDirection == 0)
        {
            HAL_GPIO_WritePin(this->GPIOx, this->GPIO_Pin, GPIO_PIN_RESET);
            this->PWMnum = this->nowPos - this->targetPos;
        }
        HAL_TIM_PWM_Start_IT(this->htim, TIM_CHANNEL_1);
                __HAL_TIM_SetCompare(this->htim, TIM_CHANNEL_1, 500);
    }
}

uint16_t Screw::readPos()
{
    return this->nowPos;
}

void Screw::OnHAL_TIM_PWM_PulseFinishedCallback()
{
    static uint16_t i = 0;
    i++;
//    this->nowPos++;
    if(i >= this->PWMnum)
    {
        i = 0;
        this->nowPos = this->targetPos;
        HAL_TIM_PWM_Stop(this->htim, TIM_CHANNEL_1);
    }
}
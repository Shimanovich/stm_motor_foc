#include "BLDCDriver3PWM.h"
#include <cmath>

BLDCDriver3PWM::BLDCDriver3PWM(TIM_HandleTypeDef* timA, uint32_t channelA,
                               TIM_HandleTypeDef* timB, uint32_t channelB,
                               TIM_HandleTypeDef* timC, uint32_t channelC)
    : _timA(timA), _timB(timB), _timC(timC),
      _channelA(channelA), _channelB(channelB), _channelC(channelC),
      _pwmPeriod(0)
{
}

int BLDCDriver3PWM::init()            // ← ИЗМЕНЕНО: возвращаем int
{
    if (_timA == nullptr) return 0;

    _pwmPeriod = _timA->Init.Period;

    HAL_TIM_PWM_Start(_timA, _channelA);
    HAL_TIM_PWM_Start(_timB, _channelB);
    HAL_TIM_PWM_Start(_timC, _channelC);

    enable();
    return 1;   // success (стандарт SimpleFOC)
}

void BLDCDriver3PWM::enable()
{
    // TC4452 на вашей плате всегда включены
}

void BLDCDriver3PWM::disable()
{
    // Можно оставить пустым или добавить остановку PWM при необходимости
}

void BLDCDriver3PWM::setPwm(float Ua, float Ub, float Uc)
{
    if (_pwmPeriod == 0) return;

    Ua = std::fmax(0.0f, std::fmin(1.0f, Ua));
    Ub = std::fmax(0.0f, std::fmin(1.0f, Ub));
    Uc = std::fmax(0.0f, std::fmin(1.0f, Uc));

    uint32_t pwmA = (uint32_t)(Ua * _pwmPeriod);
    uint32_t pwmB = (uint32_t)(Ub * _pwmPeriod);
    uint32_t pwmC = (uint32_t)(Uc * _pwmPeriod);

    __HAL_TIM_SET_COMPARE(_timA, _channelA, pwmA);
    __HAL_TIM_SET_COMPARE(_timB, _channelB, pwmB);
    __HAL_TIM_SET_COMPARE(_timC, _channelC, pwmC);
}

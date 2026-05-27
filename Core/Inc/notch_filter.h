#pragma once
#include <stdint.h>

typedef struct {
    float b0, b1, b2;   // коэффициенты числителя
    float a1, a2;       // коэффициенты знаменателя (a0 = 1)
    float x1, x2;       // предыдущие входы
    float y1, y2;       // предыдущие выходы
    float Ts;           // период дискретизации, секунды
} NotchFilter;

/* Инициализация фильтра */
void Notch_Init(NotchFilter* f, float fs, float f_notch, float Q);

/* Обновление фильтра (вызывать каждый цикл) */
float Notch_Update(NotchFilter* f, float x);

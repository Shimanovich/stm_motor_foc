#include "notch_filter.h"
#include <math.h>

void Notch_Init(NotchFilter* f, float fs, float f_notch, float Q)
{
    f->Ts = 1.0f / fs;

    float omega = 2.0f * (float)M_PI * f_notch / fs;
    float alpha = sinf(omega) / (2.0f * Q);

    float cos_omega = cosf(omega);

    f->b0 = 1.0f;
    f->b1 = -2.0f * cos_omega;
    f->b2 = 1.0f;

    f->a1 = -2.0f * cos_omega;           // a0 = 1, поэтому a1 и a2 без знака
    f->a2 = 1.0f - alpha;

    // Нормализация (делим все на (1 + alpha))
    float a0 = 1.0f + alpha;
    f->b0 /= a0;
    f->b1 /= a0;
    f->b2 /= a0;
    f->a1 /= a0;
    f->a2 /= a0;

    // Обнуляем историю
    f->x1 = f->x2 = 0.0f;
    f->y1 = f->y2 = 0.0f;
}

float Notch_Update(NotchFilter* f, float x)
{
    // y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
    float y = f->b0 * x + f->b1 * f->x1 + f->b2 * f->x2
              - f->a1 * f->y1 - f->a2 * f->y2;

    // Сдвигаем историю
    f->x2 = f->x1;
    f->x1 = x;
    f->y2 = f->y1;
    f->y1 = y;

    return y;
}

// fft.c
#include "fft.h"
#include <math.h>

void dc_removal(float *data, uint16_t n)
{
	float mean = 0.0;
    for (uint16_t i = 0; i < n; i++) mean += data[i];
    mean /= n;
    for (uint16_t i = 0; i < n; i++) data[i] -= mean;
}

void window_hamming(float *data, uint16_t n)
{
    for (uint16_t i = 0; i < n; i++) {
        data[i] *= 0.54 - 0.46 * cos(2.0 * M_PI * i / (n - 1));
    }
}

void fft(float *real, float *imag, uint16_t n)
{
    // Bit reversal
    uint16_t j = 0;
    for (uint16_t i = 0; i < n; i++) {
        if (i < j) {
            float temp = real[i]; real[i] = real[j]; real[j] = temp;
            temp = imag[i]; imag[i] = imag[j]; imag[j] = temp;
        }
        uint16_t m = n >> 1;
        while (m >= 2 && j >= m) { j -= m; m >>= 1; }
        j += m;
    }

    float theta;
    float wpr;
    float wr;
    float wpi;
    float tempr;
    float tempi;
    float wtemp;
    float wi;

    // Danielson-Lanczos
    for (uint16_t mmax = 2; mmax < n; mmax <<= 1) {
        theta = -2.0 * M_PI / mmax;
        wpr = cos(theta);
        wpi = sin(theta);
        wr = 1.0;
        wi = 0.0;
        for (uint16_t m = 0; m < mmax; m += 2) {
            for (uint16_t i = m; i < n; i += mmax) {
                uint16_t j = i + (mmax >> 1);
                tempr = wr * real[j] - wi * imag[j];
                tempi = wr * imag[j] + wi * real[j];
                real[j] = real[i] - tempr;
                imag[j] = imag[i] - tempi;
                real[i] += tempr;
                imag[i] += tempi;
            }
            wtemp = wr;
            wr = wr * wpr - wi * wpi;
            wi = wi * wpr + wtemp * wpi;
        }
    }
}

void complex_to_magnitude(float *real, uint16_t n)
{
    for (uint16_t i = 0; i < n; i++) {
        real[i] = sqrt(real[i] * real[i] + real[i + n] * real[i + n]); // imag хранится после real
    }
}

float find_peak_frequency(float *magnitude, float fs, uint16_t n)
{
	float max_mag = 0.0;
    uint16_t max_idx = 0;
    for (uint16_t i = 1; i < (n >> 1); i++) {
        if (magnitude[i] > max_mag) {
            max_mag = magnitude[i];
            max_idx = i;
        }
    }
    return max_idx * (fs / n);
}

// fft.h
#ifndef FFT_H
#define FFT_H

#include <stdint.h>

void dc_removal(float *data, uint16_t n);
void window_hamming(float *data, uint16_t n);
void fft(float *real, float *imag, uint16_t n);
void complex_to_magnitude(float *real, uint16_t n);
float find_peak_frequency(float *magnitude, float fs, uint16_t n);

#endif

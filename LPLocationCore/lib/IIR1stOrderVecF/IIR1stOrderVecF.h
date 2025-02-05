#pragma once
#include <Arduino.h>
#ifndef PI
#define PI 3.14159265358979323846
#endif

class IIR1stOrderVecF {
    public:
        // Constructor
        IIR1stOrderVecF(float alpha, size_t dim):alpha(alpha), dim(dim) {
            n_prev = new float[dim];
        }
        // Destructor
        ~IIR1stOrderVecF() {
            delete[] n_prev;
        }
        // Process
        void filter(float* n, float* out) {
            // First input
            if (first) {
                for (size_t i = 0; i < dim; i++) {
                    n_prev[i] = n[i];
                }
                first = false;
            }
            // Filter
            for (size_t i = 0; i < dim; i++) {
                n_prev[i] = alpha * n_prev[i] + (1 - alpha) * n[i];
            }
            // Copy to output
            for (size_t i = 0; i < dim; i++) {
                out[i] = n_prev[i];
            }
        }
        // Change alpha
        void setAlpha(float a) {
            alpha = a;
        }
        // Get cutoff frequency
        float getCFreq(float Ts) {
            return float(1)/(2*PI*alpha*Ts);
        }

    private:
        // Vector dimensions
        size_t dim;
        // Mixing alpha
        float alpha;
        // Previous value
        float* n_prev;
        // First input flag
        bool first = true;
};
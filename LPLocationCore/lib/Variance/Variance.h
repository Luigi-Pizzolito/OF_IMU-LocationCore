#pragma once
#include <Arduino.h>

class Variance {
    public:
        Variance(): _n(0), _mean(0), _M2(0)  {}

        void reset() {
            _mean = 0;
            _M2 = 0;
            _n = 0;
        }

        float update(float x) {
            // Welford's algorithm
            _n++;
            float delta = x - _mean;
            _mean += delta / _n;
            float delta2 = x - _mean;
            _M2 += delta * delta2;

            if (_n < 2) {
                return 0;
            } else {
                return _M2 / (_n - 1);
            }
        }

    private:
        size_t _n;
        float _mean;
        float _M2;
};
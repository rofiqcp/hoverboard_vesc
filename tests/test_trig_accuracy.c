#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "../Src/foc_motor.c"

int main(void)
{
    const double pi = 3.14159265358979323846;
    int max_error = 0;
    uint16_t worst_phase = 0U;
    uint64_t sum_error = 0U;

    for (uint32_t p = 0U; p < 65536U; ++p) {
        const int actual = trig_q14((uint16_t)p);
        const double radians = (2.0 * pi * (double)p) / 65536.0;
        const int reference = (int)llround(sin(radians) * 16384.0);
        int error = actual - reference;
        if (error < 0) error = -error;
        sum_error += (uint32_t)error;
        if (error > max_error) {
            max_error = error;
            worst_phase = (uint16_t)p;
        }
    }

    printf("TRIG_Q14_ACCURACY max_error=%d_lsb mean_abs_error=%.6f worst_phase=%u\n",
           max_error, (double)sum_error / 65536.0, worst_phase);
    return max_error <= 2 ? 0 : 1;
}

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

/* Include the production core so the static fixed-point SVM is tested directly. */
#include "../Src/foc_motor.c"

#define VESC_ONE_BY_SQRT3 0.57735026919f
#define VESC_TWO_BY_SQRT3 1.15470053838f

static int clamp_int(int value, int low, int high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static void vesc_svm_reference(float alpha, float beta, uint16_t period,
                               uint16_t *ta, uint16_t *tb, uint16_t *tc,
                               uint8_t *sector_out)
{
    uint8_t sector;
    if (beta >= 0.0f) {
        if (alpha >= 0.0f) {
            sector = (VESC_ONE_BY_SQRT3 * beta > alpha) ? 2U : 1U;
        } else {
            sector = (-VESC_ONE_BY_SQRT3 * beta > alpha) ? 3U : 2U;
        }
    } else {
        if (alpha >= 0.0f) {
            sector = (-VESC_ONE_BY_SQRT3 * beta > alpha) ? 5U : 6U;
        } else {
            sector = (VESC_ONE_BY_SQRT3 * beta > alpha) ? 4U : 5U;
        }
    }

    int a = 0, b = 0, c = 0;
    switch (sector) {
        case 1: {
            int t1 = (int)((alpha - VESC_ONE_BY_SQRT3 * beta) * period);
            int t2 = (int)((VESC_TWO_BY_SQRT3 * beta) * period);
            a = ((int)period + t1 + t2) / 2;
            b = a - t1;
            c = b - t2;
            break;
        }
        case 2: {
            int t2 = (int)((alpha + VESC_ONE_BY_SQRT3 * beta) * period);
            int t3 = (int)((-alpha + VESC_ONE_BY_SQRT3 * beta) * period);
            b = ((int)period + t2 + t3) / 2;
            a = b - t3;
            c = a - t2;
            break;
        }
        case 3: {
            int t3 = (int)((VESC_TWO_BY_SQRT3 * beta) * period);
            int t4 = (int)((-alpha - VESC_ONE_BY_SQRT3 * beta) * period);
            b = ((int)period + t3 + t4) / 2;
            c = b - t3;
            a = c - t4;
            break;
        }
        case 4: {
            int t4 = (int)((-alpha + VESC_ONE_BY_SQRT3 * beta) * period);
            int t5 = (int)((-VESC_TWO_BY_SQRT3 * beta) * period);
            c = ((int)period + t4 + t5) / 2;
            b = c - t5;
            a = b - t4;
            break;
        }
        case 5: {
            int t5 = (int)((-alpha - VESC_ONE_BY_SQRT3 * beta) * period);
            int t6 = (int)((alpha - VESC_ONE_BY_SQRT3 * beta) * period);
            c = ((int)period + t5 + t6) / 2;
            a = c - t5;
            b = a - t6;
            break;
        }
        default: {
            int t6 = (int)((-VESC_TWO_BY_SQRT3 * beta) * period);
            int t1 = (int)((alpha + VESC_ONE_BY_SQRT3 * beta) * period);
            a = ((int)period + t6 + t1) / 2;
            c = a - t1;
            b = c - t6;
            break;
        }
    }

    a = clamp_int(a, 0, period);
    b = clamp_int(b, 0, period);
    c = clamp_int(c, 0, period);
    *ta = (uint16_t)a;
    *tb = (uint16_t)b;
    *tc = (uint16_t)c;
    *sector_out = sector;
}

static unsigned abs_diff_u16(uint16_t a, uint16_t b)
{
    return a > b ? (unsigned)(a - b) : (unsigned)(b - a);
}

int main(void)
{
    const uint16_t period = 2000U;
    uint32_t rng = 0x4D595DF4U;
    unsigned tested = 0U;
    unsigned sector_mismatch = 0U;
    unsigned max_compare_error = 0U;
    uint64_t compare_error_sum = 0U;

    for (unsigned iteration = 0U; iteration < 300000U; ++iteration) {
        rng = rng * 1664525U + 1013904223U;
        int32_t alpha = (int32_t)((rng >> 16) & 0x7FFFU) - 16384;
        rng = rng * 1664525U + 1013904223U;
        int32_t beta = (int32_t)((rng >> 16) & 0x7FFFU) - 16384;

        /* Production current loop limits modulation to sqrt(3)/2. */
        if ((int64_t)alpha * alpha + (int64_t)beta * beta >
            (int64_t)MODULATION_MAX_Q14 * MODULATION_MAX_Q14) {
            continue;
        }

        uint16_t fa, fb, fc, ra, rb, rc;
        uint8_t fs, rs;
        foc_svm_q14((int16_t)alpha, (int16_t)beta, period, &fa, &fb, &fc, &fs);
        vesc_svm_reference((float)alpha / 16384.0f, (float)beta / 16384.0f,
                           period, &ra, &rb, &rc, &rs);

        unsigned da = abs_diff_u16(fa, ra);
        unsigned db = abs_diff_u16(fb, rb);
        unsigned dc = abs_diff_u16(fc, rc);
        unsigned local_max = da > db ? da : db;
        if (dc > local_max) local_max = dc;
        if (local_max > max_compare_error) max_compare_error = local_max;
        compare_error_sum += da + db + dc;
        if (fs != rs) ++sector_mismatch;
        ++tested;
    }

    const double mean_error = tested ?
        (double)compare_error_sum / (double)(tested * 3U) : 0.0;
    printf("SVM_VESC_COMPARE tested=%u max_compare_error=%u mean_abs_error=%.6f sector_mismatch=%u\n",
           tested, max_compare_error, mean_error, sector_mismatch);

    /* One timer count is the expected quantization ceiling for this Q14 port. */
    if (tested < 100000U || max_compare_error > 1U) return 1;
    return 0;
}

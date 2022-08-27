/* G.Dar extensions for Q31.1 2022 #no23 */


#ifndef _Q15_MATH
#define _Q15_MATH

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "../../src_mz/dsp_mz.h"

#define Q15_MAX (32767)
#define Q15_MIN (-32768)
#define Q31_MAX (2147483647)
#define Q31_MIN (-2147483648)

/* define the desired trigonometric resolution (higher bit values create larger tables)
 * note that, if all are undefined, the default will be the 8-bit table */
#undef SINE_TABLE_4BIT
#undef SINE_TABLE_5BIT
#undef SINE_TABLE_6BIT
#undef SINE_TABLE_7BIT
#undef SINE_TABLE_8BIT

typedef int16_t q15_t;
typedef uint16_t q16angle_t;
typedef int32_t q31_t;
typedef uint32_t q32angle_t;

double q15_to_dbl(q15_t num);
float q15_to_float(q15_t num);
int16_t q15_to_int(q15_t num);
q15_t q15_from_dbl(double num);
q15_t q15_from_float(float num);
q15_t q15_from_int(int num);
double q31_to_dbl(q31_t num);
float q31_to_float(q31_t num);
int32_t q31_to_int(q31_t num);
q31_t q31_from_dbl(double num);
q31_t q31_from_float(float num);
q31_t q31_from_int(int num);

q15_t q15_mul(q15_t multiplicand, q15_t multiplier);
#define Q15_MUL(multiplicand,multiplier) (((int32_t)multiplicand * (int32_t)multiplier) >> 16)
#define Q15_POW2(num) (((int32_t)num * (int32_t)num) >> 15)

q15_t q15_div(q15_t dividend, q15_t divisor);
q15_t q15_add(q15_t addend, q15_t adder);
#define Q15_ADD(addend,adder) ((uint32_t)addend + (uint32_t)adder)
q15_t q15_abs(q15_t num);
q15_t q15_sqrt(q15_t num);
q15_t q15_hypot(int16c n);
q31_t q31_mul(q31_t multiplicand, q31_t multiplier);
q31_t q31_div(q31_t dividend, q31_t divisor);
q31_t q31_add(q31_t addend, q31_t adder);
q31_t q31_abs(q31_t num);
q31_t q31_sqrt(q31_t num);
q31_t q31_hypot(int32c n);

q15_t q15_sin(q16angle_t theta);
q15_t q15_fast_sin(q16angle_t theta);
q15_t q15_cos(q16angle_t theta);
q15_t q15_fast_cos(q16angle_t theta);
q15_t q15_tan(q16angle_t theta);
q15_t q15_fast_tan(q16angle_t theta);
q31_t q31_sin(q32angle_t theta);
q31_t q31_fast_sin(q32angle_t theta);
q31_t q31_cos(q32angle_t theta);
q31_t q31_fast_cos(q32angle_t theta);
q31_t q31_tan(q32angle_t theta);
q31_t q31_fast_tan(q32angle_t theta);

q16angle_t q15_atan2(q15_t sine, q15_t cosine);
q32angle_t q31_atan2(q31_t sine, q31_t cosine);

/* TODO:
q16angle_t q15_acos(q15_t num);
q16angle_t q15_asin(q15_t num);
q16angle_t q15_atan(q15_t num);
q16angle_t q15_atan2(q15_t num);
q15_t q15_exp(q16angle_t theta);
... more? ...
*/

#ifdef __cplusplus
}
#endif

#endif


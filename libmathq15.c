/* G.Dar extension for Q31.1 2022 #no23 */

#include "libmathq15.h"

/***************** local defines *****************/

/***************** variable declarations *****************/
#if defined(SINE_TABLE_4BIT)
    const q15_t sine_table[] = {  0, 3211, 6392, 9511, 12539, 15446, 18204, 20787,
                            23169, 25329, 27244, 28897, 30272, 31356, 32137, 32609};
    const int SINE_TABLE_ENTRIES = 16;
    const int SINE_TABLE_SHIFT = 10;

#elif defined(SINE_TABLE_5BIT)

    const q15_t sine_table[] = {0, 1607, 3211, 4807, 6392, 7961, 9511, 11038,
                            12539, 14009, 15446, 16845, 18204, 19519, 20787, 22004,
                            23169, 24278, 25329, 26318, 27244, 28105, 28897, 29621,
                            30272, 30851, 31356, 31785, 32137, 32412, 32609, 32727};
    const int SINE_TABLE_ENTRIES = 32;
    const int SINE_TABLE_SHIFT = 9;

#elif defined(SINE_TABLE_6BIT)

    const q15_t sine_table[] = {  0, 804, 1607, 2410, 3211, 4011, 4807, 5601,
                            6392, 7179, 7961, 8739, 9511, 10278, 11038, 11792,
                            12539, 13278, 14009, 14732, 15446, 16150, 16845, 17530,
                            18204, 18867, 19519, 20159, 20787, 21402, 22004, 22594,
                            23169, 23731, 24278, 24811, 25329, 25831, 26318, 26789,
                            27244, 27683, 28105, 28510, 28897, 29268, 29621, 29955,
                            30272, 30571, 30851, 31113, 31356, 31580, 31785, 31970,
                            32137, 32284, 32412, 32520, 32609, 32678, 32727, 32757};
    const int SINE_TABLE_ENTRIES = 64;
    const int SINE_TABLE_SHIFT = 8;

#elif defined(SINE_TABLE_7BIT)

    const q15_t sine_table[] = {0, 402, 804, 1206, 1607, 2009, 2410, 2811,
                            3211, 3611, 4011, 4409, 4807, 5205, 5601, 5997,
                            6392, 6786, 7179, 7571, 7961, 8351, 8739, 9126,
                            9511, 9895, 10278, 10659, 11038, 11416, 11792, 12166,
                            12539, 12909, 13278, 13645, 14009, 14372, 14732, 15090,
                            15446, 15799, 16150, 16499, 16845, 17189, 17530, 17868,
                            18204, 18537, 18867, 19194, 19519, 19840, 20159, 20474,
                            20787, 21096, 21402, 21705, 22004, 22301, 22594, 22883,
                            23169, 23452, 23731, 24006, 24278, 24546, 24811, 25072,
                            25329, 25582, 25831, 26077, 26318, 26556, 26789, 27019,
                            27244, 27466, 27683, 27896, 28105, 28309, 28510, 28706,
                            28897, 29085, 29268, 29446, 29621, 29790, 29955, 30116,
                            30272, 30424, 30571, 30713, 30851, 30984, 31113, 31236,
                            31356, 31470, 31580, 31684, 31785, 31880, 31970, 32056,
                            32137, 32213, 32284, 32350, 32412, 32468, 32520, 32567,
                            32609, 32646, 32678, 32705, 32727, 32744, 32757, 32764};
    const int SINE_TABLE_ENTRIES = 128;
    const int SINE_TABLE_SHIFT = 7;

#else

    const q15_t sine_table[] = {    0, 201, 402, 603, 804, 1005, 1206, 1406,
                            1607, 1808, 2009, 2209, 2410, 2610, 2811, 3011,
                            3211, 3411, 3611, 3811, 4011, 4210, 4409, 4608,
                            4807, 5006, 5205, 5403, 5601, 5799, 5997, 6195,
                            6392, 6589, 6786, 6982, 7179, 7375, 7571, 7766,
                            7961, 8156, 8351, 8545, 8739, 8932, 9126, 9319,
                            9511, 9703, 9895, 10087, 10278, 10469, 10659, 10849,
                            11038, 11227, 11416, 11604, 11792, 11980, 12166, 12353,
                            12539, 12724, 12909, 13094, 13278, 13462, 13645, 13827,
                            14009, 14191, 14372, 14552, 14732, 14911, 15090, 15268,
                            15446, 15623, 15799, 15975, 16150, 16325, 16499, 16672,
                            16845, 17017, 17189, 17360, 17530, 17699, 17868, 18036,
                            18204, 18371, 18537, 18702, 18867, 19031, 19194, 19357,
                            19519, 19680, 19840, 20000, 20159, 20317, 20474, 20631,
                            20787, 20942, 21096, 21249, 21402, 21554, 21705, 21855,
                            22004, 22153, 22301, 22448, 22594, 22739, 22883, 23027,
                            23169, 23311, 23452, 23592, 23731, 23869, 24006, 24143,
                            24278, 24413, 24546, 24679, 24811, 24942, 25072, 25201,
                            25329, 25456, 25582, 25707, 25831, 25954, 26077, 26198,
                            26318, 26437, 26556, 26673, 26789, 26905, 27019, 27132,
                            27244, 27355, 27466, 27575, 27683, 27790, 27896, 28001,
                            28105, 28208, 28309, 28410, 28510, 28608, 28706, 28802,
                            28897, 28992, 29085, 29177, 29268, 29358, 29446, 29534,
                            29621, 29706, 29790, 29873, 29955, 30036, 30116, 30195,
                            30272, 30349, 30424, 30498, 30571, 30643, 30713, 30783,
                            30851, 30918, 30984, 31049, 31113, 31175, 31236, 31297,
                            31356, 31413, 31470, 31525, 31580, 31633, 31684, 31735,
                            31785, 31833, 31880, 31926, 31970, 32014, 32056, 32097,
                            32137, 32176, 32213, 32249, 32284, 32318, 32350, 32382,
                            32412, 32441, 32468, 32495, 32520, 32544, 32567, 32588,
                            32609, 32628, 32646, 32662, 32678, 32692, 32705, 32717,
                            32727, 32736, 32744, 32751, 32757, 32761, 32764, 32766};
    const int SINE_TABLE_ENTRIES = 256;
    const int SINE_TABLE_SHIFT = 6;


#endif

const q16angle_t NINETY_DEG = 16384;
const q16angle_t ONE_EIGHTY_DEG = 32768;
const q16angle_t TWO_SEVENTY_DEG = 49152;
const q32angle_t NINETY_DEG_32 = 1073741824ULL;
const q32angle_t ONE_EIGHTY_DEG_32 = 2147483648ULL;
const q32angle_t TWO_SEVENTY_DEG_32 = 3221225472ULL;

/***************** local function declarations *****************/
q15_t q15_sin90(q16angle_t theta);
q15_t q15_fast_sin90(q16angle_t theta);
q31_t q31_sin90(q32angle_t theta);
q31_t q31_fast_sin90(q32angle_t theta);

/***************** function implementations *****************/
double q15_to_dbl(q15_t num) {
  return ((double)num)/((double)32768.0);
  }

float q15_to_float(q15_t num) {
  return ((float)num)/((float)32768.0);
  }

int16_t q15_to_int(q15_t num) {
  int16_t value = 0;

  if(num > 16383)
    value = 1;
  else if(num < -16383)
    value = -1;

  return value;
  }

q15_t q15_from_dbl(double num) {
  q15_t value;

  if(num > 0.99997)
    value = Q15_MAX;
  else if(num < -1.0)
    value = Q15_MIN;
  else {
    value = (q15_t)(num * 32768.0);
    }

  return value;
  }

q15_t q15_from_float(float num) {
  q15_t value;

  if(num > 0.99997)
    value = Q15_MAX;
  else if(num < -1.0)
    value = Q15_MIN;
  else {
    value = (q15_t)(num * 32768.0);
    }

  return value;
  }

q15_t q15_from_int(int num) {
  q15_t value = 0;

  if(num > 0)
    value = Q15_MAX;
  else if(num < 0)
    value = Q15_MIN;

  return value;
  }

q15_t /*__attribute__((always_inline))*/ q15_mul(q15_t multiplicand, q15_t multiplier) {
  int32_t product = ((int32_t)multiplicand * (int32_t)multiplier) >> 15;
  return (q15_t)product;
  }

q15_t q15_div(q15_t dividend, q15_t divisor) {
  q15_t quotient;

  /* check to ensure dividend is smaller in magnitude than the divisor */
  if((q15_abs(divisor) < q15_abs(dividend)) || (divisor == 0)) {
    /* saturation: if signs are different, then saturate negative */
    if((divisor & 0x8000) ^ (dividend & 0x8000)) {
	    quotient = Q15_MIN;
      }
    else {
	    quotient = Q15_MAX;
      }
    }
  else {
    quotient = 32768 * dividend/divisor;
    }

  return quotient;
  }

q15_t q15_add(q15_t addend, q15_t adder) {
  int32_t result = (uint32_t)addend + (uint32_t)adder;

  if(result > Q15_MAX)          
    result = Q15_MAX;
  else if(result < Q15_MIN)    
    result = Q15_MIN;

  return (q15_t)result;
  }

q15_t q15_abs(q15_t num) {


  if(num < 0) {
    if(num < -Q15_MAX)  
      return Q15_MAX;
    else                
      return -num;
    }
  else
    return num;
  }

q15_t q15_sqrt(q15_t num) {
    q15_t value;
    
    if(num < 0) {
        value = -1;         // invalid
      }
    else {
        value = 16383;
        q15_t increment = 8192;

        while(increment > 0) {
            q15_t valueSquared = q15_mul(value, value);
            if(valueSquared > num) {
                value -= increment;
              } 
            else {
                value += increment;
            }

          increment >>= 1;
        }
    }

  return value;
  }

q15_t q15_hypot(int16c n) {
  int32_t value;
  
  value=q15_mul(n.re,n.re)+q15_mul(n.im,n.im);  //
  
  return q15_sqrt(value);
  }

int8_t q15_log2(q15_t n) {
  int i,j;
  
  if(n<=0)
    return 0;
/*  j=0x4000;
  i=15;
  while(!(n & j))
    i--;*/
  i=0;
  while(n >>= 1) 
    ++i;
  return i;
  }

q15_t q15_sin(q16angle_t theta) {
    q15_t value;

    if(theta < ONE_EIGHTY_DEG){
        if(theta < NINETY_DEG) {
            /* for the first 89.9 deg, use the sin90 function directly  */
            value = q15_sin90(theta);
        } 
        else {
            /* for 90 deg through 179.99, 'mirror' the 90 degree calculation */
            uint16_t tempTheta = ONE_EIGHTY_DEG - theta - 1;
            value = q15_sin90((q16angle_t)tempTheta);
        }
      }
    else {
        if(theta < TWO_SEVENTY_DEG) {
            /* for 180 through 269.9, negative of the 90 degree calculation */
            uint16_t offset = (uint16_t)theta - ONE_EIGHTY_DEG;
            value = -q15_sin90((q16angle_t)offset);
        } 
        else {
            /* for 270 through 65535.9, negative of the mirror of the 90 degree calculation */
            uint16_t tempTheta = 0 - theta - 1;
            value = -q15_sin90((q16angle_t)tempTheta);
        }
      }

    return value;
  }

/* a helper function for the sin that only works between 0 and 89.99 degrees (0 to 16383) */
q15_t q15_sin90(q16angle_t theta){
    uint16_t value;

    if(theta < 16384) {
        q15_t tempTheta0, tempTheta1, table_value0, table_value1;

        /* look up the 4-bit values surrounding theta and store in table_value0 and table_value1 */
        tempTheta0 = theta >> SINE_TABLE_SHIFT;
        table_value0 = sine_table[tempTheta0];

        tempTheta1 = tempTheta0 + 1;

        if(tempTheta1 == SINE_TABLE_ENTRIES) {
            table_value1 = Q15_MAX;   // 90 degree value
          }
        else {
            table_value1 = sine_table[tempTheta1];
          }

        /* use the mask to get the low-order bits and store in tempTheta*/
        /* tempTheta will be very small, so it is appropriate to perform the division */
        q15_t domain = 1 << SINE_TABLE_SHIFT;
        q15_t percent = q15_div(theta - (tempTheta0 << SINE_TABLE_SHIFT), domain);
        q15_t offset = q15_mul(percent, (table_value1 - table_value0));

        value = offset + table_value0;
      } 
    else {
        value = Q15_MAX;
    }

  return value;
  }

q15_t q15_fast_sin(q16angle_t theta) {
  q15_t value;

  if(theta < ONE_EIGHTY_DEG) {
      if(theta < NINETY_DEG) {
          /* for the first 89.9 deg, use the sin90 function directly  */
          value = q15_fast_sin90(theta);
        }
      else {
          /* for 90 deg through 179.99, 'mirror' the 90 degree calculation */
          uint16_t tempTheta = ONE_EIGHTY_DEG - theta - 1;
          value = q15_fast_sin90((q16angle_t)tempTheta);
      }
    }
  else {
    if(theta < TWO_SEVENTY_DEG) {
        /* for 180 through 269.9, negative of the 90 degree calculation */
        uint16_t offset = (uint16_t)theta - ONE_EIGHTY_DEG;
        value = -q15_fast_sin90((q16angle_t)offset);
      }
    else {
      /* for 270 through 65535.9, negative of the mirror of the 90 degree calculation */
      uint16_t tempTheta = 0 - theta - 1;
      value = -q15_fast_sin90((q16angle_t)tempTheta);
      }
    }

  return value;
  }

/* a helper function for the sin that only works between 0 and 89.99 degrees (0 to 16383)
 *  the fast version only does the lookup and does not interpolate */
q15_t q15_fast_sin90(q16angle_t theta) {
  q15_t tempTheta;

  /* look up the 4-bit values surrounding theta and store in table_value0 and table_value1 */
  tempTheta = theta >> SINE_TABLE_SHIFT;
  return sine_table[tempTheta];
  }

/* by piggybacking off of the sine calculation, the cosine calculation is slightly slower than
 * it might be if it had its own lookup table but it would take twice memory */
q15_t q15_cos(q16angle_t theta) {
  q16angle_t tempTheta = theta + (q16angle_t)NINETY_DEG;
  q15_t value = q15_sin(tempTheta);
  return value;
  }

q15_t q15_fast_cos(q16angle_t theta) {
  q16angle_t tempTheta = theta + (q16angle_t)NINETY_DEG;
  q15_t value = q15_fast_sin(tempTheta);
  return value;
  }

/* since q15_t can only represent numbers between -1.0 and +0.99997, this may be a good time to
 * either use a fixed-point format with a higher range or a a floating-point format */
q15_t q15_tan(q16angle_t theta) {
    q15_t sinValue = q15_sin(theta);
    q15_t cosValue = q15_cos(theta);

  /* tan(theta) = sin(theta)/cos(theta) BUT we can only
   * represent values between -1.0 through +0.99997*/
  q15_t tanValue;

  if(q15_abs(sinValue) >= q15_abs(cosValue)) {
    if((sinValue & 0x8000) ^ (cosValue & 0x8000)) {
      tanValue = Q15_MIN;
      }
    else {
      tanValue = Q15_MAX;
      }
    }
  else {
    tanValue = q15_div(sinValue, cosValue);
    }

  return tanValue;
  }

q15_t q15_fast_tan(q16angle_t theta) {
    q15_t sinValue = q15_fast_sin(theta);
    q15_t cosValue = q15_fast_cos(theta);

    /* tan(theta) = sin(theta)/cos(theta) BUT we can only
     * represent values between -1.0 through +0.99997*/
    q15_t tanValue;

    if(q15_abs(sinValue) >= q15_abs(cosValue)){
        if((sinValue & 0x8000) ^ (cosValue & 0x8000)){
            tanValue = Q15_MIN;
          }
        else {
            tanValue = Q15_MAX;
        }
      }
    else {
        tanValue = q15_div(sinValue, cosValue);
    }

  return tanValue;
  }

/**
 Primary implementation taken from https://geekshavefeelings.com/posts/fixed-point-atan2

 Modified to take advantage of normal absolute values instead of negative absolute values,
 replaced calculated constants with exact values to reduce load and ensure proper compilation
 on simple compilers.
 */
uint16_t q15_atan2(int16_t sine, int16_t cosine) {
  int16_t abs_y, abs_x;

  if(cosine == sine) { // x/y or y/x would return -1 since 1 isn't representable
    if(sine > 0) { // 1/8
      return 8192;
      } 
    else if(sine < 0) { // 5/8
      return 40960;
      } 
    else { // x = y = 0
      return 0;
      }
    }

  abs_y = q15_abs(sine);
  abs_x = q15_abs(cosine);

  if(abs_x > abs_y) {    // octants 1, 4, 5, 8
    int16_t y_over_x = q15_div(sine, cosine);

    int16_t correction = q15_mul(2847, q15_abs(y_over_x));
    int16_t unrotated = q15_mul(11039 - correction, y_over_x);

    if(cosine > 0) {   // octants 1, 8
      return unrotated;
      } 
    else {            // octants 4, 5
      return 32768 + unrotated;
      }
    } 
  else {                // octants 2, 3, 6, 7
    int16_t x_over_y = q15_div(cosine, sine);

    int16_t correction = q15_mul(2847, q15_abs(x_over_y));
    int16_t unrotated = q15_mul(11039 - correction, x_over_y);

    if(sine > 0) {     // octants 2, 3
      return 16384 - unrotated;
      } 
    else {            // octants 6, 7
      return 49152 - unrotated;
      }
    }
	}


// ----------------------------------------------------------------------------
// Q31.1 by G.Dar 2022
double q31_to_dbl(q31_t num) {
  return ((double)num)/((double)2147483648.0);
  }

float q31_to_float(q31_t num) {
  return ((float)num)/((float)2147483648.0);
  }

int32_t q31_to_int(q31_t num) {
  int32_t value = 0;

  if(num > 1073676288ULL)
    value = 1;
  else if(num < -1073676288ULL)
    value = -1;

  return value;
  }

q31_t q31_from_dbl(double num) {
  q31_t value;

  if(num > 0.99997)
    value = 2147483647ULL;
  else if(num < -1.0)
    value = -2147483648LL;
  else {
    value = (q31_t)(num * 2147483648.0);
    }

  return value;
  }

q31_t q31_from_float(float num) {
  q31_t value;

  if(num > 0.99997)
    value = 2147483647ULL;
  else if(num < -1.0)
    value = -2147483648LL;
  else {
    value = (q31_t)(num * 2147483648.0);
    }

  return value;
  }

q31_t q31_from_int(int num) {
  q31_t value = 0;

  if(num > 0)
    value = 2147483647ULL;
  else if(num < 0)
    value = -2147483648LL;

  return value;
  }

q31_t q31_mul(q31_t multiplicand, q31_t multiplier) {
//  int64_t product = ((int64_t)multiplicand * (int64_t)multiplier) >> 31;
  int32_t product = (multiplicand >> 16) * (multiplier >> 16);    // ? :2 ... sistemare?
  return (q31_t)product;
  }

q31_t q31_div(q31_t dividend, q31_t divisor) {
  q31_t quotient;

  /* check to ensure dividend is smaller in magnitude
   * than the divisor */
  if((q31_abs(divisor) < q31_abs(dividend)) || (divisor == 0)) {
      /* saturation: if signs are different,
       * then saturate negative */
    if((divisor & 0x80000000) ^ (dividend & 0x80000000)) {
	    quotient = -2147483648LL;
      }
    else {
	    quotient = 2147483647ULL;
      }
    }
  else {
    quotient = 2147483648ULL * dividend/divisor;
    }

  return quotient;
  }

q31_t q31_add(q31_t addend, q31_t adder) {
  int64_t result = (uint64_t)addend + (uint64_t)adder;

  if(result > 2147483647ULL)
    result = 2147483647ULL;
  else if(result < -2147483648LL)
    result = -2147483648LL;

  return (q31_t)result;
  }

q31_t q31_abs(q31_t num) {
  q31_t value = num;

  if(value < 0){
    if(value < -2147483647LL)  
      value = 2147483647ULL;
    else                
      value = -value;
    }

  return value;
  }

q31_t q31_sqrt(q31_t num) {
  q31_t value;

  if(num < 0) {
    value = -1;         // invalid
    }
  else {
    value = 1073676288ULL;
    q31_t increment = 536870912ULL;

    while(increment > 0) {
      q31_t valueSquared = q31_mul(value, value);
      if(valueSquared > num) {
        value -= increment;
        } 
      else {
        value += increment;
        }
      increment >>= 1;
      }
    }

  return value;
  }

q31_t q31_hypot(int32c n) {
  int64_t value;
  
  value=q31_mul(n.re,n.re)+q31_mul(n.im,n.im);  //
  
  return q31_sqrt(value);
  }

int8_t q31_log2(q31_t n) {
  int i,j;
  
  if(n<=0)
    return 0;
/*  j=0x4000;
  i=15;
  while(!(n & j))
    i--;*/
  i=0;
  while(n >>= 1) 
    ++i;
  // opp. https://stackoverflow.com/questions/11376288/fast-computing-of-log2-for-64-bit-integers
  return i;
  }

q31_t q31_sin(q32angle_t theta) {
    q31_t value;

    if(theta < ONE_EIGHTY_DEG_32) {
        if(theta < NINETY_DEG_32) {
            /* for the first 89.9 deg, use the sin90 function directly  */
            value = q31_sin90(theta);
        } 
        else {
            /* for 90 deg through 179.99, 'mirror' the 90 degree calculation */
            uint32_t tempTheta = ONE_EIGHTY_DEG_32 - theta - 1;
            value = q31_sin90((q32angle_t)tempTheta);
        }
      }
    else {
      if(theta < TWO_SEVENTY_DEG_32) {
        /* for 180 through 269.9, negative of the 90 degree calculation */
        uint32_t offset = (uint32_t)theta - ONE_EIGHTY_DEG_32;
        value = -q31_sin90((q32angle_t)offset);
        } 
      else {
        /* for 270 through 65535.9, negative of the mirror of the 90 degree calculation */
        uint32_t tempTheta = 0 - theta - 1;
        value = -q31_sin90((q32angle_t)tempTheta);
        }
      }

    return value;
  }

/* a helper function for the sin that only works between 0 and 89.99 degrees (0 to 16383) */
q31_t q31_sin90(q32angle_t theta){
  uint32_t value;

  if(theta < 1073741824) {
      q31_t tempTheta0, tempTheta1, table_value0, table_value1;

      /* look up the 4-bit values surrounding theta and store in table_value0 and table_value1 */
      tempTheta0 = theta >> SINE_TABLE_SHIFT;
      table_value0 = sine_table[tempTheta0];

      tempTheta1 = tempTheta0 + 1;

      if(tempTheta1 == SINE_TABLE_ENTRIES) {
          table_value1 = 2147483647ULL;   // 90 degree value
        }
      else {
          table_value1 = sine_table[tempTheta1];
        }

      /* use the mask to get the low-order bits and store in tempTheta*/
      /* tempTheta will be very small, so it is appropriate to perform the division */
      q31_t domain = 1 << SINE_TABLE_SHIFT;
      q31_t percent = q31_div(theta - (tempTheta0 << SINE_TABLE_SHIFT), domain);
      q31_t offset = q31_mul(percent, (table_value1 - table_value0));

      value = offset + table_value0;
    } 
  else {
    value = 2147483647ULL;
    }

  return value;
  }

q31_t q31_fast_sin(q32angle_t theta) {
    q31_t value;

    if(theta < ONE_EIGHTY_DEG_32) {
        if(theta < NINETY_DEG_32) {
            /* for the first 89.9 deg, use the sin90 function directly  */
            value = q31_fast_sin90(theta);
          }
        else {
            /* for 90 deg through 179.99, 'mirror' the 90 degree calculation */
            uint16_t tempTheta = ONE_EIGHTY_DEG_32 - theta - 1;
            value = q31_fast_sin90((q32angle_t)tempTheta);
        }
      }
    else {
      if(theta < TWO_SEVENTY_DEG) {
        /* for 180 through 269.9, negative of the 90 degree calculation */
        uint32_t offset = (uint32_t)theta - ONE_EIGHTY_DEG_32;
        value = -q31_fast_sin90((q32angle_t)offset);
        }
      else {
        /* for 270 through 65535.9, negative of the mirror of the 90 degree calculation */
        uint32_t tempTheta = 0 - theta - 1;
        value = -q31_fast_sin90((q32angle_t)tempTheta);
        }
    }

  return value;
  }

/* a helper function for the sin that only works between 0 and 89.99 degrees (0 to 16383)
 *  the fast version only does the lookup and does not interpolate */
q31_t q31_fast_sin90(q32angle_t theta) {
  q31_t tempTheta;

  /* look up the 4-bit values surrounding theta and store in table_value0 and table_value1 */
  tempTheta = theta >> SINE_TABLE_SHIFT;
  return sine_table[tempTheta];
  }

/* by piggybacking off of the sine calculation, the cosine calculation is slightly slower than
 * it might be if it had its own lookup table but it would take twice memory */
q31_t q31_cos(q32angle_t theta) {
  q32angle_t tempTheta = theta + (q32angle_t)NINETY_DEG_32;
  q31_t value = q31_sin(tempTheta);
  return value;
  }

q31_t q31_fast_cos(q32angle_t theta) {
  q32angle_t tempTheta = theta + (q32angle_t)NINETY_DEG_32;
  q31_t value = q31_fast_sin(tempTheta);
  return value;
  }

/* since q15_t can only represent numbers between -1.0 and +0.99997, this may be a good time to
 * either use a fixed-point format with a higher range or a a floating-point format */
q31_t q31_tan(q32angle_t theta) {
  q31_t sinValue = q31_sin(theta);
  q31_t cosValue = q31_cos(theta);

  /* tan(theta) = sin(theta)/cos(theta) BUT we can only
   * represent values between -1.0 through +0.99997*/
  q31_t tanValue;

  if(q31_abs(sinValue) >= q31_abs(cosValue)) {
    if((sinValue & 0x80000000) ^ (cosValue & 0x80000000)) {
      tanValue = -2147483648LL;
      }
    else {
      tanValue = 2147483647ULL;
      }
    }
  else {
    tanValue = q31_div(sinValue, cosValue);
    }

  return tanValue;
  }

q31_t q31_fast_tan(q32angle_t theta) {
  q31_t sinValue = q31_fast_sin(theta);
  q31_t cosValue = q31_fast_cos(theta);

  /* tan(theta) = sin(theta)/cos(theta) BUT we can only
   * represent values between -1.0 through +0.99997*/
  q31_t tanValue;

  if(q31_abs(sinValue) >= q31_abs(cosValue)){
    if((sinValue & 0x80000000) ^ (cosValue & 0x80000000)){
      tanValue = -2147483648LL;
      }
    else {
      tanValue = 2147483647ULL;
      }
    }
  else {
    tanValue = q31_div(sinValue, cosValue);
    }

  return tanValue;
  }

/**
 Primary implementation taken from https://geekshavefeelings.com/posts/fixed-point-atan2

 Modified to take advantage of normal absolute values instead of negative absolute values,
 replaced calculated constants with exact values to reduce load and ensure proper compilation
 on simple compilers.
 */
uint32_t q31_atan2(int32_t sine, int32_t cosine) {
  int32_t abs_y, abs_x;

  if(cosine == sine) { // x/y or y/x would return -1 since 1 isn't representable
    if(sine > 0) { // 1/8
      return 536870912ULL;
      } 
    else if(sine < 0) { // 5/8
      return 2684354560ULL;
      } 
    else { // x = y = 0
      return 0;
      }
    }

  abs_y = q31_abs(sine);
  abs_x = q31_abs(cosine);

    if(abs_x > abs_y) {    // octants 1, 4, 5, 8
      int32_t y_over_x = q31_div(sine, cosine);

      int32_t correction = q31_mul(186580992ULL, q31_abs(y_over_x));
      int32_t unrotated = q31_mul(723451904ULL - correction, y_over_x);

      if(cosine > 0) {   // octants 1, 8
        return unrotated;
        } 
      else {            // octants 4, 5
        return 2147483648ULL + unrotated;
        }
      } 
    else {                // octants 2, 3, 6, 7
      int32_t x_over_y = q31_div(cosine, sine);

      int32_t correction = q31_mul(186580992ULL, q31_abs(x_over_y));
      int32_t unrotated = q31_mul(723451904ULL - correction, x_over_y);

      if(sine > 0) {     // octants 2, 3
        return 1073741824ULL - unrotated;
        } 
      else {            // octants 6, 7
        return 3221225472ULL - unrotated;
      }
    }
	}


/******************************************************************************
 * Filename: qfixed.h
 * Description: This file contains helper functions for fixed point math using
 *              Q16 scaling.
 *
 * The Q16 data format is a signed 32bit integer. The first bit is the sign, the
 * next 15 bits are the mantissa, and the last 16 bits are the fraction.
 * [SMMMMMMMMMMMMMMM.FFFFFFFFFFFFFFFF],
 * where S is sign, M is mantissa, and F is fraction bits
 *
 * The largest possible value (0x7FFF FFFF) is 32768 - 1^(-16), or
 * 32767.999984741
 * The smallest non-negative (0x0000 0001) is 1^(-16), or
 * 0.00001525879
 * The smallest negative number (0x8000 0000) is -32768
 *
 * For multiplication of fixed point numbers, the result with have twice the
 * number of fractional bits (m*2^-16 * n*2^-16 = m*n*2^-32).
 * The way to deal with this is a right-shift after the multiply.
 * Likewise, for division, the final result will have ZERO fractional bits!
 * Left-shifting before dividing fixes this issue.
 ******************************************************************************

Copyright (c) 2019 David Miller

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
 */

#ifndef QFIXED_H_
#define QFIXED_H_

// Some helpful constants in Q16
#define Q16_EXP_BITS    16

#define Q8_UNIT         (uint32_t)256
#define Q12_UNIT        (uint32_t)4096
#define Q16_UNIT        (uint32_t)65536

#define Q16_3P3         (uint32_t)216269 // Used for 3.3V calculations
#define Q16_30P0        (uint32_t)1966080 // Used for temperature calcs (30degC)

typedef int32_t Q16_t; // 1-bit sign, 15-bits mantissa, 16-bits exponent
                       // [-32768.000 to 32767.9999...] resolution .00001526
typedef uint32_t UQ16_t; // 16-bits mantissa, 16-bits exponent
                         // [0 to 65535.999...] resolution .00001526

inline Q16_t F2Q16(float inval) {
  float retval = inval*0x10000;
  if(retval >= 0) {
    retval += 0.5f;
  } else {
    retval -= 0.5f;
  }
  return (int32_t)retval;
}

inline Q16_t Q16_MUL(Q16_t a, Q16_t b) {
  int64_t tempval = (int64_t)a;
  tempval = tempval * b;
  tempval = tempval >> Q16_EXP_BITS;
  return (Q16_t)tempval;
}

inline Q16_t Q16_DIV(Q16_t a, Q16_t b) {
  int64_t tempval = (int64_t)a;
  tempval = tempval << Q16_EXP_BITS;
  tempval = tempval / b;
  return (Q16_t)tempval;
}

#endif /* QFIXED_H_ */

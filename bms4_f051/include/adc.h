/******************************************************************************
 * Filename: adc.h
 * Description: Header for adc.c
 *              This file also contains some definitions for the ADC hardware.
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

#ifndef ADC_H_
#define ADC_H_

// Calibration data locations in system memory
#define ADC_CALDATA_TS30  (uint32_t)0x1FFFF7B8
#define ADC_CALDATA_TS110 (uint32_t)0x1FFFF7C2
#define ADC_CALDATA_VREF  (uint32_t)0x1FFFF7BA

#define ADC_NUM_CHANNELS  6
#define ADC_CH_V4         0
#define ADC_CH_V3         1
#define ADC_CH_V2         4
#define ADC_CH_V1         5
#define ADC_CH_TS         16
#define ADC_CH_VREF       17

#define ADC_NUMBER_OF_CONV_TO_AVG   16 // 4 more bits, brings to 16-bit result
#define ADC_OVERSAMPLING_BITS       4

#define ADC_DIFFAMP_SCALE_FACTOR    5

void ADC_Init(void);
void ADC_Conversion_Complete_Handler(void);
void ADC_Transfer_Error_Handler(void);


#endif /* ADC_H_ */

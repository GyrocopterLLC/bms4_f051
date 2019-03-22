/*
 * adc.h
 *
 *  Created on: Mar 20, 2019
 *      Author: David
 */

#ifndef ADC_H_
#define ADC_H_

#define ADC_NUM_CHANNELS  6
#define ADC_CH_V4         0
#define ADC_CH_V3         1
#define ADC_CH_V2         4
#define ADC_CH_V1         5
#define ADC_CH_TS         16
#define ADC_CH_VREF       17

#define ADC_NUMBER_OF_CONV_TO_AVG   256 // 8 more bits

void ADC_Init(void);
void ADC_Conversion_Complete_Handler(void);
void ADC_Transfer_Error_Handler(void);


#endif /* ADC_H_ */

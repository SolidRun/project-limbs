#ifndef __CONFIG_H
#define __CONFIG_H

/*
adjust these to suit the application
*/
#define NUM_OF_CDC_UARTS                    3
# define ADC_ENABLE 1
#define ADC_IT_MODE 0
/* calculate voltage ADC factore
  Vin = (ADC_RESOLUTION * ADC_Value[12Bit]) * (R1+R2/R2)
  Vin = [Vref/(2^12) * (R2+R1/R2)] * ADC_Value
  Factore = 3.3 / 4096 *(150/1k+150) = 0.0061767578125
  ADC_RESOLUTION = 0.0008056640625
*/
#define VOLTAGE_ADC_FACTORE 0.00645
/* calculate current ADC factore
  TO DO
  .
  .
  .
  ADC_RESOLUTION = 0.0008056640625
*/
#define CURRENT_ADC_FACTORE 0.01

#define USART2_VCP_REPRINT_ENABLE 0

# define SPI_ENABLE 1

#endif /* __CONFIG_H */

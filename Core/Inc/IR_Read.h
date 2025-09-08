/**
 *
 *
 */

#pragma once

#include "stm32f4xx.h"


void f4_adc1_init(uint16_t *rxdata)
{
RCC->APB1ENR |= (RCC_AHB1ENR_GPIOAEN
 | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN);

GPIOA->MODER |= (GPIO_MODER_MODE0 | GPIO_MODER_MODE1);
GPIOB->MODER |= GPIO_MODER_MODE0;
GPIOC->MODER |= GPIO_MODER_MODE5;

RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

// Reg channels and order must be selected in ADC_SQRx
// Total conversions must be written in ADC_SQR1 reg in L[3:0] bits
// Conversions per channel in ADC_SQRx
ADC1->SQR1 &= 0x0;
ADC1->SQR2 &= 0x0;
ADC1->SQR3 &= 0x0;
ADC1->SQR1 |= (0x5 << ADC_SQR1_L_Pos);

// in0 pa0, in1 pa1, in9 pb1, in8 pb0, in15 pc5
ADC1->SQR3 |= (0x1 << ADC_SQR3_SQ1_Pos);
ADC1->SQR3 |= (0x1 << ADC_SQR3_SQ2_Pos);
ADC1->SQR2 |= (0x1 << ADC_SQR2_SQ8_Pos);
ADC1->SQR2 |= (0x1 << ADC_SQR2_SQ9_Pos);
ADC1->SQR1 |= (0x1 << ADC_SQR1_SQ15_Pos);

// Set slowest sample rate for all (0b111)
ADC1->SMPR2 |= (ADC_SMPR2_SMP1 | ADC_SMPR2_SMP2
| ADC_SMPR2_SMP8 | ADC_SMPR2_SMP9);
ADC1->SMPR1 |= ADC_SMPR1_SMP15;

// ADC_CR1 RES (resolution = default 12 bits)\

// ADC_CR1 SCAN = 1 to read sequeuntially in SQR1
ADC1->CR1 |= ADC_CR1_SCAN;
// DMA = 1 in ADC_CR2, DDS = 1, continuous mode
ADC1->CR2 |= (ADC_CR2_DDS | ADC_CR2_DMA | ADC_CR2_CONT);

// DMA
RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

// Clear register, disable.
DMA2_Stream0->CR &= ~0x0;

// DMA2 channel 0 -> ADC1 = 0b000

// 16-bit Periph/Mem data size, incr memory when writing,
// don't increment peripheral when reading, read in circular mode.
DMA2_Stream0->CR |= (DMA_SxCR_MSIZE_0 | DMA_SxCR_PSIZE_0
 | DMA_SxCR_MINC | DMA_SxCR_CIRC);

// DIR = default peripheral to memory

// Data transfer size
DMA2_Stream0->NDTR = 0x5;
DMA2_Stream0->PAR = (uint32_t) (&(ADC1->DR));
DMA2_Stream0->M0AR = (uint32_t) (rxdata);



// ADON in CR2, then trigger to start

DMA2_Stream0->CR |= DMA_SxCR_EN;

ADC1->CR2 |= ADC_CR2_ADON;
ADC1->CR2 |= ADC_CR2_SWSTART;


}
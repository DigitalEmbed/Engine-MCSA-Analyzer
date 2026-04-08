#pragma once

#define ADC_NUM_CH             3u
#define NFFT                   2048u                  // 4096 se couber
#define ADC_SAMPLES_PER_CH     (2u * NFFT)            // half = NFFT por canal
#define ADC_DMA_LEN            (ADC_NUM_CH * ADC_SAMPLES_PER_CH)

#define FS_TOTAL_HZ            30000.0f               // TIM3 TRGO
#define FS_PER_CH              (FS_TOTAL_HZ / ADC_NUM_CH)  // 10 kS/s


/**
 * app.c — loop principal (sem RTOS), estilo do projeto
 * - Mantém app_main()
 * - Callbacks HAL só setam flags
 * - Heartbeat 1 Hz: imprime métricas detalhadas e envia frames binários
 */

#include "main.h"
#include "config.h"
#include "process.h"
#include "mcsa_stream.h"
#include "arm_math.h"
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

/* Externs HAL (gerados pelo Cube) */
extern ADC_HandleTypeDef hadc1;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim1;

/* Buffer DMA intercalado */
__attribute__((aligned(4))) volatile uint16_t adc_buffer[ADC_DMA_LEN];

/* Flags DMA */
static volatile uint8_t dma_half_ready = 0;
static volatile uint8_t dma_full_ready = 0;

/* Timebase 1 kHz (TIM1) */
static volatile uint64_t millis_tick = 0;

/* Retarget printf -> UART2 (bloqueante simples) */
int _write(int file, char *ptr, int len) {
    (void)file;
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, (uint16_t)len, HAL_MAX_DELAY);
    return len;
}

/* Callbacks HAL */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->Instance == ADC1) { dma_half_ready = 1; }
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if (hadc->Instance == ADC1) { dma_full_ready = 1; }
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM1) { millis_tick++; }
}

/* app_main: loop cooperativo */
int8_t app_main(void) {
    dsp_init();

    HAL_TIM_Base_Start_IT(&htim1);

    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_DMA_LEN);
    HAL_TIM_Base_Start(&htim3);

    printf("Init OK | Fs_total=%.1f Hz | ch=%u | NFFT=%u\r\n",
           (double)FS_TOTAL_HZ, (unsigned)ADC_NUM_CH, (unsigned)NFFT);

    uint64_t last_hb_ms = 0;

    for (;;) {
        __WFI();

        if (dma_half_ready == 1) {
            dma_half_ready = 0;
            process_block(0);
        }
        if (dma_full_ready == 1) {
            dma_full_ready = 0;
            process_block(ADC_DMA_LEN / 2);
        }

        if ((millis_tick - last_hb_ms) >= 1000) {
            last_hb_ms = millis_tick;

            /* Debug rápido da aquisição */
            uint16_t s0 = adc_buffer[0];
            uint16_t s1 = (ADC_NUM_CH > 1) ? adc_buffer[1] : 0;
            uint16_t s2 = (ADC_NUM_CH > 2) ? adc_buffer[2] : 0;
            printf("[HB %lu ms] adc[0..2]=(%u,%u,%u) Fs_ch=%.1fHz NFFT=%u\r\n",
                   (unsigned long)millis_tick, s0, s1, s2, (double)FS_PER_CH, (unsigned)NFFT);

            /* Métricas detalhadas */
            process_print_metrics_all();

            /* Envio binário para PC (um frame por canal) */
            for (uint32_t ch = 0; ch < ADC_NUM_CH; ++ch) {
                (void)mcsa_stream_send_channel(ch, &huart2);
            }
        }
    }
    // return 0;
}

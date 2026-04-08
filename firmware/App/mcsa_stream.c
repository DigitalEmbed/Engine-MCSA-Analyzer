#include "mcsa_stream.h"
#include "config.h"
#include <string.h>
#include <math.h>
#include "stm32l4xx_hal.h"

static inline void wr_u8 (uint8_t *buf, size_t *off, uint8_t v){ buf[(*off)++] = v; }
static inline void wr_u16(uint8_t *buf, size_t *off, uint16_t v){ memcpy(&buf[*off], &v, 2); *off += 2; }
static inline void wr_u32(uint8_t *buf, size_t *off, uint32_t v){ memcpy(&buf[*off], &v, 4); *off += 4; }
static inline void wr_f32(uint8_t *buf, size_t *off, float v)   { memcpy(&buf[*off], &v, 4); *off += 4; }

uint16_t mcsa_crc16_ccitt(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (int bit = 0; bit < 8; ++bit) {
            if ((crc & 0x8000) != 0) {
                crc = (uint16_t)((crc << 1) ^ 0x1021);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

int mcsa_stream_send_channel(uint32_t channel_id, void *huart_void) {
    if ((channel_id >= ADC_NUM_CH) || (huart_void == NULL)) {
        return -1;
    }
    UART_HandleTypeDef *huart = (UART_HandleTypeDef*)huart_void;

    const mcsa_metrics_t *m = mcsa_get_metrics(channel_id);
    if ((m == NULL) || (m->valid == false)) {
        return -2;
    }

    uint32_t nfft = (uint32_t)m->nfft;
    uint32_t bin_max = mcsa_hz_to_bin(F_SPEC_MAX_HZ, m->fs_hz, nfft);
    if (bin_max > nfft/2u) { bin_max = nfft/2u; }
    uint16_t n_bins = (uint16_t)(bin_max + 1u);

    static uint8_t frame[MCSA_FRAME_MAX];
    size_t off = 0;

    /* header */
    wr_u16(frame, &off, 0xA55A);
    wr_u8 (frame, &off, 1);                     /* version */
    wr_u8 (frame, &off, (uint8_t)channel_id);
    wr_u32(frame, &off, nfft);
    wr_f32(frame, &off, m->fs_hz);
    wr_u16(frame, &off, (uint16_t)m->dc_removed_i16);
    wr_f32(frame, &off, m->f1_hz);
    wr_f32(frame, &off, m->a1);
    wr_f32(frame, &off, m->thd);
    wr_f32(frame, &off, m->snr_db);
    wr_f32(frame, &off, m->slip_est);
    wr_f32(frame, &off, m->fr_hz);
    wr_f32(frame, &off, m->amp_fr);
    wr_f32(frame, &off, m->amp_2fr);

    /* harmônicos */
    uint16_t hcount = (m->harmonic_count > MCSA_MAX_HARMONICS) ? MCSA_MAX_HARMONICS : m->harmonic_count;
    wr_u16(frame, &off, hcount);
    for (uint16_t i = 0; i < hcount; ++i) {
        wr_u16(frame, &off, m->harmonics[i].order);
        wr_f32(frame, &off, m->harmonics[i].freq_hz);
        wr_f32(frame, &off, m->harmonics[i].amplitude);
    }

    /* espectro */
    wr_u16(frame, &off, n_bins);
    static float temp_spec[(NFFT/2)+1];
    size_t copied = mcsa_copy_last_spectrum(channel_id, temp_spec, (size_t)n_bins);
    if (copied != n_bins) {
        return -3;
    }
    for (uint16_t k = 0; k < n_bins; ++k) {
        wr_f32(frame, &off, temp_spec[k]);
    }

    /* CRC sobre [version..fim] */
    uint16_t crc = mcsa_crc16_ccitt(&frame[2], off - 2);
    wr_u16(frame, &off, crc);

    if (off > MCSA_FRAME_MAX) {
        return -4;
    }
    HAL_StatusTypeDef st = HAL_UART_Transmit(huart, frame, (uint16_t)off, 200);
    return (st == HAL_OK) ? 0 : -5;
}

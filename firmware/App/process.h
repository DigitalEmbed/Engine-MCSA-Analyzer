#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "config.h"   // NFFT, FS_PER_CH, etc.

/* ===================== Parâmetros ===================== */

#ifndef MCSA_MAX_HARMONICS
#define MCSA_MAX_HARMONICS  4u
#endif

#ifndef MCSA_MAINS_SEARCH_BAND_HZ
#define MCSA_MAINS_SEARCH_BAND_HZ  5.0f
#endif

#ifndef MCSA_MAINS_NOMINAL_HZ
#define MCSA_MAINS_NOMINAL_HZ  60.0f
#endif

/* ===================== Tipos ===================== */

typedef struct {
    uint16_t order;
    float    freq_hz;
    float    amplitude;
} mcsa_harmonic_t;

typedef struct {
    float f_low_hz;
    float f_high_hz;
    float amp_low;
    float amp_high;
    float ratio_to_f1;
} mcsa_sidebands_t;

typedef struct {
    /* domínio do tempo */
    int16_t dc_removed_i16;
    float   mean;
    float   std_dev;
    float   rms;

    /* fundamental e harmônicos */
    float   f1_hz;
    float   a1;
    uint16_t harmonic_count;
    mcsa_harmonic_t harmonics[MCSA_MAX_HARMONICS];
    float   thd;

    /* sidebands e slip */
    float   slip_est;
    mcsa_sidebands_t f1_sidebands;

    /* mecânicas */
    float   fr_hz;
    float   amp_fr;
    float   amp_2fr;

    /* ruído */
    float   noise_floor_db;
    float   snr_db;

    /* housekeeping */
    float    fs_hz;
    uint32_t nfft;
    uint64_t t_ms;
    bool     valid;
} mcsa_metrics_t;

/* ===================== API ===================== */

void dsp_init(void);
void process_block(size_t dma_offset);
void mcsa_time_stats_i16(const int16_t *samples, size_t count,
                         float *out_mean, float *out_std_dev, float *out_rms);

void mcsa_extract_metrics(uint32_t channel_id,
                          const float *mag_values, size_t mag_len,
                          float fs_hz, int16_t dc_removed_i16,
                          float mains_nominal_hz, float search_band_hz,
                          mcsa_metrics_t *out_metrics);

void mcsa_print_metrics(uint32_t channel_id, const mcsa_metrics_t *m);

/* acesso aos dados processados */
const mcsa_metrics_t* mcsa_get_metrics(uint32_t channel_id);
size_t mcsa_copy_last_spectrum(uint32_t channel_id, float *dst, size_t max_out);
void mcsa_print_metrics_detailed(uint32_t channel_id, uint32_t max_harmonics_to_show);
void process_print_metrics_all(void);

/* helpers freq <-> bin */
static inline uint32_t mcsa_hz_to_bin(float f_hz, float fs_hz, uint32_t nfft) {
    if (f_hz < 0.0f) { f_hz = 0.0f; }
    float kf = (f_hz / fs_hz) * (float)nfft;
    uint32_t k = (uint32_t)(kf + 0.5f);
    uint32_t kmax = nfft / 2u;
    return (k > kmax) ? kmax : k;
}
static inline float mcsa_bin_to_hz(uint32_t k, float fs_hz, uint32_t nfft) {
    if (k > nfft / 2u) { k = nfft / 2u; }
    return (fs_hz * (float)k) / (float)nfft;
}

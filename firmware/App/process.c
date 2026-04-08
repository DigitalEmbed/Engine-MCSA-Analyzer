#include "process.h"
#include "arm_math.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

/* Buffer DMA intercalado definido em app.c */
extern volatile uint16_t adc_buffer[ADC_DMA_LEN];

/* Estado DSP */
static float hann_window[NFFT];
static float fft_input[NFFT];
static float fft_output[NFFT];            /* saída packed da RFFT */
static float mag_spectrum[(NFFT/2)+1];

static arm_rfft_fast_instance_f32 rfft_instance;
static uint8_t dsp_initialized = 0;

/* Dados processados */
static mcsa_metrics_t g_last_metrics[ADC_NUM_CH];
static float g_last_mag[ADC_NUM_CH][(NFFT/2)+1];
static const size_t G_MAG_LEN = (NFFT/2) + 1;

/* ===================== Helpers internos ===================== */

static int16_t remove_dc_i16(int16_t *restrict samples, size_t count) {
    int64_t acc = 0;
    for (size_t i = 0; i < count; ++i) {
        acc += (int32_t)samples[i];
    }
    int16_t mean_i16 = (int16_t)((acc + (int64_t)(count/2)) / (int64_t)count);
    for (size_t i = 0; i < count; ++i) {
        samples[i] = (int16_t)(samples[i] - mean_i16);
    }
    return mean_i16;
}

static void remove_dc_3ch_i16(int16_t *ch0, int16_t *ch1, int16_t *ch2, size_t count,
                              int16_t *dc0, int16_t *dc1, int16_t *dc2) {
    if (dc0 != NULL) { *dc0 = remove_dc_i16(ch0, count); } else { (void)remove_dc_i16(ch0, count); }
    if (dc1 != NULL) { *dc1 = remove_dc_i16(ch1, count); } else { (void)remove_dc_i16(ch1, count); }
    if (dc2 != NULL) { *dc2 = remove_dc_i16(ch2, count); } else { (void)remove_dc_i16(ch2, count); }
}

static inline uint32_t clamp_u32(uint32_t value, uint32_t lo, uint32_t hi) {
    if (value < lo) { return lo; }
    if (value > hi) { return hi; }
    return value;
}

static uint32_t find_peak_bin_range(const float *mag_values, uint32_t bin_lo, uint32_t bin_hi) {
    if (mag_values == NULL) { return bin_lo; }
    if (bin_hi < bin_lo) { uint32_t t = bin_lo; bin_lo = bin_hi; bin_hi = t; }
    uint32_t best_bin = bin_lo;
    float best_amp = mag_values[bin_lo];
    for (uint32_t k = bin_lo + 1; k <= bin_hi; ++k) {
        if (mag_values[k] > best_amp) {
            best_amp = mag_values[k];
            best_bin = k;
        }
    }
    return best_bin;
}

static int is_in_notch(float freq_hz, float center_hz, float half_bw_hz) {
    return (freq_hz >= center_hz - half_bw_hz) && (freq_hz <= center_hz + half_bw_hz);
}

static void estimate_sidebands_and_slip(const float *mag_values, uint32_t nfft,
                                        float fs_hz, float f1_hz,
                                        float slip_min, float slip_max,
                                        mcsa_sidebands_t *out_sb, float a1) {
    memset(out_sb, 0, sizeof(*out_sb));
    if ((mag_values == NULL) || (f1_hz <= 0.0f) || (a1 <= 0.0f)) {
        out_sb->ratio_to_f1 = 0.0f;
        return;
    }

    const uint32_t nyquist_bin = nfft / 2u;
    const float bin_hz = fs_hz / (float)nfft;

    float best_sum = 0.0f;
    float best_delta = 0.0f;

    float fmin = (2.0f * slip_min) * f1_hz;
    float fmax = (2.0f * slip_max) * f1_hz;

    uint32_t d_min = (uint32_t)floorf(fmin / bin_hz);
    uint32_t d_max = (uint32_t)ceilf (fmax / bin_hz);
    uint32_t bin_f1 = mcsa_hz_to_bin(f1_hz, fs_hz, nfft);

    if (d_min < 1u) { d_min = 1u; }

    for (uint32_t d = d_min; d <= d_max; ++d) {
        int32_t bin_low  = (int32_t)bin_f1 - (int32_t)d;
        uint32_t bin_high = bin_f1 + d;
        if ((bin_low < 1) || (bin_high > nyquist_bin - 1u)) {
            continue;
        }
        float amp_low  = mag_values[(uint32_t)bin_low];
        float amp_high = mag_values[bin_high];
        float sum = amp_low + amp_high;
        if (sum > best_sum) {
            best_sum = sum;
            best_delta = (float)d * bin_hz;
            out_sb->amp_low  = amp_low;
            out_sb->amp_high = amp_high;
            out_sb->f_low_hz  = mcsa_bin_to_hz((uint32_t)bin_low, fs_hz, nfft);
            out_sb->f_high_hz = mcsa_bin_to_hz(bin_high, fs_hz, nfft);
        }
    }

    out_sb->ratio_to_f1 = (a1 > 0.0f) ? (best_sum / a1) : 0.0f;
    (void)best_delta;
}

/* ===================== API ===================== */

void dsp_init(void) {
    for (uint32_t n = 0; n < NFFT; ++n) {
        float theta = (2.0f * PI * (float)n) / (float)(NFFT - 1u);
        float c = arm_cos_f32(theta);
        hann_window[n] = 0.5f * (1.0f - c);
    }
    arm_rfft_fast_init_f32(&rfft_instance, NFFT);

    memset(g_last_metrics, 0, sizeof(g_last_metrics));
    memset(g_last_mag, 0, sizeof(g_last_mag));
    for (uint32_t ch = 0; ch < ADC_NUM_CH; ++ch) {
        g_last_metrics[ch].fs_hz = FS_PER_CH;
        g_last_metrics[ch].nfft  = NFFT;
        g_last_metrics[ch].valid = false;
    }
    dsp_initialized = 1;
}

void process_block(size_t dma_offset) {
    if (dsp_initialized == 0) {
        dsp_init();
    }

    const size_t interleaved_count = ADC_DMA_LEN / 2u;
    const size_t samples_per_channel = interleaved_count / ADC_NUM_CH; /* = NFFT */

    static int16_t ch0[NFFT];
    static int16_t ch1[NFFT];
    static int16_t ch2[NFFT];

    /* de-interleave */
    for (size_t i = 0, j = 0; i < samples_per_channel; ++i, j += ADC_NUM_CH) {
        ch0[i] = (int16_t)adc_buffer[dma_offset + j + 0u];
        ch1[i] = (int16_t)adc_buffer[dma_offset + j + 1u];
        ch2[i] = (int16_t)adc_buffer[dma_offset + j + 2u];
    }

    /* remove DC */
    int16_t dc0 = 0, dc1 = 0, dc2 = 0;
    remove_dc_3ch_i16(ch0, ch1, ch2, samples_per_channel, &dc0, &dc1, &dc2);

    /* para cada canal: i16->f32, janela, RFFT, magnitude, métricas */
    for (uint32_t ch = 0; ch < ADC_NUM_CH; ++ch) {
        int16_t *src_i16 = (ch == 0u) ? ch0 : ((ch == 1u) ? ch1 : ch2);

        const float scale = 1.0f / 2048.0f;
        for (size_t i = 0; i < samples_per_channel; ++i) {
            fft_input[i] = (float)src_i16[i] * scale * hann_window[i];
        }

        arm_rfft_fast_f32(&rfft_instance, fft_input, fft_output, 0);

        mag_spectrum[0] = fabsf(fft_output[0]);
        if (NFFT > 2u) {
            arm_cmplx_mag_f32(&fft_output[2], &mag_spectrum[1], (NFFT/2u) - 1u);
        }
        mag_spectrum[NFFT/2u] = fabsf(fft_output[1]);

        mcsa_metrics_t metrics_local;
        memset(&metrics_local, 0, sizeof(metrics_local));

        const int16_t dc_removed_i16 = (ch == 0u) ? dc0 : ((ch == 1u) ? dc1 : dc2);

        mcsa_extract_metrics(ch,
                             mag_spectrum, (NFFT/2u) + 1u,
                             FS_PER_CH, dc_removed_i16,
                             MCSA_MAINS_NOMINAL_HZ, MCSA_MAINS_SEARCH_BAND_HZ,
                             &metrics_local);

        g_last_metrics[ch] = metrics_local;
        memcpy(g_last_mag[ch], mag_spectrum, G_MAG_LEN * sizeof(float));
    }
}

void mcsa_time_stats_i16(const int16_t *samples, size_t count,
                         float *out_mean, float *out_std_dev, float *out_rms) {
    if ((samples == NULL) || (count == 0u)) {
        if (out_mean != NULL) { *out_mean = 0.0f; }
        if (out_std_dev != NULL) { *out_std_dev = 0.0f; }
        if (out_rms != NULL) { *out_rms = 0.0f; }
        return;
    }

    int64_t acc = 0;
    int64_t acc2 = 0;
    for (size_t i = 0; i < count; ++i) {
        int32_t v = samples[i];
        acc += v;
        acc2 += (int64_t)v * (int64_t)v;
    }

    float n_f = (float)count;
    float mean = (float)acc / n_f;
    float ex2 = (float)acc2 / n_f;

    if (out_mean != NULL) { *out_mean = mean; }
    if (out_rms  != NULL) { *out_rms  = sqrtf(ex2); }
    if (out_std_dev != NULL) {
        float var = ex2 - mean * mean;
        if (var < 0.0f) { var = 0.0f; }
        *out_std_dev = sqrtf(var);
    }
}

void mcsa_extract_metrics(uint32_t channel_id,
                          const float *mag_values, size_t mag_len,
                          float fs_hz, int16_t dc_removed_i16,
                          float mains_nominal_hz, float search_band_hz,
                          mcsa_metrics_t *out_metrics) {
    (void)channel_id;

    if (out_metrics == NULL) {
        return;
    }
    memset(out_metrics, 0, sizeof(*out_metrics));
    out_metrics->fs_hz = fs_hz;
    out_metrics->nfft  = (mag_len > 0u) ? (uint32_t)((mag_len - 1u) * 2u) : 0u;
    out_metrics->dc_removed_i16 = dc_removed_i16;

    if ((mag_values == NULL) || (mag_len < 3u) || (fs_hz <= 0.0f) || (out_metrics->nfft == 0u)) {
        out_metrics->valid = false;
        return;
    }

    const uint32_t nfft = out_metrics->nfft;
    const uint32_t nyquist_bin = nfft / 2u;

    /* 1) Fundamental em torno da rede */
    uint32_t bin_lo = mcsa_hz_to_bin(mains_nominal_hz - search_band_hz, fs_hz, nfft);
    uint32_t bin_hi = mcsa_hz_to_bin(mains_nominal_hz + search_band_hz, fs_hz, nfft);
    bin_lo = clamp_u32(bin_lo, 1u, nyquist_bin - 1u);
    bin_hi = clamp_u32(bin_hi, 1u, nyquist_bin - 1u);

    uint32_t bin_f1 = find_peak_bin_range(mag_values, bin_lo, bin_hi);
    float f1_hz = mcsa_bin_to_hz(bin_f1, fs_hz, nfft);
    float a1 = mag_values[bin_f1];
    out_metrics->f1_hz = f1_hz;
    out_metrics->a1    = a1;

    /* 2) Harmônicos ímpares (3,5,7,...) */
    out_metrics->harmonic_count = 0u;
    for (uint16_t order = 3u; order <= 31u && out_metrics->harmonic_count < MCSA_MAX_HARMONICS; order += 2u) {
        float fhz = f1_hz * (float)order;
        if (fhz >= fs_hz * 0.5f) {
            break;
        }
        uint32_t bin_h = mcsa_hz_to_bin(fhz, fs_hz, nfft);
        uint32_t bin_h0 = (bin_h > 1u) ? (bin_h - 1u) : bin_h;
        uint32_t bin_h1 = (bin_h + 1u < nyquist_bin) ? (bin_h + 1u) : nyquist_bin;
        uint32_t bin_peak = find_peak_bin_range(mag_values, bin_h0, bin_h1);

        mcsa_harmonic_t *h = &out_metrics->harmonics[out_metrics->harmonic_count++];
        h->order     = order;
        h->freq_hz   = mcsa_bin_to_hz(bin_peak, fs_hz, nfft);
        h->amplitude = mag_values[bin_peak];
    }

    /* 3) THD */
    if ((a1 > 0.0f) && (out_metrics->harmonic_count > 0u)) {
        float sum_sq = 0.0f;
        for (uint16_t i = 0; i < out_metrics->harmonic_count; ++i) {
            float a = out_metrics->harmonics[i].amplitude;
            sum_sq += a * a;
        }
        out_metrics->thd = sqrtf(sum_sq) / a1;
    } else {
        out_metrics->thd = 0.0f;
    }

    /* 4) Sidebands e slip */
    {
        const float slip_min = 0.005f;
        const float slip_max = 0.08f;
        estimate_sidebands_and_slip(mag_values, nfft, fs_hz, f1_hz, slip_min, slip_max,
                                    &out_metrics->f1_sidebands, a1);
        if ((out_metrics->f1_sidebands.f_high_hz > 0.0f) && (out_metrics->f1_sidebands.f_low_hz > 0.0f)) {
            float delta = fabsf(out_metrics->f1_sidebands.f_high_hz - f1_hz);
            out_metrics->slip_est = delta / (2.0f * f1_hz);
        } else {
            out_metrics->slip_est = -1.0f;
        }
    }

    /* 5) Pico mecânico fr (2..100 Hz), evitando rede */
    {
        uint32_t bin_a = mcsa_hz_to_bin(2.0f, fs_hz, nfft);
        uint32_t bin_b = mcsa_hz_to_bin(100.0f, fs_hz, nfft);
        if (bin_b > nyquist_bin) { bin_b = nyquist_bin; }

        float best_amp = 0.0f;
        uint32_t best_bin = bin_a;
        for (uint32_t k = bin_a; k <= bin_b; ++k) {
            float f_hz = mcsa_bin_to_hz(k, fs_hz, nfft);
            if (is_in_notch(f_hz, f1_hz, search_band_hz)) {
                continue;
            }
            if (mag_values[k] > best_amp) {
                best_amp = mag_values[k];
                best_bin = k;
            }
        }
        out_metrics->fr_hz  = mcsa_bin_to_hz(best_bin, fs_hz, nfft);
        out_metrics->amp_fr = best_amp;

        float f_2fr = 2.0f * out_metrics->fr_hz;
        uint32_t bin_2fr = mcsa_hz_to_bin(f_2fr, fs_hz, nfft);
        out_metrics->amp_2fr = (bin_2fr <= nyquist_bin) ? mag_values[bin_2fr] : 0.0f;
    }

    /* 6) Ruído médio (300..1000 Hz) e SNR relativo a A1 */
    {
        float f_lo = 300.0f;
        float f_hi = fs_hz * 0.5f - 1.0f;
        if (f_hi > 1000.0f) { f_hi = 1000.0f; }

        uint32_t bin_l = mcsa_hz_to_bin(f_lo, fs_hz, nfft);
        uint32_t bin_h = mcsa_hz_to_bin(f_hi, fs_hz, nfft);
        if (bin_l < 1u) { bin_l = 1u; }
        if (bin_h > nyquist_bin) { bin_h = nyquist_bin; }

        const float half_bw = 2.0f * (fs_hz / (float)nfft);
        double acc = 0.0;
        uint32_t count = 0u;

        for (uint32_t k = bin_l; k <= bin_h; ++k) {
            float f_hz = mcsa_bin_to_hz(k, fs_hz, nfft);
            if (is_in_notch(f_hz, f1_hz, half_bw)) {
                continue;
            }
            bool skip = false;
            for (uint16_t i = 0; i < out_metrics->harmonic_count; ++i) {
                if (is_in_notch(f_hz, out_metrics->harmonics[i].freq_hz, half_bw)) {
                    skip = true;
                    break;
                }
            }
            if (skip) { continue; }
            acc += (double)mag_values[k];
            count++;
        }

        float noise = (count > 0u) ? (float)(acc / (double)count) : 0.0f;
        out_metrics->noise_floor_db = (a1 > 0.0f && noise > 0.0f) ? (20.0f * log10f(noise / a1)) : -120.0f;
        out_metrics->snr_db = -out_metrics->noise_floor_db;
    }

    out_metrics->valid = true;
}

void mcsa_print_metrics(uint32_t channel_id, const mcsa_metrics_t *m) {
    if ((m == NULL) || (m->valid == false)) {
        return;
    }
    printf("CH%lu | f1=%.2fHz A1=%.4f | THD=%.1f%% | SB r=%.3f slip=%s%.3f | fr=%.1fHz A(fr)=%.4f | SNR=%.1fdB | DC=%d\r\n",
           (unsigned long)channel_id,
           m->f1_hz, m->a1, 100.0f * m->thd,
           m->f1_sidebands.ratio_to_f1,
           (m->slip_est < 0.0f ? "-" : ""), (m->slip_est < 0.0f ? -m->slip_est : m->slip_est),
           m->fr_hz, m->amp_fr, m->snr_db, (int)m->dc_removed_i16);
}

const mcsa_metrics_t* mcsa_get_metrics(uint32_t channel_id) {
    if (channel_id >= ADC_NUM_CH) {
        return NULL;
    }
    return &g_last_metrics[channel_id];
}

size_t mcsa_copy_last_spectrum(uint32_t channel_id, float *dst, size_t max_out) {
    if ((channel_id >= ADC_NUM_CH) || (dst == NULL) || (max_out == 0u)) {
        return 0;
    }
    size_t count = (G_MAG_LEN <= max_out) ? G_MAG_LEN : max_out;
    memcpy(dst, g_last_mag[channel_id], count * sizeof(float));
    return count;
}

void mcsa_print_metrics_detailed(uint32_t channel_id, uint32_t max_harmonics_to_show) {
    const mcsa_metrics_t *m = mcsa_get_metrics(channel_id);
    if ((m == NULL) || (m->valid == false)) {
        return;
    }

    printf("CH%lu | f1=%.2f Hz A1=%.4f | THD=%.2f%% | SNR=%.1f dB | DC=%d\r\n",
           (unsigned long)channel_id, m->f1_hz, m->a1, 100.0f*m->thd, m->snr_db, (int)m->dc_removed_i16);

    uint32_t to_show = (m->harmonic_count < max_harmonics_to_show) ? m->harmonic_count : max_harmonics_to_show;
    if (to_show > 0u) {
        printf("  Harm: ");
        for (uint32_t i = 0; i < to_show; ++i) {
            const mcsa_harmonic_t *h = &m->harmonics[i];
            printf("%uª=%.2fHz(%.4f)%s", h->order, h->freq_hz, h->amplitude, (i + 1u < to_show) ? ", " : "\r\n");
        }
    }

    if (m->slip_est >= 0.0f) {
        printf("  Bars SB: fL=%.2fHz(%.4f) fH=%.2fHz(%.4f) | ratio=%.3f | slip~%.3f\r\n",
               m->f1_sidebands.f_low_hz, m->f1_sidebands.amp_low,
               m->f1_sidebands.f_high_hz, m->f1_sidebands.amp_high,
               m->f1_sidebands.ratio_to_f1, m->slip_est);
    } else {
        printf("  Bars SB: (não detectado na faixa)\r\n");
    }

    printf("  Mech: fr=%.2fHz A(fr)=%.4f  2fr=%.2fHz A(2fr)=%.4f\r\n",
           m->fr_hz, m->amp_fr, 2.0f*m->fr_hz, m->amp_2fr);

    const float a1 = (m->a1 > 0.0f) ? m->a1 : 1.0f;
    if ((m->fr_hz > 1.0f) && (m->amp_fr > 0.0f)) {
        if ((m->amp_fr > 0.2f * a1) && (m->amp_2fr < 0.1f * a1)) {
            printf("  Hint: forte 1×fr -> possível desbalanceio.\r\n");
        } else if (m->amp_2fr > 0.15f * a1) {
            printf("  Hint: forte 2×fr -> possível desalinhamento/folga.\r\n");
        }
    }
}

void process_print_metrics_all(void) {
    for (uint32_t ch = 0; ch < ADC_NUM_CH; ++ch) {
        mcsa_print_metrics_detailed(ch, MCSA_MAX_HARMONICS);
    }
}

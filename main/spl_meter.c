/*
 * Sound pressure level (SPL) meter using an I2S MEMS microphone.
 *
 * Adapted to ESP-IDF (C, new I2S std driver, portable IIR filters) from
 * "Display A-weighted sound level measured by I2S Microphone"
 * (c)2019 Ivan Kostoski
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Operation:
 *  - I2S is sampled at 32 kHz (fixed by the design of the IIR filters).
 *  - mic_i2s_reader_task reads the data in small chunks, runs it through the
 *    microphone equalizer filter and the weighting filter and pushes the sum of
 *    squares of each 125 ms block into a queue.
 *  - spl_calc_task converts the sums into dB values relative to the microphone
 *    reference amplitude, calculates Leq over CONFIG_SPL_LEQ_PERIOD_MS and
 *    publishes it via MQTT.
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "sdkconfig.h"
#include "driver/i2s_std.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "mqtt_sensor_client.h"
#include "spl_meter.h"
#include "driver/gpio.h"

static const char *TAG = "SPL";

/* ------------------------------------------------------------------------- */
/* Configuration                                                             */
/* ------------------------------------------------------------------------- */

#define SAMPLE_RATE         32000               /* Hz, fixed to design of IIR filters */
#define SAMPLE_BITS         32                  /* slot width on the I2S bus */
#define SAMPLES_SHORT       (SAMPLE_RATE / 8)   /* 125 ms block */
#define SAMPLES_CHUNK       1000                /* samples per I2S read, must divide SAMPLES_SHORT */
#define SAMPLES_LEQ         (SAMPLE_RATE * CONFIG_SPL_LEQ_PERIOD_MS / 1000)

#define DMA_FRAME_NUM       500
#define DMA_DESC_NUM        16

#define MIC_OFFSET_DB       3.0103              /* sine-wave RMS vs. dBFS, modify for linear calibration */
#define MIC_SENSITIVITY     CONFIG_SPL_MIC_SENSITIVITY_DBFS
#define MIC_REF_DB          CONFIG_SPL_MIC_REF_DB
#define MIC_OVERLOAD_DB     CONFIG_SPL_MIC_OVERLOAD_DB
#define MIC_NOISE_DB        CONFIG_SPL_MIC_NOISE_DB
#define MIC_BITS            CONFIG_SPL_MIC_BITS

/* The calc task needs ~0.2% CPU but must never be starved (a full queue drops blocks), so it is above
 * mqtt_task (5, TLS handshakes are CPU heavy) and the other application tasks. */
#define I2S_TASK_PRI        7
#define I2S_TASK_STACK      4096
#define CALC_TASK_PRI       6
#define CALC_TASK_STACK     4096

#if CONFIG_SPL_DIAGNOSTICS
#define MON_TASK_PRI        8                   /* above the reader so the report is printed even when overloaded */
#define MON_TASK_STACK      4096
#define MON_PERIOD_MS       5000

/* Time available to process one I2S chunk in real time */
#define CHUNK_BUDGET_US     ((int64_t)SAMPLES_CHUNK * 1000000 / SAMPLE_RATE)
#endif

_Static_assert((SAMPLES_SHORT % SAMPLES_CHUNK) == 0, "SAMPLES_CHUNK must divide SAMPLES_SHORT");

/* ------------------------------------------------------------------------- */
/* Second-order-sections IIR filter                                          */
/* ------------------------------------------------------------------------- */

/*
 * Filters are a cascade of second-order sections with b0 = a0 = 1.0 and the
 * 'gain' applied after the last section. The a1/a2 coefficients are stored
 * negated (as produced by the filter design scripts of the original project):
 *
 *   H(z) = (1 + b1 z^-1 + b2 z^-2) / (1 - a1 z^-1 - a2 z^-2)
 *
 * The ESP32-C6 has no FPU, so the filters run in integer arithmetic (about
 * 8x cheaper than soft-float). Coefficients are converted to Q29 at init.
 * Each section is a transposed direct form II with 64-bit Q29 state:
 *
 *   y  = x + s1
 *   s1 = b1 * x + a1 * y + s2
 *   s2 = b2 * x + a2 * y
 *
 * The state and the feedback path keep the full precision of y (integer part
 * and 29-bit fraction), only the value passed on to the next section is
 * rounded to an integer sample. The input is the (24 bit) microphone sample.
 *
 * IMPORTANT: a section's zeros close to DC must be paired with the poles close
 * to DC in the same section (they cancel the huge low frequency gain of the
 * poles). Otherwise the rounding noise between sections is amplified by up to
 * 100 dB at low frequencies.
 */

#define SOS_MAX_SECTIONS 3
#define SOS_Q            29

typedef struct {
    float b1;
    float b2;
    float a1;
    float a2;
} sos_coeffs_t;

typedef struct {
    int32_t b1;
    int32_t b2;
    int32_t a1;
    int32_t a2;
} sos_coeffs_q_t;

typedef struct {
    int64_t s1;
    int64_t s2;
} sos_state_t;

typedef struct {
    int32_t gain;       /* Q29 */
    size_t num_sos;
    sos_coeffs_q_t sos[SOS_MAX_SECTIONS];
    sos_state_t w[SOS_MAX_SECTIONS];
} sos_filter_t;

#define SOS_COUNT(a) (sizeof(a) / sizeof((a)[0]))

#if CONFIG_SPL_MIC_DC_BLOCKER_ONLY
/* DC-blocker, a1 = 0.9988 heavily attenuates frequencies below 10 Hz (Fs = 32 kHz) */
static const sos_coeffs_t DC_BLOCKER_SOS[] = {
    {-1.0f, 0.0f, +0.9988f, 0.0f},
};
#endif

#if CONFIG_SPL_MIC_ICS43434
/* TDK/InvenSense ICS-43434, Fs = 32 kHz (refitted from the 48 kHz design) */
static const sos_coeffs_t ICS43434_SOS[] = {
    {+0.996498536617843f, +0.216410090586061f, -0.615459867185092f, -0.015896737666048f},
    {-1.983604244360304f, +0.983671494444864f, +1.996325628098673f, -0.996328994679850f},
};
#define ICS43434_GAIN 0.728034123679601f
#endif

#if CONFIG_SPL_MIC_ICS43432
/* TDK/InvenSense ICS-43432, Fs = 32 kHz (refitted from the 48 kHz design) */
static const sos_coeffs_t ICS43432_SOS[] = {
    {-0.722549186773345f, -0.034406596838015f, -0.066407195132619f, -0.087243387063456f},
    {-1.867741676712384f, +0.869752817272554f, +1.690529733981306f, -0.720070890476632f},
    {+1.113370468960765f, +0.279908015328788f, +0.317617623516459f, +0.676139970282671f},
};
#define ICS43432_GAIN -0.678555067070096f
#endif

#if CONFIG_SPL_MIC_INMP441
/* TDK/InvenSense INMP441, Fs = 32 kHz (refitted from the 48 kHz design) */
static const sos_coeffs_t INMP441_SOS[] = {
    {-1.980412778699711f, +0.980508693433226f, +1.992772118411720f, -0.992785178958565f},
};
#define INMP441_GAIN 1.004054147257500f
#endif

#if CONFIG_SPL_WEIGHTING_A
/* A-weighting, Fs = 32 kHz (refitted from the 48 kHz design by Dr. Matt L., https://dsp.stackexchange.com/a/36122) */
static const sos_coeffs_t A_WEIGHTING_SOS[] = {
    /* double zero at DC with the double pole at 20.6 Hz */
    {-2.000404520228076f, +1.000405870540219f, +1.973438660309667f, -0.973565076099889f},
    /* zero at DC with the high frequency poles */
    {-0.853021517560096f, -0.147006595392790f, -1.048836720591701f, -0.234932360967442f},
    /* 107 Hz and 737 Hz poles */
    {+3.087604603951603f, +1.753020042737736f, +1.026432215714052f, -0.140112941122877f},
};
#define A_WEIGHTING_GAIN 0.331042907341575f
#endif

#if CONFIG_SPL_WEIGHTING_C
/* C-weighting, Fs = 32 kHz (refitted from the 48 kHz design) */
static const sos_coeffs_t C_WEIGHTING_SOS[] = {
    /* double zero at DC with the 20 Hz pole pair */
    {-2.000000018805878f, +1.000000018798071f, +1.991927156328619f, -0.991943449232784f},
    {+1.336159408766719f, +0.428637253011002f, +0.312064612810319f, -0.025838265408680f},
    {+0.058804135275968f, -0.037960472031892f, -1.266850857195736f, -0.377420486633736f},
};
#define C_WEIGHTING_GAIN -0.670782154866622f
#endif

static int32_t to_q29(float v)
{
    return (int32_t)lroundf(v * (float)(1 << SOS_Q));
}

static void sos_filter_init(sos_filter_t *f, float gain, const sos_coeffs_t *sos, size_t num_sos)
{
    memset(f, 0, sizeof(*f));
    f->gain = to_q29(gain);
    f->num_sos = num_sos;
    for (size_t i = 0; i < num_sos; i++) {
        f->sos[i].b1 = to_q29(sos[i].b1);
        f->sos[i].b2 = to_q29(sos[i].b2);
        f->sos[i].a1 = to_q29(sos[i].a1);
        f->sos[i].a2 = to_q29(sos[i].a2);
    }
}

/**
 * Filter 'len' samples in place and return the sum of squares of the output.
 * Input and output samples are integers (24 bit microphone range).
 */
static uint64_t sos_filter_apply(sos_filter_t *f, int32_t *buf, size_t len)
{
    for (size_t s = 0; s < f->num_sos; s++) {
        const int32_t b1 = f->sos[s].b1;
        const int32_t b2 = f->sos[s].b2;
        const int32_t a1 = f->sos[s].a1;
        const int32_t a2 = f->sos[s].a2;
        int64_t s1 = f->w[s].s1;
        int64_t s2 = f->w[s].s2;

        for (size_t i = 0; i < len; i++) {
            const int32_t x = buf[i];
            const int64_t y64 = ((int64_t)x << SOS_Q) + s1;                         /* Q29 */
            const int32_t y = (int32_t)((y64 + (1 << (SOS_Q - 1))) >> SOS_Q);       /* rounded to integer */
            const int32_t yf = (int32_t)(y64 - ((int64_t)y << SOS_Q));              /* fraction, Q29 */
            s1 = (int64_t)b1 * x + (int64_t)a1 * y + (((int64_t)a1 * yf) >> SOS_Q) + s2;
            s2 = (int64_t)b2 * x + (int64_t)a2 * y + (((int64_t)a2 * yf) >> SOS_Q);
            buf[i] = y;
        }

        f->w[s].s1 = s1;
        f->w[s].s2 = s2;
    }

    /* Gain and sum of squares */
    const int32_t gain = f->gain;
    uint64_t sum_sqr = 0;
    for (size_t i = 0; i < len; i++) {
        const int32_t o = (int32_t)(((int64_t)buf[i] * gain + (1 << (SOS_Q - 1))) >> SOS_Q);
        buf[i] = o;
        sum_sqr += (uint64_t)((int64_t)o * o);
    }
    return sum_sqr;
}

/* ------------------------------------------------------------------------- */
/* Microphone / weighting selection                                          */
/* ------------------------------------------------------------------------- */

#if CONFIG_SPL_MIC_ICS43434
#define MIC_EQ_SOS      ICS43434_SOS
#define MIC_EQ_GAIN     ICS43434_GAIN
#elif CONFIG_SPL_MIC_ICS43432
#define MIC_EQ_SOS      ICS43432_SOS
#define MIC_EQ_GAIN     ICS43432_GAIN
#elif CONFIG_SPL_MIC_INMP441
#define MIC_EQ_SOS      INMP441_SOS
#define MIC_EQ_GAIN     INMP441_GAIN
#else /* DC blocker only */
#define MIC_EQ_SOS      DC_BLOCKER_SOS
#define MIC_EQ_GAIN     1.0f
#endif

#if CONFIG_SPL_WEIGHTING_A
#define WEIGHTING_NAME  "A"
#define DB_UNITS        "dBA"
#elif CONFIG_SPL_WEIGHTING_C
#define WEIGHTING_NAME  "C"
#define DB_UNITS        "dBC"
#else
#define WEIGHTING_NAME  "Z"
#define DB_UNITS        "dBZ"
#endif

static sos_filter_t mic_equalizer;
static sos_filter_t weighting;

/* ------------------------------------------------------------------------- */
/* I2S microphone                                                            */
/* ------------------------------------------------------------------------- */

typedef struct {
    uint64_t sum_sqr_spl;      /* sum of squares of samples after the equalizer filter */
    uint64_t sum_sqr_weighted; /* sum of squares of weighted samples */
#if CONFIG_SPL_DIAGNOSTICS
    int32_t raw_min;        /* diagnostics: raw I2S word statistics of the block */
    int32_t raw_max;
    int64_t raw_sum;
    uint32_t raw_zeros;     /* number of raw words equal to 0 */
#endif
} sum_queue_t;

static QueueHandle_t samples_queue;
static i2s_chan_handle_t rx_chan;
static TaskHandle_t reader_task_handle;
static TaskHandle_t calc_task_handle;

#if CONFIG_SPL_DIAGNOSTICS
/* Performance counters, written by the reader/calc tasks and read by the monitor task.
 * Torn reads are harmless (diagnostics only). */
typedef struct {
    /* reader task, microseconds of wall time spent in each section */
    volatile uint64_t rd_wait_us;       /* blocked in i2s_channel_read() waiting for DMA data */
    volatile uint64_t rd_stats_us;      /* raw min/max/sum diagnostics loop */
    volatile uint64_t rd_convert_us;    /* shifting I2S words to the microphone bit depth */
    volatile uint64_t rd_eq_us;         /* microphone equalizer filter */
    volatile uint64_t rd_weight_us;     /* weighting filter */
    volatile uint32_t rd_chunks;
    volatile uint32_t rd_overruns;      /* chunks that took longer to process than they take to record */
    volatile uint32_t rd_max_chunk_us;  /* slowest chunk (processing only, excluding the wait) */
    volatile uint32_t drops;            /* blocks dropped because the queue was full */
    volatile uint32_t queue_max;        /* highest queue depth seen */
    /* calc task */
    volatile uint64_t calc_busy_us;     /* total time spent handling blocks (including logging) */
    volatile uint64_t calc_log_us;      /* of which time spent in logging */
    volatile uint32_t calc_blocks;
    volatile uint32_t calc_max_us;      /* slowest block */
} perf_stats_t;

static perf_stats_t perf;
#endif /* CONFIG_SPL_DIAGNOSTICS */

/* Raw 32-bit I2S words, shifted to the microphone bit depth and filtered in place */
static int32_t chunk[SAMPLES_CHUNK];

/* SPL MQTT threshold: only publish readings above this level (dB) */
static double spl_mqtt_threshold = CONFIG_SPL_MQTT_THRESHOLD_DB;

/* Load SPL MQTT threshold from NVS, or use default from Kconfig */
static void spl_threshold_load(void)
{
    extern char* mqtt_nvs_get_parameter(const char *parameter);
    char *threshold_str = mqtt_nvs_get_parameter("spl_threshold");
    if (threshold_str != NULL) {
        double val = atof(threshold_str);
        if (val >= 0.0 && val <= 150.0) {
            spl_mqtt_threshold = val;
            ESP_LOGI(TAG, "SPL MQTT threshold loaded from NVS: %.1f dB", spl_mqtt_threshold);
        }
        free(threshold_str);
    } else {
        ESP_LOGI(TAG, "SPL MQTT threshold using default: %.1f dB", spl_mqtt_threshold);
    }
}

/* Save SPL MQTT threshold to NVS */
void spl_threshold_save(double threshold_db)
{
    extern void mqtt_nvs_save_parameters(const char *parameter, const char *value);
    if (threshold_db >= 0.0 && threshold_db <= 150.0) {
        char buf[16];
        snprintf(buf, sizeof(buf), "%.1f", threshold_db);
        mqtt_nvs_save_parameters("spl_threshold", buf);
        spl_mqtt_threshold = threshold_db;
        ESP_LOGI(TAG, "SPL MQTT threshold saved: %.1f dB", threshold_db);
    } else {
        ESP_LOGW(TAG, "SPL MQTT threshold out of range (0-150): %.1f dB", threshold_db);
    }
}

/* Get current SPL MQTT threshold */
double spl_threshold_get(void)
{
    return spl_mqtt_threshold;
}

static esp_err_t mic_i2s_init(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = DMA_DESC_NUM;
    chan_cfg.dma_frame_num = DMA_FRAME_NUM;
    ESP_RETURN_ON_ERROR(i2s_new_channel(&chan_cfg, NULL, &rx_chan), TAG, "i2s_new_channel failed");

    gpio_reset_pin(GPIO_NUM_17);
    gpio_set_direction(GPIO_NUM_17, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_17, 0);

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = (gpio_num_t)CONFIG_SPL_I2S_SCK_GPIO,
            .ws   = (gpio_num_t)CONFIG_SPL_I2S_WS_GPIO,
            .dout = I2S_GPIO_UNUSED,
            .din  = (gpio_num_t)CONFIG_SPL_I2S_SD_GPIO,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
#if CONFIG_SPL_I2S_RIGHT_CHANNEL
    std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_RIGHT;
#else
    std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;
#endif

    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(rx_chan, &std_cfg), TAG, "i2s_channel_init_std_mode failed");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(rx_chan), TAG, "i2s_channel_enable failed");
    return ESP_OK;
}

/*
 * I2S reader task. Runs at a high priority and only does the minimum required
 * work: the data is 'compressed' into sums of squares per 125 ms block.
 */
static void mic_i2s_reader_task(void *arg)
{
    size_t bytes_read = 0;

    /* Discard the first chunk, the microphone may have a startup time (i.e. INMP441 up to 83 ms) */
    i2s_channel_read(rx_chan, chunk, sizeof(chunk), &bytes_read, pdMS_TO_TICKS(1000));
#if CONFIG_SPL_DIAGNOSTICS
    ESP_LOGI(TAG, "first I2S read: %u bytes, words: %08lx %08lx %08lx %08lx", (unsigned)bytes_read,
             (unsigned long)chunk[0], (unsigned long)chunk[1], (unsigned long)chunk[2],
             (unsigned long)chunk[3]);
#endif

    while (true) {
        sum_queue_t q = {0};
#if CONFIG_SPL_DIAGNOSTICS
        q.raw_min = INT32_MAX;
        q.raw_max = INT32_MIN;
#endif
        bool block_ok = true;

        for (int c = 0; c < SAMPLES_SHORT / SAMPLES_CHUNK; c++) {
#if CONFIG_SPL_DIAGNOSTICS
            int64_t t0 = esp_timer_get_time();
#endif
            esp_err_t err = i2s_channel_read(rx_chan, chunk, sizeof(chunk), &bytes_read, pdMS_TO_TICKS(1000));
#if CONFIG_SPL_DIAGNOSTICS
            int64_t t1 = esp_timer_get_time();
#endif
            if (err != ESP_OK || bytes_read != sizeof(chunk)) {
                ESP_LOGW(TAG, "I2S read failed: %s (%u bytes)", esp_err_to_name(err), (unsigned)bytes_read);
                block_ok = false;
                break;
            }

#if CONFIG_SPL_DIAGNOSTICS
            /* Raw statistics for diagnostics */
            for (int i = 0; i < SAMPLES_CHUNK; i++) {
                int32_t v = chunk[i];
                if (v < q.raw_min) {
                    q.raw_min = v;
                }
                if (v > q.raw_max) {
                    q.raw_max = v;
                }
                q.raw_sum += v;
                if (v == 0) {
                    q.raw_zeros++;
                }
            }
            int64_t t2 = esp_timer_get_time();
#endif

            /* Shift the I2S words to the microphone bit depth (sign preserving) */
            for (int i = 0; i < SAMPLES_CHUNK; i++) {
                chunk[i] >>= (SAMPLE_BITS - MIC_BITS);
            }
#if CONFIG_SPL_DIAGNOSTICS
            int64_t t3 = esp_timer_get_time();
#endif

            /* Equalization, Z-weighted sum of squares (filtered samples are written back to the buffer) */
            q.sum_sqr_spl += sos_filter_apply(&mic_equalizer, chunk, SAMPLES_CHUNK);
#if CONFIG_SPL_DIAGNOSTICS
            int64_t t4 = esp_timer_get_time();
#endif
            /* Weighting and weighted sum of squares */
            q.sum_sqr_weighted += sos_filter_apply(&weighting, chunk, SAMPLES_CHUNK);
#if CONFIG_SPL_DIAGNOSTICS
            int64_t t5 = esp_timer_get_time();

            perf.rd_wait_us += t1 - t0;
            perf.rd_stats_us += t2 - t1;
            perf.rd_convert_us += t3 - t2;
            perf.rd_eq_us += t4 - t3;
            perf.rd_weight_us += t5 - t4;
            perf.rd_chunks++;
            if ((t5 - t1) > CHUNK_BUDGET_US) {
                perf.rd_overruns++;
            }
            if ((uint32_t)(t5 - t1) > perf.rd_max_chunk_us) {
                perf.rd_max_chunk_us = (uint32_t)(t5 - t1);
            }
#endif
        }

        if (!block_ok) {
            continue;
        }

        /* Do not block the real-time reader, drop the block if the consumer is behind */
        if (xQueueSend(samples_queue, &q, 0) != pdTRUE) {
#if CONFIG_SPL_DIAGNOSTICS
            perf.drops++;
#endif
            ESP_LOGW(TAG, "Sample queue full, dropping block");
        }
#if CONFIG_SPL_DIAGNOSTICS
        uint32_t depth = uxQueueMessagesWaiting(samples_queue);
        if (depth > perf.queue_max) {
            perf.queue_max = depth;
        }
#endif
    }
}

/* ------------------------------------------------------------------------- */
/* Leq calculation and publishing                                            */
/* ------------------------------------------------------------------------- */

static void spl_calc_task(void *arg)
{
    /* Reference amplitude derived from the microphone sensitivity */
    const double mic_ref_ampl = pow(10.0, (double)MIC_SENSITIVITY / 20.0) * (double)((1ULL << (MIC_BITS - 1)) - 1);

    uint8_t mac[6];
    char mac_str[13];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(mac_str, sizeof(mac_str), "%02X%02X%02X%02X%02X%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    sum_queue_t q;
    uint32_t leq_samples = 0;
    double leq_sum_sqr = 0;
    bool overload = false;
#if CONFIG_SPL_DIAGNOSTICS
    double leq_sum_sqr_z = 0;
    int32_t raw_min = INT32_MAX;
    int32_t raw_max = INT32_MIN;
    int64_t raw_sum = 0;
    uint32_t raw_zeros = 0;
#endif

    while (xQueueReceive(samples_queue, &q, portMAX_DELAY)) {
#if CONFIG_SPL_DIAGNOSTICS
        int64_t tc0 = esp_timer_get_time();
        int64_t log_us = 0;
#endif

        /* dB value of the short block relative to the microphone reference */
        double short_rms = sqrt((double)q.sum_sqr_spl / SAMPLES_SHORT);
        double short_spl_db = MIC_OFFSET_DB + MIC_REF_DB + 20 * log10(short_rms / mic_ref_ampl);
        if (short_spl_db > MIC_OVERLOAD_DB) {
            overload = true;
        }

        leq_sum_sqr += q.sum_sqr_weighted;
        leq_samples += SAMPLES_SHORT;
#if CONFIG_SPL_DIAGNOSTICS
        leq_sum_sqr_z += q.sum_sqr_spl;
        if (q.raw_min < raw_min) {
            raw_min = q.raw_min;
        }
        if (q.raw_max > raw_max) {
            raw_max = q.raw_max;
        }
        raw_sum += q.raw_sum;
        raw_zeros += q.raw_zeros;
#endif

        if (leq_samples >= SAMPLES_LEQ) {
            double leq_rms = sqrt(leq_sum_sqr / leq_samples);
            double leq_db = MIC_OFFSET_DB + MIC_REF_DB + 20 * log10(leq_rms / mic_ref_ampl);
            bool low = !isfinite(leq_db) || (leq_db < MIC_NOISE_DB);
            if (!isfinite(leq_db)) {
                leq_db = 0.0;
            }

#if CONFIG_SPL_DIAGNOSTICS
            int64_t tl0 = esp_timer_get_time();
#endif
            ESP_LOGI(TAG, "L%seq(%.1fs) = %.1f %s%s%s", WEIGHTING_NAME, CONFIG_SPL_LEQ_PERIOD_MS / 1000.0, leq_db, DB_UNITS,
                     overload ? " [overload]" : "", low ? " [below noise floor]" : "");
#if CONFIG_SPL_DIAGNOSTICS
            double leq_z_db = MIC_OFFSET_DB + MIC_REF_DB + 20 * log10(sqrt(leq_sum_sqr_z / leq_samples) / mic_ref_ampl);
            ESP_LOGI(TAG, "diag: raw min=%ld max=%ld mean=%ld zeros=%lu/%lu, Z-weighted=%.1f dB, blocks=%u",
                     (long)raw_min, (long)raw_max, (long)(raw_sum / (int64_t)leq_samples),
                     (unsigned long)raw_zeros, (unsigned long)leq_samples, isfinite(leq_z_db) ? leq_z_db : 0.0,
                     (unsigned)(leq_samples / SAMPLES_SHORT));
            if (raw_zeros == leq_samples) {
                ESP_LOGW(TAG, "all I2S samples are 0: check mic power, SD/SCK/WS wiring (SCK=%d WS=%d SD=%d) and L/R pin",
                         CONFIG_SPL_I2S_SCK_GPIO, CONFIG_SPL_I2S_WS_GPIO, CONFIG_SPL_I2S_SD_GPIO);
            } else if (raw_min == raw_max) {
                ESP_LOGW(TAG, "I2S samples are constant (0x%08lx): data line stuck or floating, or wrong channel selected",
                         (unsigned long)raw_min);
            }
            log_us = esp_timer_get_time() - tl0;
#endif

            char json[128];
            snprintf(json, sizeof(json),
                     "{\"spl\":%.1f,\"weighting\":\"%s\",\"overload\":%s,\"below_noise_floor\":%s}",
                     leq_db, WEIGHTING_NAME, overload ? "true" : "false", low ? "true" : "false");
            /* Publish via MQTT if above threshold */
            if (leq_db >= spl_mqtt_threshold) {
                publish_sensor_string(mac_str, json);
            }

            leq_sum_sqr = 0;
            leq_samples = 0;
            overload = false;
#if CONFIG_SPL_DIAGNOSTICS
            leq_sum_sqr_z = 0;
            raw_min = INT32_MAX;
            raw_max = INT32_MIN;
            raw_sum = 0;
            raw_zeros = 0;
#endif
        }

#if CONFIG_SPL_DIAGNOSTICS
        uint32_t busy_us = (uint32_t)(esp_timer_get_time() - tc0);
        perf.calc_busy_us += busy_us;
        perf.calc_log_us += log_us;
        perf.calc_blocks++;
        if (busy_us > perf.calc_max_us) {
            perf.calc_max_us = busy_us;
        }
#endif
    }
}

/* ------------------------------------------------------------------------- */
/* Performance monitor                                                       */
/* ------------------------------------------------------------------------- */

#if CONFIG_SPL_DIAGNOSTICS

#if CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS && CONFIG_FREERTOS_USE_TRACE_FACILITY
#ifndef configRUN_TIME_COUNTER_TYPE
#define configRUN_TIME_COUNTER_TYPE uint32_t
#endif

#define MON_MAX_TASKS 32

/* Per-task CPU usage over the last monitor period, from the FreeRTOS run-time statistics */
static void log_cpu_usage(void)
{
    static struct {
        TaskHandle_t handle;
        uint32_t run_time;
    } prev[MON_MAX_TASKS];
    static uint32_t prev_total;
    static size_t prev_count;

    UBaseType_t n = uxTaskGetNumberOfTasks();
    if (n > MON_MAX_TASKS) {
        n = MON_MAX_TASKS;
    }
    TaskStatus_t *st = malloc(n * sizeof(TaskStatus_t));
    if (st == NULL) {
        return;
    }

    configRUN_TIME_COUNTER_TYPE total = 0;
    n = uxTaskGetSystemState(st, n, &total);
    uint32_t total_delta = (uint32_t)total - prev_total;

    char line[256];
    size_t len = 0;
    float idle_pct = 0.0f;
    size_t count = 0;
    typeof(prev) cur;

    for (UBaseType_t i = 0; i < n; i++) {
        uint32_t run = (uint32_t)st[i].ulRunTimeCounter;
        uint32_t last = 0;
        for (size_t j = 0; j < prev_count; j++) {
            if (prev[j].handle == st[i].xHandle) {
                last = prev[j].run_time;
                break;
            }
        }
        cur[count].handle = st[i].xHandle;
        cur[count].run_time = run;
        count++;

        float pct = total_delta ? 100.0f * (float)(run - last) / (float)total_delta : 0.0f;
        if (strncmp(st[i].pcTaskName, "IDLE", 4) == 0) {
            idle_pct += pct;
        }
        if (pct >= 0.5f && len < sizeof(line)) {
            len += snprintf(line + len, sizeof(line) - len, " %s=%.1f%%", st[i].pcTaskName, pct);
        }
    }
    memcpy(prev, cur, sizeof(prev));
    prev_count = count;
    prev_total = (uint32_t)total;
    free(st);

    ESP_LOGI(TAG, "cpu: total=%.1f%% (idle=%.1f%%) |%s", 100.0f - idle_pct, idle_pct, line);
}
#endif /* run-time stats */

static void perf_monitor_task(void *arg)
{
    perf_stats_t prev = {0};
    int64_t prev_time = esp_timer_get_time();

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(MON_PERIOD_MS));

        perf_stats_t cur = perf;
        uint32_t rd_max = perf.rd_max_chunk_us;
        uint32_t calc_max = perf.calc_max_us;
        perf.rd_max_chunk_us = 0;
        perf.calc_max_us = 0;
        uint32_t q_max = perf.queue_max;
        perf.queue_max = 0;

        int64_t now = esp_timer_get_time();
        double wall = (double)(now - prev_time);
        prev_time = now;

        uint32_t chunks = cur.rd_chunks - prev.rd_chunks;
        uint32_t blocks = cur.calc_blocks - prev.calc_blocks;
        double wait = (double)(cur.rd_wait_us - prev.rd_wait_us);
        double stats = (double)(cur.rd_stats_us - prev.rd_stats_us);
        double conv = (double)(cur.rd_convert_us - prev.rd_convert_us);
        double eq = (double)(cur.rd_eq_us - prev.rd_eq_us);
        double wgt = (double)(cur.rd_weight_us - prev.rd_weight_us);
        double rd_proc = stats + conv + eq + wgt;
        double c_busy = (double)(cur.calc_busy_us - prev.calc_busy_us);
        double c_log = (double)(cur.calc_log_us - prev.calc_log_us);

        /* Real-time budget: processing time per chunk compared with the duration of the chunk */
        double per_chunk = chunks ? rd_proc / chunks : 0.0;
        ESP_LOGI(TAG, "perf[%.1fs]: chunks=%u (real-time=%.1f/s) processing/chunk=%.2f ms of %.2f ms budget (%.0f%%), max=%.2f ms, overruns=%u",
                 wall / 1e6, (unsigned)chunks, chunks * 1e6 / wall, per_chunk / 1e3, CHUNK_BUDGET_US / 1e3,
                 100.0 * per_chunk / CHUNK_BUDGET_US, rd_max / 1e3,
                 (unsigned)(cur.rd_overruns - prev.rd_overruns));
        ESP_LOGI(TAG, "perf: reader wall%%: wait=%.1f stats=%.1f convert=%.1f eq=%.1f weight=%.1f | per chunk ms: stats=%.2f convert=%.2f eq=%.2f weight=%.2f",
                 100.0 * wait / wall, 100.0 * stats / wall, 100.0 * conv / wall, 100.0 * eq / wall,
                 100.0 * wgt / wall,
                 chunks ? stats / chunks / 1e3 : 0.0, chunks ? conv / chunks / 1e3 : 0.0,
                 chunks ? eq / chunks / 1e3 : 0.0, chunks ? wgt / chunks / 1e3 : 0.0);
        ESP_LOGI(TAG, "perf: calc blocks=%u busy=%.1f%% wall (avg %.2f ms, max %.2f ms, logging %.2f ms avg) | queue max=%u drops=%u",
                 (unsigned)blocks, 100.0 * c_busy / wall, blocks ? c_busy / blocks / 1e3 : 0.0, calc_max / 1e3,
                 blocks ? c_log / blocks / 1e3 : 0.0, (unsigned)q_max, (unsigned)(cur.drops - prev.drops));
        ESP_LOGI(TAG, "perf: stack free min: reader=%u calc=%u monitor=%u bytes, heap free=%u",
                 (unsigned)(reader_task_handle ? uxTaskGetStackHighWaterMark(reader_task_handle) : 0),
                 (unsigned)(calc_task_handle ? uxTaskGetStackHighWaterMark(calc_task_handle) : 0),
                 (unsigned)uxTaskGetStackHighWaterMark(NULL), (unsigned)esp_get_free_heap_size());
#if CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS && CONFIG_FREERTOS_USE_TRACE_FACILITY
        log_cpu_usage();
#endif

        prev = cur;
    }
}

#endif /* CONFIG_SPL_DIAGNOSTICS */

esp_err_t spl_meter_start(void)
{
    spl_threshold_load();
    sos_filter_init(&mic_equalizer, MIC_EQ_GAIN, MIC_EQ_SOS, SOS_COUNT(MIC_EQ_SOS));
#if CONFIG_SPL_WEIGHTING_A
    sos_filter_init(&weighting, A_WEIGHTING_GAIN, A_WEIGHTING_SOS, SOS_COUNT(A_WEIGHTING_SOS));
#elif CONFIG_SPL_WEIGHTING_C
    sos_filter_init(&weighting, C_WEIGHTING_GAIN, C_WEIGHTING_SOS, SOS_COUNT(C_WEIGHTING_SOS));
#else
    sos_filter_init(&weighting, 1.0f, NULL, 0);
#endif

    samples_queue = xQueueCreate(8, sizeof(sum_queue_t));
    ESP_RETURN_ON_FALSE(samples_queue != NULL, ESP_ERR_NO_MEM, TAG, "queue creation failed");

    ESP_RETURN_ON_ERROR(mic_i2s_init(), TAG, "I2S init failed");

    ESP_RETURN_ON_FALSE(xTaskCreate(spl_calc_task, "spl_calc", CALC_TASK_STACK, NULL, CALC_TASK_PRI, &calc_task_handle) == pdPASS,
                        ESP_ERR_NO_MEM, TAG, "spl_calc task creation failed");
    ESP_RETURN_ON_FALSE(xTaskCreate(mic_i2s_reader_task, "mic_i2s_reader", I2S_TASK_STACK, NULL, I2S_TASK_PRI, &reader_task_handle) == pdPASS,
                        ESP_ERR_NO_MEM, TAG, "mic_i2s_reader task creation failed");
#if CONFIG_SPL_DIAGNOSTICS
    ESP_RETURN_ON_FALSE(xTaskCreate(perf_monitor_task, "spl_mon", MON_TASK_STACK, NULL, MON_TASK_PRI, NULL) == pdPASS,
                        ESP_ERR_NO_MEM, TAG, "spl_mon task creation failed");
#endif

    ESP_LOGI(TAG, "SPL meter started (%s weighting, Leq period %.1f s)", WEIGHTING_NAME, CONFIG_SPL_LEQ_PERIOD_MS / 1000.0);
    return ESP_OK;
}

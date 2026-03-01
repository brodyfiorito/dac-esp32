// Brody Fiorito 2026
// Lissajous Box - main.c
// Generates stereo sine waves with adjustable frequency ratio and phase offset
// for Lissajous figure display. 
// Also supports USB audio passthrough mode for music visualization.

#include <stdio.h>
#include <driver/uart.h>
#include <string.h>
#include <driver/gpio.h>
#include <driver/i2s_std.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <esp_adc/adc_oneshot.h>
#include <math.h>
#include <usb_device_uac.h>
#include <freertos/ringbuf.h>

// Hardware Pins

#define LRCK_PIN GPIO_NUM_35
#define DOUT_PIN GPIO_NUM_36
#define BCK_PIN GPIO_NUM_37
#define SCK_PIN GPIO_NUM_38

#define PHASE_PIN GPIO_NUM_4
#define FREQ_PIN GPIO_NUM_5
#define PHASE_ADC_CHANNEL ADC_CHANNEL_3
#define FREQ_ADC_CHANNEL ADC_CHANNEL_4
#define MODE_SWITCH_PIN GPIO_NUM_9

// Constants
#define PI 3.14159265358979
#define UAC_RX_BUFSIZE 4096
#define ADC_MV_MAX 3200.0f

// Global State
static RingbufHandle_t audio_ringbuf = NULL;

static i2s_chan_handle_t tx_chan;

volatile float freqRatio;   // Shared between i2s_task and adc_task. Volatile prevents the compiler
volatile float phaseDiff;   // from optimizing reads into a register, ensuring the latest value is always used.

typedef struct {
    int last_raw;
    float smooth;
} adc_ctrl_t;

adc_ctrl_t freq_ctrl = { .last_raw = 0, .smooth = 0 };
adc_ctrl_t phase_ctrl = { .last_raw = 0, .smooth = 0 };

typedef enum {
    MODE_LISSAJOUS = 0,
    MODE_AUDIO_INPUT = 1
} system_state_t;

TaskHandle_t adc_sampling_handle = NULL;
TaskHandle_t audio_input_handle = NULL;
TaskHandle_t i2s_handle = NULL;

static system_state_t current_mode = MODE_LISSAJOUS;
static system_state_t requested_mode = MODE_LISSAJOUS;

// Function prototypes
void adc_task(void *arg);
void i2s_task(void *arg);
void audio_input_task(void *arg);
int adc_hysteresis(adc_ctrl_t *c, int);
float adc_smooth(adc_ctrl_t *c, int);
float get_ratio(float);
float get_phase_diff(float);
static esp_err_t uac_output_callback(uint8_t *buf, size_t len, void *cb_ctx);
void enter_mode(system_state_t);
static system_state_t read_mode_switch(void);


void i2s_task (void *arg) {

    i2s_chan_config_t chan_config = I2S_CHANNEL_DEFAULT_CONFIG(
        I2S_NUM_0,
        I2S_ROLE_MASTER
    );

    i2s_std_config_t i2s_config = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(44100),

        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_16BIT,
            I2S_SLOT_MODE_STEREO
        ),

        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = BCK_PIN,
            .ws = LRCK_PIN,
            .dout = DOUT_PIN,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
            
        },
    };

    i2s_new_channel(&chan_config, &tx_chan, NULL);
    i2s_channel_init_std_mode(tx_chan, &i2s_config);
    i2s_channel_enable(tx_chan);

    const float base_freq = 200.0f;
    float phaseL = 0.0f;
    float phaseR = 0.0f;
    const float sample_rate = 44100.0f;
    const int BUFFER_SIZE = 256;
    int16_t buffer[BUFFER_SIZE * 2];

    while (1) {

        if (current_mode == MODE_LISSAJOUS) {

            float fL = base_freq;
            float fR = base_freq * freqRatio;

            for (int i = 0; i < BUFFER_SIZE; i++) {

                float sampleL = sinf(phaseL);
                float sampleR = sinf(phaseR + phaseDiff);

                buffer[i*2 + 0] = (int16_t)(sampleL * 0x7FFF);
                buffer[i*2 + 1] = (int16_t)(sampleR * 0x7FFF);

                phaseL += 2.0f * PI * fL / sample_rate;
                phaseR += 2.0f * PI * fR / sample_rate;

                if (phaseL >= 2.0f * PI) phaseL -= 2.0f * PI;
                if (phaseR >= 2.0f * PI) phaseR -= 2.0f * PI;

            }

            size_t bytes_written;
            i2s_channel_write(tx_chan, buffer, sizeof(buffer), &bytes_written, portMAX_DELAY);


        } else if (current_mode == MODE_AUDIO_INPUT) {

            if (audio_ringbuf == NULL) {
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }

            size_t bytes_available;
            uint8_t *data = (uint8_t *)xRingbufferReceiveUpTo(
                audio_ringbuf,
                &bytes_available,
                pdMS_TO_TICKS(20),  // Block up to 20ms waiting for data; sizeof(buffer) caps the read to our
                sizeof(buffer)      // output buffer size. If no data arrives we loop and retry rather than stall.
            );

            if (data && bytes_available > 0) {
                size_t bytes_written;
                i2s_channel_write(tx_chan, data, bytes_available, &bytes_written, portMAX_DELAY);
                vRingbufferReturnItem(audio_ringbuf, data);
            }

        } else {
            vTaskDelay(1);
        }
    }
}

void adc_task(void *arg) {


    // Configure ADC using oneshot
    adc_oneshot_unit_handle_t adc_handle;

    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1
    };

    adc_oneshot_new_unit(&init_config, &adc_handle);

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };

    adc_oneshot_config_channel(adc_handle, FREQ_ADC_CHANNEL, &chan_cfg);

    adc_oneshot_config_channel(adc_handle, PHASE_ADC_CHANNEL, &chan_cfg);

    adc_cali_handle_t cali_handle;

    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };

    adc_cali_create_scheme_curve_fitting(&cali_cfg, &cali_handle);


    while (1) {

        int raw_freq, mv_freq;
        int raw_phase, mv_phase;

        adc_oneshot_read(adc_handle, FREQ_ADC_CHANNEL, &raw_freq);
        adc_oneshot_read(adc_handle, PHASE_ADC_CHANNEL, &raw_phase);

        adc_cali_raw_to_voltage(cali_handle, raw_freq, &mv_freq); // Calibrates raw ADC values to mV
        adc_cali_raw_to_voltage(cali_handle, raw_phase, &mv_phase);

        mv_freq = adc_hysteresis(&freq_ctrl, mv_freq);
        mv_phase = adc_hysteresis(&phase_ctrl, mv_phase);

        float freq_norm = adc_smooth(&freq_ctrl, mv_freq) / ADC_MV_MAX;     // Normalizes from 0-1
        float phase_norm = adc_smooth(&phase_ctrl, mv_phase) / ADC_MV_MAX;

        freqRatio = get_ratio(freq_norm);
        phaseDiff = get_phase_diff(phase_norm);

        vTaskDelay(pdMS_TO_TICKS(10));
    }


}

void audio_input_task(void *arg) {

    // Make ring buffer, computer puts data in back while DAC picks up data from front
    audio_ringbuf = xRingbufferCreate(UAC_RX_BUFSIZE, RINGBUF_TYPE_BYTEBUF);

    // Configure and start UAC
    uac_device_config_t uac_cfg = {
        .output_cb = uac_output_callback,
        .input_cb = NULL,
        .set_mute_cb = NULL,
        .set_volume_cb = NULL,
        .cb_ctx = NULL,
    };

    uac_device_init(&uac_cfg);

    // Task stays alive to keep UAC initialized; audio data is handled via uac_output_callback
    while (1) {
        vTaskDelay(portMAX_DELAY);
    }
}

void app_main(void)
{

    // Configuring GPIO
    gpio_set_direction(LRCK_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(DOUT_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(BCK_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(SCK_PIN, GPIO_MODE_OUTPUT);

    gpio_set_direction(PHASE_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(FREQ_PIN, GPIO_MODE_INPUT);

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << MODE_SWITCH_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE

    };
    gpio_config(&io_conf);

    // Start persistent i2s task and enter into default mode
    xTaskCreate(
            i2s_task,
            "I2S_task",
            8192,
            NULL,
            5,
            &i2s_handle
        ); 

    enter_mode(MODE_LISSAJOUS);

    while (1) {

        requested_mode = read_mode_switch();

        if (requested_mode != current_mode) {
            enter_mode(requested_mode);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

////////////////// Helper Functions //////////////////

#define ADC_HYST 10     // Amount of millivolts in order to cause change in output value

int adc_hysteresis(adc_ctrl_t *c, int raw)
{
    int delta = raw - c->last_raw;

    if (delta > ADC_HYST) {
        c->last_raw += ADC_HYST;
    } else if (delta < -ADC_HYST) {
        c->last_raw -= ADC_HYST;
    }

    return c->last_raw;
}

float adc_smooth(adc_ctrl_t *c, int val) {

    c->smooth = 0.05f * c->smooth + 0.95f * val; // 5% old, 95% new creates an exponential moving average
    return c->smooth;

}

// Translate millivolts to ratio between frequencies
float get_ratio(float freq_norm) {

    if (freq_norm < 1.0f/8.0f) return 1.0f;         // 1:1 ratio
    if (freq_norm < 2.0f/8.0f) return 2.0f;         // 2:1 ratio
    if (freq_norm < 3.0f/8.0f) return 3.0f;         // 3:1 ratio
    if (freq_norm < 4.0f/8.0f) return 4.0f;         // 4:1 ratio
    if (freq_norm < 5.0f/8.0f) return 0.5f;         // 1:2 ratio
    if (freq_norm < 6.0f/8.0f) return 1.5f;         // 3:2 ratio
    if (freq_norm < 7.0f/8.0f) return 4.0f/3.0f;    // 4:3 ratio
    return 6.0f/5.0f;                               // 6:5 ratio
}

// Translate millivolts to phase difference
float get_phase_diff(float phase_norm) {

    if (phase_norm < 1.0f/8.0f) return 0;
    if (phase_norm < 2.0f/8.0f) return PI/8.0f;
    if (phase_norm < 3.0f/8.0f) return PI/4.0f;
    if (phase_norm < 4.0f/8.0f) return PI*3.0f/8.0f;
    if (phase_norm < 5.0f/8.0f) return PI/2.0f;
    if (phase_norm < 6.0f/8.0f) return PI*5.0f/8.0f;
    if (phase_norm < 7.0f/8.0f) return PI*3.0f/4.0f;
    if (phase_norm < 1.0f) return PI*7.0f/8.0f;
    return PI;

}

static esp_err_t uac_output_callback(uint8_t *buf, size_t len, void *arg) {

    if (audio_ringbuf) {
        xRingbufferSend(audio_ringbuf, buf, len, 0); // Timeout of 0 drops data if the buffer is full
    }

    return ESP_OK;
}


////////////////// State Machine Functions //////////////////

void enter_mode(system_state_t mode) {



    // Cleans up tasks that are no longer in use, lowers task load before transitioning to new mode
    if (adc_sampling_handle) {
        vTaskDelete(adc_sampling_handle);
        adc_sampling_handle = NULL;
    }

    if (audio_input_handle) {
        vTaskDelete(audio_input_handle);
        audio_input_handle = NULL;
    }

    if (audio_ringbuf) {
        vRingbufferDelete(audio_ringbuf);
        audio_ringbuf = NULL;
    }

    switch (mode) {

        case MODE_LISSAJOUS:

            xTaskCreate(
            adc_task,
            "ADC_Sampling",
            4096,
            NULL,
            3,
            &adc_sampling_handle
            ); 

            break;

        case MODE_AUDIO_INPUT:

            xTaskCreate(
            audio_input_task,
            "Audio_Input",
            8192,
            NULL,
            5,
            &audio_input_handle
            );

            break;
    }
    current_mode = mode;
}

// Reads mode switch on board to determine requested mode
static system_state_t read_mode_switch(void) {
    return gpio_get_level(MODE_SWITCH_PIN) ? MODE_AUDIO_INPUT : MODE_LISSAJOUS;
}
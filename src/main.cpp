#include <Arduino.h>
#include <driver/i2s.h>
#include <math.h>

constexpr int I2S_SCK = 26;
constexpr int I2S_WS  = 25;
constexpr int I2S_SD  = 33;

constexpr i2s_port_t I2S_PORT = I2S_NUM_0;
constexpr int SAMPLE_RATE = 16000;
constexpr int BUFFER_SIZE = 512;

int32_t samples[BUFFER_SIZE];

void setup() {
    Serial.begin(115200);
    delay(1000);

    i2s_config_t config = {};
    config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
    config.sample_rate = SAMPLE_RATE;
    config.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;
    config.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
    config.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    config.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
    config.dma_buf_count = 4;
    config.dma_buf_len = BUFFER_SIZE;
    config.use_apll = false;

    i2s_pin_config_t pins = {};
    pins.mck_io_num = I2S_PIN_NO_CHANGE;
    pins.bck_io_num = I2S_SCK;
    pins.ws_io_num = I2S_WS;
    pins.data_out_num = I2S_PIN_NO_CHANGE;
    pins.data_in_num = I2S_SD;

    ESP_ERROR_CHECK(
        i2s_driver_install(I2S_PORT, &config, 0, nullptr)
    );

    ESP_ERROR_CHECK(
        i2s_set_pin(I2S_PORT, &pins)
    );

    Serial.println("JingleSync microphone initialized!");
}

void loop() {
    size_t bytesRead = 0;

    esp_err_t result = i2s_read(
        I2S_PORT,
        samples,
        sizeof(samples),
        &bytesRead,
        portMAX_DELAY
    );

    if (result != ESP_OK || bytesRead == 0) {
        return;
    }

    int count = bytesRead / sizeof(int32_t);
    double sumSquares = 0;

    for (int i = 0; i < count; i++) {
        int32_t sample = samples[i] >> 8;
        sumSquares += (double)sample * sample;
    }

    double rms = sqrt(sumSquares / count);

    Serial.printf("Audio level: %.2f\n", rms);
}
#include <Arduino.h>
#include <driver/i2s.h>
#include <arduinoFFT.h>
#include <math.h>

// Microphone configuration
constexpr int I2S_SCK = 26;
constexpr int I2S_WS  = 25;
constexpr int I2S_SD  = 33;

constexpr i2s_port_t I2S_PORT = I2S_NUM_0;

constexpr int SAMPLE_RATE = 16000;
constexpr int FFT_SIZE = 512;
constexpr int HOP_SIZE = 256;

// Detection configuration
constexpr float SENSITIVITY = 2.5f;
constexpr float MIN_FLUX = 0.24f;
constexpr float MIN_RMS = 20000.0f;

constexpr float ADAPTATION = 0.03f;
constexpr unsigned long MIN_BEAT_INTERVAL_MS = 60;

// Frequency range: approximately 125 Hz - 6000 Hz
constexpr int FIRST_BIN = 4;
constexpr int LAST_BIN = 192;

int32_t samples[HOP_SIZE];

float audioWindow[FFT_SIZE] = {};
float vReal[FFT_SIZE];
float vImag[FFT_SIZE];
float hannWindow[FFT_SIZE];

float previousSpectrum[FFT_SIZE / 2] = {};

ArduinoFFT<float> FFT(
    vReal,
    vImag,
    FFT_SIZE,
    SAMPLE_RATE
);

float averageFlux = 0;
float fluxDeviation = 0;

float previousFlux = 0;
float olderFlux = 0;
float previousThreshold = 0;
float previousRms = 0;

unsigned long lastBeatTime = 0;
int frameCount = 0;

void setup() {
    Serial.begin(115200);
    delay(1000);

    i2s_config_t config = {};

    config.mode = (i2s_mode_t)(
        I2S_MODE_MASTER | I2S_MODE_RX
    );

    config.sample_rate = SAMPLE_RATE;
    config.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;
    config.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
    config.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    config.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
    config.dma_buf_count = 4;
    config.dma_buf_len = HOP_SIZE;
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

    // Precompute Hann window
    for (int i = 0; i < FFT_SIZE; i++) {
        hannWindow[i] = 0.5f * (
            1.0f - cosf(
                2.0f * PI * i / (FFT_SIZE - 1)
            )
        );
    }

    Serial.println("JingleSync FFT initialized!");
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

    if (result != ESP_OK ||
        bytesRead != sizeof(samples)) {
        return;
    }

    // Retain the previous half-window
    memmove(
        audioWindow,
        audioWindow + HOP_SIZE,
        (FFT_SIZE - HOP_SIZE) * sizeof(float)
    );

    // Append newly captured audio
    for (int i = 0; i < HOP_SIZE; i++) {
        audioWindow[FFT_SIZE - HOP_SIZE + i] =
            (float)(samples[i] >> 8);
    }

    frameCount++;

    // Remove local DC offset
    float mean = 0;

    for (int i = 0; i < FFT_SIZE; i++) {
        mean += audioWindow[i];
    }

    mean /= FFT_SIZE;

    float sumSquares = 0;

    for (int i = 0; i < FFT_SIZE; i++) {
        float centered = audioWindow[i] - mean;

        sumSquares += centered * centered;

        vReal[i] = centered * hannWindow[i];
        vImag[i] = 0;
    }

    float rms = sqrtf(sumSquares / FFT_SIZE);

    // Convert time-domain audio to frequency domain
    FFT.compute(FFTDirection::Forward);
    FFT.complexToMagnitude();

    // Calculate logarithmic spectral flux
    float flux = 0;

    for (int i = FIRST_BIN; i <= LAST_BIN; i++) {
        float magnitude = vReal[i] / FFT_SIZE;

        float compressed = log1pf(magnitude / 1000.0f);

        float change = compressed - previousSpectrum[i];

        if (change > 0) {
            flux += change;
        }

        previousSpectrum[i] = compressed;
    }

    flux /= (LAST_BIN - FIRST_BIN + 1);

    // Adaptive onset threshold
    float threshold = fmaxf(
        MIN_FLUX,
        averageFlux + SENSITIVITY * fluxDeviation
    );

    unsigned long now = millis();

    // Detect a local maximum in spectral flux
    bool isPeak =
        previousFlux > olderFlux &&
        previousFlux >= flux;

    bool isOnset =
        isPeak &&
        previousFlux > previousThreshold &&
        previousRms > MIN_RMS;

    // Allow one second for initial adaptation
    if (frameCount > 63 &&
        isOnset &&
        now - lastBeatTime >= MIN_BEAT_INTERVAL_MS) {

        Serial.printf(
            "BEAT! Time: %lu | RMS: %.0f"
            " | Flux: %.4f | Threshold: %.4f\n",
            now,
            previousRms,
            previousFlux,
            previousThreshold
        );

        lastBeatTime = now;
    }

    // Update adaptive statistics
    float deviation = fabsf(flux - averageFlux);

    averageFlux += ADAPTATION *
        (flux - averageFlux);

    fluxDeviation += ADAPTATION *
        (deviation - fluxDeviation);

    olderFlux = previousFlux;
    previousFlux = flux;
    previousThreshold = threshold;
    previousRms = rms;
}
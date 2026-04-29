/******************************************************************************
 * @file    esp32_bluetooth.ino
 * @author  Junkai Li
 * @date    2026-04-28
 * @brief   ESP32 Bluetooth A2DP sink + UART bridge
 * @mail    ssyjl10@outlook.com
 *
 * Georgia Institute of Technology
 * GitHub: https://github.com/KaiLyee/bluetooth-dsp-speaker
 ******************************************************************************/
#include "BluetoothA2DPSink.h"
#include "driver/i2s.h"

BluetoothA2DPSink a2dp_sink;//bluetooth receiver

void audio_data_callback(const uint8_t *data, uint32_t len) {
    size_t bytes_written = 0;
    const int16_t *samples = (const int16_t *)data;
    int sample_count = len / 2;
    
    int i = 0;
    while (i < sample_count) {
        int32_t chunk[128];
        int c = 0;
        while (c < 128 && i < sample_count) {
            chunk[c++] = (int32_t)samples[i++] * 65536;//->16
        }
        i2s_write(I2S_NUM_0, (uint8_t *)chunk, c * 4, &bytes_written, portMAX_DELAY);//send to stm32 portMAX_DELAY wait if the buffer area is full
    }
}

/* UART bridge task - runs independently on Core 0, not affected by audio */
void uartBridgeTask(void *param) {
    uint8_t buf[64];
    for (;;) {
        // PC -> STM32
        int len = Serial.available();//detect data from USB
        if (len > 0) {
            if (len > 64) len = 64;
            Serial.readBytes(buf, len);
            Serial2.write(buf, len);//transfer data to STM32
        }
        // STM32 -> PC
        len = Serial2.available();//detect data from GPIO
        if (len > 0) {
            if (len > 64) len = 64;
            Serial2.readBytes(buf, len);
            Serial.write(buf, len);//transfer data to PC
        }
        vTaskDelay(1);//Run the loop once per 1ms
    }
}

void setup() {
    Serial.begin(38400);                              // USB to PC
    //Serial2.begin(9600, SERIAL_8N1, 32, 33);         // UART to STM32: RX=GPIO32, TX=GPIO33
    Serial2.begin(38400, SERIAL_8N1, 4, 2);  // RX=GPIO4, TX=GPIO2
    i2s_config_t i2s_cfg = {};
    i2s_cfg.mode = (i2s_mode_t)(I2S_MODE_SLAVE | I2S_MODE_TX);
    i2s_cfg.sample_rate = 44100;
    i2s_cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;
    i2s_cfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
    i2s_cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    i2s_cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
    i2s_cfg.dma_buf_count = 8;
    i2s_cfg.dma_buf_len = 256;
    i2s_cfg.use_apll = false;
    i2s_cfg.tx_desc_auto_clear = true;

    i2s_driver_install(I2S_NUM_0, &i2s_cfg, 0, NULL);

    i2s_pin_config_t pins = {};
    pins.bck_io_num   = 26;
    pins.ws_io_num    = 25;
    pins.data_out_num = 22;
    pins.data_in_num  = -1;
    i2s_set_pin(I2S_NUM_0, &pins);

    a2dp_sink.set_stream_reader(audio_data_callback, false);
    a2dp_sink.start("Kai_Speaker");

    // Start UART bridge on Core 0 (audio runs on Core 1)
    xTaskCreatePinnedToCore(uartBridgeTask, "uart_bridge", 4096, NULL, 1, NULL, 0);

    Serial.println("Bluetooth A2DP + UART Bridge started");
}

void loop() {
    delay(1000);
}

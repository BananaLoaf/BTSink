#ifndef BTAUDIO_H
#define BTAUDIO_H

#endif


#include "Arduino.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"
#include "driver/i2s.h"


class BTSink {
    private:
        const char *_device_name;
        static unsigned char *_addr;

        static bool _conn_stat;
        static esp_a2d_audio_state_t _audio_state;

        static uint32_t _sampleRate;
        static float _vol;

    public:
        static String title;
        static String artist;
        static String album;
        static String genre;


    private:
        // Callbacks
        static void a2dp_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param);
        static void avrc_cb(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param);
        static void i2s_cb(const uint8_t *data, uint32_t len);

    public:
        BTSink(const char *device_name) {_device_name = device_name;};

        // Bluetooth
        void begin();
        void end();
        void set_data_callback(void (*data_callback)(const uint8_t *data, uint32_t len));
        bool is_connected() {return _conn_stat;};

        // I2S
        void i2s(int bck, int dout, int ws);
        void set_volume(float vol);

        // Metadata
        void update_metadata();
};

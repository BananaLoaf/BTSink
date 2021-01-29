#include "btAudio.h"


const char *_audio_state_str[3]{"ESP_A2D_AUDIO_STATE_REMOTE_SUSPEND",
                                "ESP_A2D_AUDIO_STATE_STOPPED",
                                "ESP_A2D_AUDIO_STATE_STARTED"};

// Statics
unsigned char *BTSink::_addr = 0;

bool BTSink::_conn_stat = false;
esp_a2d_audio_state_t BTSink::_audio_state = ESP_A2D_AUDIO_STATE_REMOTE_SUSPEND;

uint32_t BTSink::_sampleRate = 44100;
float BTSink::_vol = 0.0;

String BTSink::title = "";
String BTSink::artist = "";
String BTSink::album = "";
String BTSink::genre = "";


// Callbacks
void BTSink::a2dp_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t*param){
    esp_a2d_cb_param_t *a2d = (esp_a2d_cb_param_t *)(param);

    switch (event) {
    case ESP_A2D_CONNECTION_STATE_EVT:{
        _addr = a2d->conn_stat.remote_bda;
        ESP_LOGE(BT_AV_TAG, "A2DP device: %08X", a2d->conn_stat.remote_bda);
        break;
    }
    case ESP_A2D_AUDIO_CFG_EVT: {
        ESP_LOGI(BT_AV_TAG, "A2DP audio stream configuration, codec type %d", a2d->audio_cfg.mcc.type);

        // For now only SBC stream is supported
        if (a2d->audio_cfg.mcc.type == ESP_A2D_MCT_SBC) {
            _sampleRate = 16000;
            char oct0 = a2d->audio_cfg.mcc.cie.sbc[0];
            if (oct0 & (0x01 << 6)) {
                _sampleRate = 32000;
            }
            else if (oct0 & (0x01 << 5)) {
                _sampleRate = 44100;
            }
            else if (oct0 & (0x01 << 4)) {
                _sampleRate = 48000;
            }
            ESP_LOGI(BT_AV_TAG, "Configure audio player %x-%x-%x-%x",
                     a2d->audio_cfg.mcc.cie.sbc[0],
                     a2d->audio_cfg.mcc.cie.sbc[1],
                     a2d->audio_cfg.mcc.cie.sbc[2],
                     a2d->audio_cfg.mcc.cie.sbc[3]);
            if (i2s_set_sample_rates(I2S_NUM_0, _sampleRate)==ESP_OK) {
                ESP_LOGI(BT_AV_TAG, "Audio player configured, sample rate=%d", _sampleRate);
            }
        }
        break;
    }
    case ESP_A2D_AUDIO_STATE_EVT: {
        _audio_state = a2d->audio_stat.state;
        ESP_LOGE(BT_AV_TAG, "Audio state set to %s", _audio_state_str[_audio_state]);
        break;
    }
    default:
        ESP_LOGI(BT_AV_TAG, "A2DP invalid cb event: %d", event);
        break;
    }
}
void BTSink::avrc_cb(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param) {
    esp_avrc_ct_cb_param_t *rc = (esp_avrc_ct_cb_param_t *)(param);
    char *attr_text;
    String mystr;

    switch (event) {
    case ESP_AVRC_CT_METADATA_RSP_EVT: {
        attr_text = (char *) malloc (rc->meta_rsp.attr_length + 1);
        memcpy(attr_text, rc->meta_rsp.attr_text, rc->meta_rsp.attr_length);
        attr_text[rc->meta_rsp.attr_length] = 0;
        mystr = String(attr_text);

        switch (rc->meta_rsp.attr_id) {
            case ESP_AVRC_MD_ATTR_TITLE:
                //Serial.print("Title: ");
                //Serial.println(mystr);
                title= mystr;
                break;
            case ESP_AVRC_MD_ATTR_ARTIST:
                //Serial.print("Artist: ");
                //Serial.println(mystr);
                artist= mystr;
                break;
            case ESP_AVRC_MD_ATTR_ALBUM:
                //Serial.print("Album: ");
                //Serial.println(mystr);
                album= mystr;
                break;
            case ESP_AVRC_MD_ATTR_GENRE:
                //Serial.print("Genre: ");
                //Serial.println(mystr);
                genre= mystr;
                break;
        }
        free(attr_text);
        break;
    }
    case ESP_AVRC_CT_CONNECTION_STATE_EVT: {
        _conn_stat = rc->conn_stat.connected;
        ESP_LOGE(BT_RC_CT_TAG, "Connection state set to %s", _conn_stat ? "true" : "false");

        ESP_LOGE(BT_RC_CT_TAG, "Scan mode set to %s",
                 _conn_stat ? "ESP_BT_SCAN_MODE_CONNECTABLE" : "ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE");
        if (_conn_stat) {
            esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE);
        }
        else {
            esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE);
        }
        break;
    }
    default:
        ESP_LOGE(BT_RC_CT_TAG, "unhandled AVRC event: %d", event);
        break;
    }
}
void BTSink::i2s_cb(const uint8_t *data, uint32_t len) {
    size_t i2s_bytes_write = 0;
    int16_t* data16 = (int16_t*) data; // PlayData doesn't want const
    int16_t fy[2];
    float temp;

    int jump = 4; // How many bytes at a time get sent to buffer
    int  n = len/jump; // Number of byte chunks
    for (int i=0; i<n; i++) {
        // Left channel
        fy[0] = *data16;
        data16++;

        // Right channel
        fy[1] = *data16;
        data16++;

        i2s_write(I2S_NUM_0, fy, jump, &i2s_bytes_write, 100);
    }
}

// Bluetooth
void BTSink::begin() {
    // Bluetooth
    btStart();
    esp_bluedroid_init();
    esp_bluedroid_enable();
    esp_bt_dev_set_device_name(_device_name);

    // A2DP
    esp_a2d_sink_init();
    esp_a2d_register_callback(a2dp_cb);

    // AVRCP
    esp_avrc_ct_init();
    esp_avrc_ct_register_callback(avrc_cb);

    // Set scan mode
    esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE);
}
void BTSink::end() {
  esp_a2d_sink_deinit();
  esp_bluedroid_disable();
  esp_bluedroid_deinit();
  btStop();  
}
void BTSink::set_data_callback(void (*data_callback)(const uint8_t *data, uint32_t len) ) {
    esp_a2d_sink_register_data_callback(data_callback);
}

// I2S
void BTSink::i2s(int bck, int dout, int ws) {
    // I2S configuration
    static const i2s_config_t i2s_config = {
        .mode = static_cast <i2s_mode_t> (I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate = _sampleRate,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = static_cast <i2s_comm_format_t> (I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8, // 3
        .dma_buf_len = 64, // 600
        .use_apll = false,
        .tx_desc_auto_clear = true
    };

    // I2S pinout
    static const i2s_pin_config_t pin_config = {
        .bck_io_num = bck, // 26
        .ws_io_num = ws, // 27
        .data_out_num = dout, // 25
        .data_in_num = I2S_PIN_NO_CHANGE
    };

    i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM_0, &pin_config);
    esp_a2d_sink_register_data_callback(i2s_cb);
}
void BTSink::set_volume(float vol) {
	_vol = constrain(vol,0,1);
}

// Metadata
void BTSink::update_metadata() {
    uint8_t attr_mask = ESP_AVRC_MD_ATTR_TITLE | ESP_AVRC_MD_ATTR_ARTIST | ESP_AVRC_MD_ATTR_ALBUM | ESP_AVRC_MD_ATTR_GENRE;
    esp_avrc_ct_send_metadata_cmd(1, attr_mask);
}

#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <esp_mac.h>
#include <esp_system.h>
#include <ctype.h>
#include <math.h>
#include <mbedtls/aes.h>
#include <mbedtls/base64.h>
#include <stdlib.h>
#include <stdint.h>
#include <strings.h>
#include <string.h>

#define MSH_SPI_HZ                  8000000U
#define MSH_RESET_PULSE_MS          3U
#define MSH_BOOT_SETTLE_MS          20U
#define MSH_POLL_INTERVAL_MS        40U
#define MSH_TX_TIMEOUT_MS           6000U
#define MSH_HEADER_LEN              16U
#define MSH_MAX_PACKET_LEN          255U
#define MSH_MAX_DATA_LEN            (MSH_MAX_PACKET_LEN - MSH_HEADER_LEN)
#define MSH_MAX_KEY_LEN             32U
#define MSH_CHANNEL_NAME_MAX        31U
#define MSH_MAX_TEXT_CHARS          180U
#define MSH_LAST_TEXT_MAX           96U
#define MSH_DEFAULT_HOP_LIMIT       3U
#define MSH_DUP_CACHE_SIZE          48U
#define MSH_DUP_MAX_AGE_MS          1800000U

// SX126x commands
#define MSH_CMD_SET_STANDBY         0x80
#define MSH_CMD_SET_TX              0x83
#define MSH_CMD_SET_RX              0x82
#define MSH_CMD_SET_REGULATOR_MODE  0x96
#define MSH_CMD_SET_PA_CONFIG       0x95
#define MSH_CMD_SET_DIO_IRQ_PARAMS  0x08
#define MSH_CMD_SET_DIO2_AS_RF      0x9D
#define MSH_CMD_SET_DIO3_AS_TCXO    0x97
#define MSH_CMD_SET_RF_FREQUENCY    0x86
#define MSH_CMD_SET_PACKET_TYPE     0x8A
#define MSH_CMD_SET_TX_PARAMS       0x8E
#define MSH_CMD_SET_MOD_PARAMS      0x8B
#define MSH_CMD_SET_PKT_PARAMS      0x8C
#define MSH_CMD_SET_BUFFER_BASE     0x8F
#define MSH_CMD_WRITE_REGISTER      0x0D
#define MSH_CMD_READ_REGISTER       0x1D
#define MSH_CMD_WRITE_BUFFER        0x0E
#define MSH_CMD_READ_BUFFER         0x1E
#define MSH_CMD_GET_IRQ_STATUS      0x12
#define MSH_CMD_CLEAR_IRQ_STATUS    0x02
#define MSH_CMD_GET_RX_BUFFER       0x13
#define MSH_CMD_GET_PACKET_STATUS   0x14
#define MSH_CMD_GET_STATUS          0xC0
#define MSH_CMD_GET_PACKET_TYPE     0x11
#define MSH_CMD_GET_DEVICE_ERRORS   0x17
#define MSH_CMD_CALIBRATE_IMAGE     0x98

// SX126x values
#define MSH_PKT_TYPE_LORA           0x01
#define MSH_REGULATOR_DCDC          0x01
#define MSH_DIO2_RF_SWITCH_ON       0x01
#define MSH_STANDBY_RC              0x00
#define MSH_RX_TIMEOUT_INF          0xFFFFFFUL
#define MSH_TX_TIMEOUT_NONE         0x000000UL

// LoRa params (Meshtastic LONG_FAST default behavior)
#define MSH_LORA_BW_250             0x05
#define MSH_LORA_SF11               0x0B
#define MSH_LORA_CR_4_5             0x01
#define MSH_LORA_LDRO_AUTO_OFF      0x00
#define MSH_LORA_HEADER_EXPLICIT    0x00
#define MSH_LORA_CRC_ON             0x01
#define MSH_LORA_IQ_STANDARD        0x00

// IRQ bits
#define MSH_IRQ_TX_DONE             0x0001
#define MSH_IRQ_RX_DONE             0x0002
#define MSH_IRQ_PREAMBLE_DETECTED   0x0004
#define MSH_IRQ_SYNCWORD_VALID      0x0008
#define MSH_IRQ_HEADER_VALID        0x0010
#define MSH_IRQ_HEADER_ERR          0x0020
#define MSH_IRQ_CRC_ERR             0x0040
#define MSH_IRQ_TIMEOUT             0x0200
#define MSH_IRQ_ALL                 0x43FF

// Meshtastic wire constants
#define MSH_PORT_TEXT_MESSAGE_APP   1U
#define MSH_NODENUM_BROADCAST       0xFFFFFFFFUL

enum MeshtasticState {
    MSH_STATE_OFF,
    MSH_STATE_ON,
    MSH_STATE_ERROR
};

struct MeshtasticScanResult {
    bool ok;
    bool powered_before;
    bool temporary_power;
    bool has_status;
    uint8_t status;
    bool has_packet_type;
    uint8_t packet_type;
    bool has_device_errors;
    uint16_t device_errors;
    uint32_t elapsed_ms;
    bool mesh_ready;
    uint32_t rx_packets;
    uint32_t rx_text_packets;
    uint32_t rx_duplicates;
    uint32_t tx_packets;
    uint32_t rx_decode_failures;
    char error[48];
};

struct MeshtasticSnapshot {
    bool power_on;
    bool mesh_ready;
    bool last_probe_ok;
    bool has_status;
    uint8_t status;
    bool has_packet_type;
    uint8_t packet_type;
    bool has_device_errors;
    uint16_t device_errors;
    uint32_t last_probe_ms;
    uint32_t scan_count;
    uint32_t scan_fail_count;

    uint32_t node_num;
    uint32_t channel_num;
    float freq_mhz;
    uint8_t channel_hash;
    char channel_name[MSH_CHANNEL_NAME_MAX + 1];

    uint32_t rx_packets;
    uint32_t rx_text_packets;
    uint32_t rx_duplicates;
    uint32_t rx_decode_failures;
    uint32_t rx_crc_errors;
    uint32_t tx_packets;
    uint32_t tx_failures;

    uint32_t last_rx_ms;
    uint32_t last_tx_ms;
    uint32_t last_rx_from;
    uint32_t last_rx_to;
    uint32_t last_rx_id;
    int last_rx_rssi_dbm;
    float last_rx_snr_db;
    bool has_last_text;
    char last_text[MSH_LAST_TEXT_MAX + 1];

    char last_error[48];
};

static volatile MeshtasticState msh_state = MSH_STATE_OFF;
static bool msh_radio_ready = false;
static bool msh_last_probe_ok = false;
static bool msh_has_status = false;
static uint8_t msh_status = 0;
static bool msh_has_packet_type = false;
static uint8_t msh_packet_type = 0;
static bool msh_has_device_errors = false;
static uint16_t msh_device_errors = 0;
static uint32_t msh_last_probe_ms = 0;
static uint32_t msh_scan_count = 0;
static uint32_t msh_scan_fail_count = 0;
static char msh_last_error[48] = "";

static uint32_t msh_node_num = 0;
static uint32_t msh_channel_num = 0;
static float msh_freq_mhz = 0.0f;
static uint8_t msh_channel_hash = 0;
static char msh_channel_name[MSH_CHANNEL_NAME_MAX + 1] = "LongFast";
static uint8_t msh_channel_key[MSH_MAX_KEY_LEN] = {0};
static uint8_t msh_channel_key_len = 0;

static uint32_t msh_rx_packets = 0;
static uint32_t msh_rx_text_packets = 0;
static uint32_t msh_rx_duplicates = 0;
static uint32_t msh_rx_decode_failures = 0;
static uint32_t msh_rx_crc_errors = 0;
static uint32_t msh_tx_packets = 0;
static uint32_t msh_tx_failures = 0;
static uint32_t msh_last_rx_ms = 0;
static uint32_t msh_last_tx_ms = 0;
static uint32_t msh_last_rx_from = 0;
static uint32_t msh_last_rx_to = 0;
static uint32_t msh_last_rx_id = 0;
static int msh_last_rx_rssi_dbm = 0;
static float msh_last_rx_snr_db = 0.0f;
static bool msh_has_last_text = false;
static char msh_last_text[MSH_LAST_TEXT_MAX + 1] = "";

static uint32_t msh_last_poll_ms = 0;
static uint32_t msh_packet_counter = 0;
struct MshSeenPacket {
    uint32_t from;
    uint32_t id;
    uint32_t seen_ms;
};
static MshSeenPacket msh_seen_packets[MSH_DUP_CACHE_SIZE] = {};

static const uint8_t MSH_DEFAULT_PSK[16] = {
    0xd4, 0xf1, 0xbb, 0x3a, 0x20, 0x29, 0x07, 0x59,
    0xf0, 0xbc, 0xff, 0xab, 0xcf, 0x4e, 0x69, 0x01
};

static constexpr float MSH_REGION_FREQ_START_MHZ = 902.0f;
static constexpr float MSH_REGION_FREQ_END_MHZ = 928.0f;
static constexpr float MSH_REGION_SPACING_MHZ = 0.0f;
static constexpr float MSH_DEFAULT_BW_KHZ = 250.0f;

static void mshMarkRenderDirty() {
    render_requested = true;
    term_render_requested = true;
}

static uint32_t mshReadLe32(const uint8_t* p) {
    return ((uint32_t)p[0]) |
           ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

static void mshWriteLe32(uint8_t* p, uint32_t v) {
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
    p[3] = (uint8_t)((v >> 24) & 0xFF);
}

static bool mshAllBytesEqual(const uint8_t* data, size_t len, uint8_t value) {
    if (!data || len == 0) return false;
    for (size_t i = 0; i < len; i++) {
        if (data[i] != value) return false;
    }
    return true;
}

static bool mshWaitWhileBusy(uint32_t timeout_ms = 30U) {
#if defined(BOARD_LORA_BUSY)
    if (BOARD_LORA_BUSY >= 0) {
        uint32_t start = millis();
        while (digitalRead(BOARD_LORA_BUSY) == HIGH) {
            if ((uint32_t)(millis() - start) > timeout_ms) return false;
            delayMicroseconds(50);
        }
    }
#endif
    return true;
}

static bool mshSpiExchange(uint8_t opcode, const uint8_t* tx, size_t tx_len, uint8_t* rx, size_t rx_len) {
    if (!mshWaitWhileBusy()) return false;

    sdAcquire();
    SPI.beginTransaction(SPISettings(MSH_SPI_HZ, MSBFIRST, SPI_MODE0));
    digitalWrite(BOARD_LORA_CS, LOW);

    SPI.transfer(opcode);
    for (size_t i = 0; i < tx_len; i++) {
        SPI.transfer(tx ? tx[i] : 0x00);
    }
    for (size_t i = 0; i < rx_len; i++) {
        uint8_t v = SPI.transfer(0x00);
        if (rx) rx[i] = v;
    }

    digitalWrite(BOARD_LORA_CS, HIGH);
    SPI.endTransaction();
    sdRelease();

    return mshWaitWhileBusy();
}

static bool mshWriteCommand(uint8_t opcode, const uint8_t* data, size_t len) {
    return mshSpiExchange(opcode, data, len, NULL, 0);
}

static bool mshReadCommandEx(uint8_t opcode, const uint8_t* data, size_t data_len, uint8_t* out_status, uint8_t* out, size_t out_len) {
    uint8_t status = 0;
    if (!mshWaitWhileBusy()) return false;

    sdAcquire();
    SPI.beginTransaction(SPISettings(MSH_SPI_HZ, MSBFIRST, SPI_MODE0));
    digitalWrite(BOARD_LORA_CS, LOW);

    SPI.transfer(opcode);
    for (size_t i = 0; i < data_len; i++) SPI.transfer(data ? data[i] : 0x00);

    status = SPI.transfer(0x00);
    for (size_t i = 0; i < out_len; i++) {
        uint8_t v = SPI.transfer(0x00);
        if (out) out[i] = v;
    }

    digitalWrite(BOARD_LORA_CS, HIGH);
    SPI.endTransaction();
    sdRelease();

    if (!mshWaitWhileBusy()) return false;
    if (out_status) *out_status = status;
    return true;
}

// Raw helper used by probe/status paths where first byte is expected to be status.
static bool mshReadCommand(uint8_t opcode, uint8_t* out, size_t out_len) {
    if (!out || out_len == 0) return false;
    uint8_t status = 0;
    if (!mshReadCommandEx(opcode, NULL, 0, &status, out + 1, out_len - 1)) return false;
    out[0] = status;
    return true;
}

static bool mshWriteRegister(uint16_t addr, const uint8_t* data, size_t len) {
    if (!data || len == 0) return false;
    if (!mshWaitWhileBusy()) return false;

    sdAcquire();
    SPI.beginTransaction(SPISettings(MSH_SPI_HZ, MSBFIRST, SPI_MODE0));
    digitalWrite(BOARD_LORA_CS, LOW);

    SPI.transfer(MSH_CMD_WRITE_REGISTER);
    SPI.transfer((uint8_t)((addr >> 8) & 0xFF));
    SPI.transfer((uint8_t)(addr & 0xFF));
    for (size_t i = 0; i < len; i++) SPI.transfer(data[i]);

    digitalWrite(BOARD_LORA_CS, HIGH);
    SPI.endTransaction();
    sdRelease();

    return mshWaitWhileBusy();
}

static bool mshReadRegister(uint16_t addr, uint8_t* out, size_t len) {
    if (!out || len == 0) return false;
    uint8_t prefix[2] = {
        (uint8_t)((addr >> 8) & 0xFF),
        (uint8_t)(addr & 0xFF)
    };
    return mshReadCommandEx(MSH_CMD_READ_REGISTER, prefix, sizeof(prefix), NULL, out, len);
}

static bool mshWriteBuffer(uint8_t offset, const uint8_t* data, size_t len) {
    if (!data || len == 0) return false;
    if (len > MSH_MAX_PACKET_LEN) return false;
    if (!mshWaitWhileBusy()) return false;

    sdAcquire();
    SPI.beginTransaction(SPISettings(MSH_SPI_HZ, MSBFIRST, SPI_MODE0));
    digitalWrite(BOARD_LORA_CS, LOW);

    SPI.transfer(MSH_CMD_WRITE_BUFFER);
    SPI.transfer(offset);
    for (size_t i = 0; i < len; i++) SPI.transfer(data[i]);

    digitalWrite(BOARD_LORA_CS, HIGH);
    SPI.endTransaction();
    sdRelease();

    return mshWaitWhileBusy();
}

static bool mshReadBuffer(uint8_t offset, uint8_t* out, size_t len) {
    if (!out || len == 0) return false;
    if (len > MSH_MAX_PACKET_LEN) return false;
    uint8_t prefix[1] = { offset };
    return mshReadCommandEx(MSH_CMD_READ_BUFFER, prefix, sizeof(prefix), NULL, out, len);
}

static bool mshSetStandby() {
    uint8_t data[1] = { MSH_STANDBY_RC };
    return mshWriteCommand(MSH_CMD_SET_STANDBY, data, sizeof(data));
}

static bool mshSetPacketTypeLoRa() {
    uint8_t data[1] = { MSH_PKT_TYPE_LORA };
    return mshWriteCommand(MSH_CMD_SET_PACKET_TYPE, data, sizeof(data));
}

static bool mshSetRegulatorDcdc() {
    uint8_t data[1] = { MSH_REGULATOR_DCDC };
    return mshWriteCommand(MSH_CMD_SET_REGULATOR_MODE, data, sizeof(data));
}

static bool mshSetPaConfigDefault() {
    // SX1262 defaults used by RadioLib for normal operation.
    uint8_t data[4] = { 0x04, 0x07, 0x00, 0x01 };
    return mshWriteCommand(MSH_CMD_SET_PA_CONFIG, data, sizeof(data));
}

static bool mshSetTxParamsDefault() {
    uint8_t data[2] = { 17, 0x04 }; // 17dBm, 200us ramp
    return mshWriteCommand(MSH_CMD_SET_TX_PARAMS, data, sizeof(data));
}

static bool mshSetDio2RfSwitch() {
    uint8_t data[1] = { MSH_DIO2_RF_SWITCH_ON };
    return mshWriteCommand(MSH_CMD_SET_DIO2_AS_RF, data, sizeof(data));
}

static bool mshSetDio3Tcxo(float voltage, uint32_t delay_us = 5000U) {
    uint8_t v = 0x04; // 2.4V default for T-Deck Pro module.
    if (fabsf(voltage - 1.6f) <= 0.05f) v = 0x00;
    else if (fabsf(voltage - 1.7f) <= 0.05f) v = 0x01;
    else if (fabsf(voltage - 1.8f) <= 0.05f) v = 0x02;
    else if (fabsf(voltage - 2.2f) <= 0.05f) v = 0x03;
    else if (fabsf(voltage - 2.4f) <= 0.05f) v = 0x04;
    else if (fabsf(voltage - 2.7f) <= 0.05f) v = 0x05;
    else if (fabsf(voltage - 3.0f) <= 0.05f) v = 0x06;
    else if (fabsf(voltage - 3.3f) <= 0.05f) v = 0x07;

    uint32_t delay = (uint32_t)((float)delay_us / 15.625f);
    uint8_t data[4] = {
        v,
        (uint8_t)((delay >> 16) & 0xFF),
        (uint8_t)((delay >> 8) & 0xFF),
        (uint8_t)(delay & 0xFF)
    };
    return mshWriteCommand(MSH_CMD_SET_DIO3_AS_TCXO, data, sizeof(data));
}

static uint32_t mshFreqToReg(float mhz) {
    double hz = (double)mhz * 1000000.0;
    return (uint32_t)((hz * (double)(1UL << 25)) / 32000000.0);
}

static bool mshSetRfFrequency(float mhz) {
    uint32_t frf = mshFreqToReg(mhz);
    uint8_t data[4] = {
        (uint8_t)((frf >> 24) & 0xFF),
        (uint8_t)((frf >> 16) & 0xFF),
        (uint8_t)((frf >> 8) & 0xFF),
        (uint8_t)(frf & 0xFF)
    };
    return mshWriteCommand(MSH_CMD_SET_RF_FREQUENCY, data, sizeof(data));
}

static bool mshCalibrateImage(float freq_mhz) {
    uint8_t data[2] = {0, 0};
    int band = (int)freq_mhz;
    if (band >= 902 && band <= 928) {
        data[0] = 0xE1;
        data[1] = 0xE9;
    } else if (band >= 863 && band <= 870) {
        data[0] = 0xD7;
        data[1] = 0xDB;
    } else if (band >= 779 && band <= 787) {
        data[0] = 0xC1;
        data[1] = 0xC5;
    } else if (band >= 470 && band <= 510) {
        data[0] = 0x75;
        data[1] = 0x81;
    } else if (band >= 430 && band <= 440) {
        data[0] = 0x6B;
        data[1] = 0x6F;
    } else {
        return true; // Skip calibration for unknown range.
    }
    return mshWriteCommand(MSH_CMD_CALIBRATE_IMAGE, data, sizeof(data));
}

static bool mshSetModulationParams() {
    uint8_t data[4] = {
        MSH_LORA_SF11,
        MSH_LORA_BW_250,
        MSH_LORA_CR_4_5,
        MSH_LORA_LDRO_AUTO_OFF
    };
    return mshWriteCommand(MSH_CMD_SET_MOD_PARAMS, data, sizeof(data));
}

static bool mshSetPacketParams(uint8_t payload_len) {
    uint8_t data[6] = {
        (uint8_t)((16U >> 8) & 0xFF),
        (uint8_t)(16U & 0xFF),
        MSH_LORA_HEADER_EXPLICIT,
        payload_len,
        MSH_LORA_CRC_ON,
        MSH_LORA_IQ_STANDARD
    };
    return mshWriteCommand(MSH_CMD_SET_PKT_PARAMS, data, sizeof(data));
}

static bool mshSetSyncWord(uint8_t sync_word, uint8_t control_bits = 0x44) {
    uint8_t data[2] = {
        (uint8_t)((sync_word & 0xF0) | ((control_bits & 0xF0) >> 4)),
        (uint8_t)(((sync_word & 0x0F) << 4) | (control_bits & 0x0F))
    };
    return mshWriteRegister(0x0740, data, sizeof(data));
}

static bool mshSetBufferBaseAddress() {
    uint8_t data[2] = {0x00, 0x00};
    return mshWriteCommand(MSH_CMD_SET_BUFFER_BASE, data, sizeof(data));
}

static bool mshSetDioIrqParams(uint16_t irq_mask, uint16_t dio1_mask, uint16_t dio2_mask, uint16_t dio3_mask) {
    uint8_t data[8] = {
        (uint8_t)((irq_mask >> 8) & 0xFF), (uint8_t)(irq_mask & 0xFF),
        (uint8_t)((dio1_mask >> 8) & 0xFF), (uint8_t)(dio1_mask & 0xFF),
        (uint8_t)((dio2_mask >> 8) & 0xFF), (uint8_t)(dio2_mask & 0xFF),
        (uint8_t)((dio3_mask >> 8) & 0xFF), (uint8_t)(dio3_mask & 0xFF)
    };
    return mshWriteCommand(MSH_CMD_SET_DIO_IRQ_PARAMS, data, sizeof(data));
}

static bool mshClearIrqStatus(uint16_t mask) {
    uint8_t data[2] = {
        (uint8_t)((mask >> 8) & 0xFF),
        (uint8_t)(mask & 0xFF)
    };
    return mshWriteCommand(MSH_CMD_CLEAR_IRQ_STATUS, data, sizeof(data));
}

static bool mshGetIrqStatus(uint16_t* out_irq) {
    if (!out_irq) return false;
    uint8_t data[2] = {0};
    if (!mshReadCommandEx(MSH_CMD_GET_IRQ_STATUS, NULL, 0, NULL, data, sizeof(data))) return false;
    *out_irq = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
    return true;
}

static bool mshSetRxContinuous() {
    uint8_t data[3] = {
        (uint8_t)((MSH_RX_TIMEOUT_INF >> 16) & 0xFF),
        (uint8_t)((MSH_RX_TIMEOUT_INF >> 8) & 0xFF),
        (uint8_t)(MSH_RX_TIMEOUT_INF & 0xFF)
    };
    return mshWriteCommand(MSH_CMD_SET_RX, data, sizeof(data));
}

static bool mshSetTx() {
    uint8_t data[3] = {
        (uint8_t)((MSH_TX_TIMEOUT_NONE >> 16) & 0xFF),
        (uint8_t)((MSH_TX_TIMEOUT_NONE >> 8) & 0xFF),
        (uint8_t)(MSH_TX_TIMEOUT_NONE & 0xFF)
    };
    return mshWriteCommand(MSH_CMD_SET_TX, data, sizeof(data));
}

static bool mshGetRxBufferStatus(uint8_t* out_len, uint8_t* out_offset) {
    if (!out_len || !out_offset) return false;
    uint8_t data[2] = {0};
    if (!mshReadCommandEx(MSH_CMD_GET_RX_BUFFER, NULL, 0, NULL, data, sizeof(data))) return false;
    *out_len = data[0];
    *out_offset = data[1];
    return true;
}

static bool mshGetPacketStatus(int* out_rssi_dbm, float* out_snr_db) {
    uint8_t data[3] = {0};
    if (!mshReadCommandEx(MSH_CMD_GET_PACKET_STATUS, NULL, 0, NULL, data, sizeof(data))) return false;

    if (out_rssi_dbm) *out_rssi_dbm = -(int)(data[0] / 2);
    if (out_snr_db) *out_snr_db = ((int8_t)data[1]) / 4.0f;
    return true;
}

static bool mshStartReceiveMode() {
    if (!mshSetStandby()) return false;
    if (!mshSetPacketParams(0xFF)) return false;
    if (!mshSetBufferBaseAddress()) return false;
    if (!mshClearIrqStatus(MSH_IRQ_ALL)) return false;
    if (!mshSetRxContinuous()) return false;
    return true;
}

static uint8_t mshXorHash(const uint8_t* p, size_t len) {
    uint8_t h = 0;
    if (!p) return h;
    for (size_t i = 0; i < len; i++) h ^= p[i];
    return h;
}

static uint32_t mshHashDjb2(const char* s) {
    uint32_t h = 5381;
    if (!s) return h;
    while (*s) {
        h = ((h << 5) + h) + (uint8_t)(*s);
        s++;
    }
    return h;
}

static void mshBuildNodeNumFromMac() {
    uint8_t b[6] = {0};
    esp_efuse_mac_get_default(b);

    uint32_t node = ((uint32_t)b[2] << 24) |
                    ((uint32_t)b[3] << 16) |
                    ((uint32_t)b[4] << 8) |
                    ((uint32_t)b[5]);

    if (node == 0 || node == MSH_NODENUM_BROADCAST || node < 4) {
        node ^= 0x5A5A5A5A;
        if (node == 0 || node == MSH_NODENUM_BROADCAST || node < 4) {
            node = 0x2A000001;
        }
    }
    msh_node_num = node;
}

static int mshHexNibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
    if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
    return -1;
}

static void mshExpandDefaultPskIndex(uint8_t index, uint8_t* out_key, uint8_t* out_len) {
    if (!out_key || !out_len) return;
    memset(out_key, 0, MSH_MAX_KEY_LEN);
    if (index == 0) {
        *out_len = 0;
        return;
    }
    memcpy(out_key, MSH_DEFAULT_PSK, sizeof(MSH_DEFAULT_PSK));
    out_key[sizeof(MSH_DEFAULT_PSK) - 1] = (uint8_t)(out_key[sizeof(MSH_DEFAULT_PSK) - 1] + index - 1U);
    *out_len = sizeof(MSH_DEFAULT_PSK);
}

static void mshNormalizeChannelName(const char* in, char* out, size_t out_len) {
    if (!out || out_len < 2) return;
    const char* src = in ? in : "";
    while (*src && isspace((unsigned char)*src)) src++;

    size_t len = strlen(src);
    while (len > 0 && isspace((unsigned char)src[len - 1])) len--;

    if (len == 0) {
        snprintf(out, out_len, "LongFast");
        return;
    }
    if (len > out_len - 1) len = out_len - 1;
    memcpy(out, src, len);
    out[len] = '\0';
}

static bool mshTryParseHexKey(const char* hex, uint8_t* out_key, uint8_t* out_len) {
    if (!hex || !out_key || !out_len) return false;
    size_t n = strlen(hex);
    if ((n % 2U) != 0U || n < 2U || n > (MSH_MAX_KEY_LEN * 2U)) return false;

    size_t bytes = n / 2U;
    uint8_t tmp[MSH_MAX_KEY_LEN] = {0};
    for (size_t i = 0; i < bytes; i++) {
        int hi = mshHexNibble(hex[i * 2U]);
        int lo = mshHexNibble(hex[i * 2U + 1U]);
        if (hi < 0 || lo < 0) return false;
        tmp[i] = (uint8_t)((hi << 4) | lo);
    }

    if (bytes == 1U) {
        mshExpandDefaultPskIndex(tmp[0], out_key, out_len);
        return true;
    }
    if (bytes != 16U && bytes != 32U) return false;

    memset(out_key, 0, MSH_MAX_KEY_LEN);
    memcpy(out_key, tmp, bytes);
    *out_len = (uint8_t)bytes;
    return true;
}

static bool mshTryParseBase64Key(const char* b64, uint8_t* out_key, uint8_t* out_len) {
    if (!b64 || !out_key || !out_len) return false;
    char normalized[128];
    size_t n = 0;
    for (const char* p = b64; *p; p++) {
        char c = *p;
        if (isspace((unsigned char)c)) continue;
        if (c == '-') c = '+';
        else if (c == '_') c = '/';
        if (n + 1 >= sizeof(normalized)) return false;
        normalized[n++] = c;
    }
    if (n < 2U) return false;
    while ((n % 4U) != 0U) {
        if (n + 1 >= sizeof(normalized)) return false;
        normalized[n++] = '=';
    }
    normalized[n] = '\0';

    uint8_t tmp[MSH_MAX_KEY_LEN] = {0};
    size_t out_n = 0;
    int rc = mbedtls_base64_decode(tmp, sizeof(tmp), &out_n, (const unsigned char*)normalized, n);
    if (rc != 0) return false;

    if (out_n == 1U) {
        mshExpandDefaultPskIndex(tmp[0], out_key, out_len);
        return true;
    }
    if (out_n != 16U && out_n != 32U) return false;

    memset(out_key, 0, MSH_MAX_KEY_LEN);
    memcpy(out_key, tmp, out_n);
    *out_len = (uint8_t)out_n;
    return true;
}

static bool mshParseKeySpec(const char* spec, uint8_t* out_key, uint8_t* out_len) {
    if (!out_key || !out_len) return false;
    char tmp[96];
    tmp[0] = '\0';

    if (spec) {
        snprintf(tmp, sizeof(tmp), "%s", spec);
    }

    char* start = tmp;
    while (*start && isspace((unsigned char)*start)) start++;
    char* end = start + strlen(start);
    while (end > start && isspace((unsigned char)*(end - 1))) end--;
    *end = '\0';

    if (*start == '\0' || strcasecmp(start, "default") == 0 || strcasecmp(start, "public") == 0) {
        mshExpandDefaultPskIndex(1, out_key, out_len);
        return true;
    }
    if (strcasecmp(start, "none") == 0 || strcasecmp(start, "off") == 0 ||
        strcasecmp(start, "clear") == 0 || strcmp(start, "0") == 0) {
        memset(out_key, 0, MSH_MAX_KEY_LEN);
        *out_len = 0;
        return true;
    }

    bool all_digits = true;
    for (const char* p = start; *p; p++) {
        if (*p < '0' || *p > '9') {
            all_digits = false;
            break;
        }
    }
    if (all_digits) {
        long idx = strtol(start, NULL, 10);
        if (idx < 0 || idx > 255) return false;
        mshExpandDefaultPskIndex((uint8_t)idx, out_key, out_len);
        return true;
    }

    const char* hex = start;
    if (strncasecmp(start, "hex:", 4) == 0) hex = start + 4;
    if (mshTryParseHexKey(hex, out_key, out_len)) return true;

    const char* b64 = start;
    if (strncasecmp(start, "base64:", 7) == 0) b64 = start + 7;
    if (mshTryParseBase64Key(b64, out_key, out_len)) return true;

    return false;
}

static void mshDeriveChannelParams() {
    float bw_mhz = MSH_DEFAULT_BW_KHZ / 1000.0f;
    float channel_spacing_mhz = MSH_REGION_SPACING_MHZ + bw_mhz;
    if (channel_spacing_mhz <= 0.0f) channel_spacing_mhz = bw_mhz;

    uint32_t num_channels = (uint32_t)floorf((MSH_REGION_FREQ_END_MHZ - MSH_REGION_FREQ_START_MHZ) / channel_spacing_mhz);
    if (num_channels < 1U) num_channels = 1U;

    msh_channel_num = mshHashDjb2(msh_channel_name) % num_channels;
    msh_freq_mhz = MSH_REGION_FREQ_START_MHZ + (MSH_DEFAULT_BW_KHZ / 2000.0f) + ((float)msh_channel_num * bw_mhz);
    msh_channel_hash = mshXorHash((const uint8_t*)msh_channel_name, strlen(msh_channel_name)) ^
                       mshXorHash(msh_channel_key, msh_channel_key_len);
}

static void mshDeriveChannelDefaults() {
    mshNormalizeChannelName("LongFast", msh_channel_name, sizeof(msh_channel_name));
    mshExpandDefaultPskIndex(1, msh_channel_key, &msh_channel_key_len);
    mshDeriveChannelParams();
}

static void mshResetDuplicateCache() {
    memset(msh_seen_packets, 0, sizeof(msh_seen_packets));
}

static bool mshIsDuplicatePacket(uint32_t from, uint32_t packet_id) {
    uint32_t now = millis();
    int free_index = -1;
    int oldest_index = 0;
    uint32_t oldest_age = 0;

    for (int i = 0; i < (int)MSH_DUP_CACHE_SIZE; i++) {
        MshSeenPacket& e = msh_seen_packets[i];
        if (e.from == 0U) {
            if (free_index < 0) free_index = i;
            continue;
        }

        uint32_t age = now - e.seen_ms;
        if (age > MSH_DUP_MAX_AGE_MS) {
            e.from = 0U;
            e.id = 0U;
            e.seen_ms = 0U;
            if (free_index < 0) free_index = i;
            continue;
        }

        if (e.from == from && e.id == packet_id) {
            e.seen_ms = now;
            return true;
        }

        if (age >= oldest_age) {
            oldest_age = age;
            oldest_index = i;
        }
    }

    int write_index = (free_index >= 0) ? free_index : oldest_index;
    msh_seen_packets[write_index].from = from;
    msh_seen_packets[write_index].id = packet_id;
    msh_seen_packets[write_index].seen_ms = now;
    return false;
}

static void mshResetRuntimeStats() {
    msh_rx_packets = 0;
    msh_rx_text_packets = 0;
    msh_rx_duplicates = 0;
    msh_rx_decode_failures = 0;
    msh_rx_crc_errors = 0;
    msh_tx_packets = 0;
    msh_tx_failures = 0;
    msh_last_rx_ms = 0;
    msh_last_tx_ms = 0;
    msh_last_rx_from = 0;
    msh_last_rx_to = 0;
    msh_last_rx_id = 0;
    msh_last_rx_rssi_dbm = 0;
    msh_last_rx_snr_db = 0.0f;
    msh_has_last_text = false;
    msh_last_text[0] = '\0';
    msh_last_poll_ms = 0;
    mshResetDuplicateCache();
}

static uint32_t mshNextPacketId() {
    if (msh_packet_counter == 0) {
        msh_packet_counter = (uint32_t)(esp_random() & 0x3FFU);
    }
    msh_packet_counter = (msh_packet_counter + 1U) & 0x3FFU;
    uint32_t id = (uint32_t)(esp_random() & 0xFFFFFC00U) | msh_packet_counter;
    if (id == 0) id = 1;
    return id;
}

static bool mshCryptCtr(uint32_t from_node, uint32_t packet_id, uint8_t* bytes, size_t len) {
    if (!bytes || len == 0) return false;
    if (msh_channel_key_len == 0) return true; // unencrypted channel

    uint8_t nonce_counter[16] = {0};
    uint64_t pid = packet_id;
    memcpy(nonce_counter, &pid, sizeof(pid));          // LE packet id in first 8 bytes.
    memcpy(nonce_counter + 8, &from_node, sizeof(uint32_t)); // LE from-node in next 4 bytes.

    uint8_t stream_block[16] = {0};
    size_t nc_off = 0;

    mbedtls_aes_context aes;
    mbedtls_aes_init(&aes);
    int rc = mbedtls_aes_setkey_enc(&aes, msh_channel_key, msh_channel_key_len * 8U);
    if (rc == 0) {
        rc = mbedtls_aes_crypt_ctr(&aes, len, &nc_off, nonce_counter, stream_block, bytes, bytes);
    }
    mbedtls_aes_free(&aes);
    return rc == 0;
}

static bool mshWriteVarint(uint32_t v, uint8_t* out, size_t out_len, size_t* io_off) {
    if (!out || !io_off) return false;
    do {
        if (*io_off >= out_len) return false;
        uint8_t b = (uint8_t)(v & 0x7F);
        v >>= 7;
        if (v) b |= 0x80;
        out[(*io_off)++] = b;
    } while (v);
    return true;
}

static bool mshEncodeTextData(const char* text, uint8_t* out, size_t out_len, size_t* out_used) {
    if (!text || !out || !out_used) return false;
    size_t text_len = strlen(text);
    if (text_len > MSH_MAX_TEXT_CHARS) return false;

    size_t off = 0;
    if (!mshWriteVarint((1U << 3) | 0U, out, out_len, &off)) return false; // field 1 varint
    if (!mshWriteVarint(MSH_PORT_TEXT_MESSAGE_APP, out, out_len, &off)) return false;
    if (!mshWriteVarint((2U << 3) | 2U, out, out_len, &off)) return false; // field 2 bytes
    if (!mshWriteVarint((uint32_t)text_len, out, out_len, &off)) return false;
    if (off + text_len > out_len) return false;

    memcpy(out + off, text, text_len);
    off += text_len;
    *out_used = off;
    return true;
}

static bool mshReadVarint(const uint8_t* buf, size_t len, size_t* io_off, uint32_t* out) {
    if (!buf || !io_off || !out) return false;
    uint64_t value = 0;
    uint8_t shift = 0;
    while (*io_off < len && shift <= 63) {
        uint8_t b = buf[(*io_off)++];
        value |= ((uint64_t)(b & 0x7F) << shift);
        if ((b & 0x80) == 0) {
            *out = (uint32_t)value;
            return true;
        }
        shift = (uint8_t)(shift + 7);
    }
    return false;
}

static bool mshSkipField(const uint8_t* buf, size_t len, size_t* io_off, uint8_t wire_type) {
    if (!buf || !io_off) return false;
    uint32_t n = 0;
    switch (wire_type) {
        case 0: // varint
            return mshReadVarint(buf, len, io_off, &n);
        case 1: // fixed64
            if (*io_off + 8 > len) return false;
            *io_off += 8;
            return true;
        case 2: // length-delimited
            if (!mshReadVarint(buf, len, io_off, &n)) return false;
            if (*io_off + n > len) return false;
            *io_off += n;
            return true;
        case 5: // fixed32
            if (*io_off + 4 > len) return false;
            *io_off += 4;
            return true;
        default:
            return false;
    }
}

static bool mshDecodeData(const uint8_t* buf, size_t len, uint32_t* out_portnum, const uint8_t** out_payload, size_t* out_payload_len) {
    if (!buf || !out_portnum || !out_payload || !out_payload_len) return false;

    *out_portnum = 0;
    *out_payload = NULL;
    *out_payload_len = 0;

    size_t off = 0;
    bool has_port = false;
    while (off < len) {
        uint32_t key = 0;
        if (!mshReadVarint(buf, len, &off, &key)) return false;

        uint32_t field = key >> 3;
        uint8_t wire = (uint8_t)(key & 0x07);

        if (field == 1 && wire == 0) {
            uint32_t v = 0;
            if (!mshReadVarint(buf, len, &off, &v)) return false;
            *out_portnum = v;
            has_port = true;
            continue;
        }

        if (field == 2 && wire == 2) {
            uint32_t n = 0;
            if (!mshReadVarint(buf, len, &off, &n)) return false;
            if (off + n > len) return false;
            *out_payload = buf + off;
            *out_payload_len = n;
            off += n;
            continue;
        }

        if (!mshSkipField(buf, len, &off, wire)) return false;
    }

    return has_port && (*out_portnum != 0);
}

static void mshCopyTextForUi(char* out, size_t out_len, const uint8_t* src, size_t src_len) {
    if (!out || out_len < 1) return;
    if (!src || src_len == 0) {
        out[0] = '\0';
        return;
    }
    size_t n = src_len;
    if (n > out_len - 1) n = out_len - 1;
    for (size_t i = 0; i < n; i++) {
        uint8_t c = src[i];
        if (c == '\r' || c == '\n' || c == '\t') out[i] = ' ';
        else if (c >= 32 && c < 127) out[i] = (char)c;
        else out[i] = '.';
    }
    out[n] = '\0';
}

static uint8_t mshClampHopLimit(uint8_t hop_limit) {
    if (hop_limit == 0 || hop_limit > 7U) return MSH_DEFAULT_HOP_LIMIT;
    return hop_limit;
}

static bool mshBuildWirePacket(const uint8_t* encrypted_data, size_t data_len, uint32_t to_node, uint32_t packet_id,
                               bool want_ack, uint8_t hop_limit, uint8_t* out, size_t* out_len) {
    if (!encrypted_data || !out || !out_len) return false;
    if (data_len > MSH_MAX_DATA_LEN) return false;

    uint8_t hop = mshClampHopLimit(hop_limit);
    size_t total = MSH_HEADER_LEN + data_len;
    if (total > MSH_MAX_PACKET_LEN) return false;

    mshWriteLe32(out + 0, to_node);
    mshWriteLe32(out + 4, msh_node_num);
    mshWriteLe32(out + 8, packet_id);

    uint8_t flags = (uint8_t)(hop & 0x07U);
    if (want_ack) flags |= 0x08U;
    flags |= (uint8_t)((hop & 0x07U) << 5);
    out[12] = flags;
    out[13] = msh_channel_hash;
    out[14] = 0; // no next-hop preference
    out[15] = (uint8_t)(msh_node_num & 0xFF);

    memcpy(out + MSH_HEADER_LEN, encrypted_data, data_len);
    *out_len = total;
    return true;
}

static bool mshTransmitPacket(const uint8_t* frame, size_t frame_len) {
    if (!frame || frame_len < MSH_HEADER_LEN || frame_len > MSH_MAX_PACKET_LEN) return false;

    if (!mshSetStandby()) return false;
    if (!mshSetPacketParams((uint8_t)frame_len)) return false;
    if (!mshSetBufferBaseAddress()) return false;
    if (!mshWriteBuffer(0x00, frame, frame_len)) return false;
    if (!mshClearIrqStatus(MSH_IRQ_ALL)) return false;
    if (!mshSetTx()) return false;

    uint32_t start = millis();
    while ((uint32_t)(millis() - start) < MSH_TX_TIMEOUT_MS) {
        uint16_t irq = 0;
        if (!mshGetIrqStatus(&irq)) break;

        if (irq & MSH_IRQ_TX_DONE) {
            mshClearIrqStatus(MSH_IRQ_TX_DONE | MSH_IRQ_TIMEOUT);
            return mshStartReceiveMode();
        }
        if (irq & MSH_IRQ_TIMEOUT) {
            mshClearIrqStatus(MSH_IRQ_TIMEOUT);
            return false;
        }
        delay(5);
    }
    return false;
}

static bool mshConfigureForMesh() {
    const uint16_t irq_mask = (MSH_IRQ_TX_DONE | MSH_IRQ_RX_DONE | MSH_IRQ_TIMEOUT |
                               MSH_IRQ_HEADER_ERR | MSH_IRQ_CRC_ERR);

    if (!mshSetStandby()) return false;
    if (!mshSetPacketTypeLoRa()) return false;
    if (!mshSetRegulatorDcdc()) return false;
    if (!mshSetDio3Tcxo(2.4f)) return false;
    if (!mshSetDio2RfSwitch()) return false;
    if (!mshCalibrateImage(msh_freq_mhz)) return false;
    if (!mshSetRfFrequency(msh_freq_mhz)) return false;
    if (!mshSetModulationParams()) return false;
    if (!mshSetSyncWord(0x2B, 0x44)) return false;
    if (!mshSetPaConfigDefault()) return false;
    if (!mshSetTxParamsDefault()) return false;
    if (!mshSetDioIrqParams(irq_mask, irq_mask, 0, 0)) return false;
    if (!mshStartReceiveMode()) return false;
    return true;
}

const char* meshtasticPacketTypeName(uint8_t packet_type) {
    switch (packet_type) {
        case 0x00: return "gfsk";
        case 0x01: return "lora";
        case 0x02: return "lrfhss";
        default: return "?";
    }
}

void meshtasticInit() {
    pinMode(BOARD_LORA_EN, OUTPUT);
    pinMode(BOARD_LORA_RST, OUTPUT);
    pinMode(BOARD_LORA_CS, OUTPUT);
#if defined(BOARD_LORA_BUSY)
    if (BOARD_LORA_BUSY >= 0) pinMode(BOARD_LORA_BUSY, INPUT);
#endif
#if defined(BOARD_LORA_DIO1)
    if (BOARD_LORA_DIO1 >= 0) pinMode(BOARD_LORA_DIO1, INPUT);
#endif

    digitalWrite(BOARD_LORA_CS, HIGH);
    digitalWrite(BOARD_LORA_RST, LOW);
    digitalWrite(BOARD_LORA_EN, LOW);

    msh_state = MSH_STATE_OFF;
    msh_radio_ready = false;
    msh_last_probe_ok = false;
    msh_has_status = false;
    msh_status = 0;
    msh_has_packet_type = false;
    msh_packet_type = 0;
    msh_has_device_errors = false;
    msh_device_errors = 0;
    msh_last_probe_ms = 0;
    msh_scan_count = 0;
    msh_scan_fail_count = 0;
    msh_last_error[0] = '\0';
    msh_packet_counter = 0;

    mshBuildNodeNumFromMac();
    mshDeriveChannelDefaults();
    mshResetRuntimeStats();
}

bool meshtasticIsPowered() {
    return msh_state != MSH_STATE_OFF;
}

const char* meshtasticStatusShort() {
    if (msh_state == MSH_STATE_OFF) return "off";
    if (msh_state == MSH_STATE_ERROR) return "err";
    if (!msh_radio_ready) return "...";
    return "on";
}

const char* meshtasticLastError() {
    return msh_last_error;
}

bool meshtasticConfigureChannel(const char* channel_name = NULL, const char* key_spec = NULL) {
    char next_name[MSH_CHANNEL_NAME_MAX + 1];
    snprintf(next_name, sizeof(next_name), "%s", msh_channel_name);
    if (channel_name && channel_name[0] != '\0') {
        mshNormalizeChannelName(channel_name, next_name, sizeof(next_name));
    }

    uint8_t next_key[MSH_MAX_KEY_LEN];
    memset(next_key, 0, sizeof(next_key));
    memcpy(next_key, msh_channel_key, msh_channel_key_len);
    uint8_t next_key_len = msh_channel_key_len;
    if (key_spec && key_spec[0] != '\0') {
        if (!mshParseKeySpec(key_spec, next_key, &next_key_len)) {
            snprintf(msh_last_error, sizeof(msh_last_error), "bad key spec");
            return false;
        }
    }

    bool same_name = (strncmp(next_name, msh_channel_name, sizeof(next_name)) == 0);
    bool same_key = (next_key_len == msh_channel_key_len) &&
                    (memcmp(next_key, msh_channel_key, next_key_len) == 0);
    if (same_name && same_key) return true;

    char old_name[MSH_CHANNEL_NAME_MAX + 1];
    uint8_t old_key[MSH_MAX_KEY_LEN];
    uint8_t old_key_len = msh_channel_key_len;
    snprintf(old_name, sizeof(old_name), "%s", msh_channel_name);
    memset(old_key, 0, sizeof(old_key));
    memcpy(old_key, msh_channel_key, msh_channel_key_len);

    snprintf(msh_channel_name, sizeof(msh_channel_name), "%s", next_name);
    memset(msh_channel_key, 0, sizeof(msh_channel_key));
    memcpy(msh_channel_key, next_key, next_key_len);
    msh_channel_key_len = next_key_len;
    mshDeriveChannelParams();

    if (meshtasticIsPowered()) {
        msh_radio_ready = mshConfigureForMesh();
        if (!msh_radio_ready) {
            snprintf(msh_channel_name, sizeof(msh_channel_name), "%s", old_name);
            memset(msh_channel_key, 0, sizeof(msh_channel_key));
            memcpy(msh_channel_key, old_key, old_key_len);
            msh_channel_key_len = old_key_len;
            mshDeriveChannelParams();
            bool restore_ok = mshConfigureForMesh();
            msh_radio_ready = restore_ok;
            msh_state = restore_ok ? MSH_STATE_ON : MSH_STATE_ERROR;
            snprintf(msh_last_error, sizeof(msh_last_error), "channel apply failed");
            return false;
        }
        msh_state = MSH_STATE_ON;
    }

    mshResetRuntimeStats();
    msh_last_error[0] = '\0';
    mshMarkRenderDirty();
    return true;
}

bool meshtasticSetPowered(bool on) {
    if (on) {
        if (meshtasticIsPowered()) {
            snprintf(msh_last_error, sizeof(msh_last_error), "already on");
            return false;
        }

        // BOARD_1V8_EN is shared with touch; keep it enabled before radio power-up.
        digitalWrite(BOARD_1V8_EN, HIGH);
        digitalWrite(BOARD_LORA_CS, HIGH);
        digitalWrite(BOARD_LORA_EN, HIGH);
        delay(2);
        digitalWrite(BOARD_LORA_RST, LOW);
        delay(MSH_RESET_PULSE_MS);
        digitalWrite(BOARD_LORA_RST, HIGH);
        delay(MSH_BOOT_SETTLE_MS);

        msh_state = MSH_STATE_ON;
        msh_radio_ready = mshConfigureForMesh();
        if (!msh_radio_ready) {
            msh_state = MSH_STATE_ERROR;
            snprintf(msh_last_error, sizeof(msh_last_error), "radio init failed");
            digitalWrite(BOARD_LORA_RST, LOW);
            digitalWrite(BOARD_LORA_EN, LOW);
            SERIAL_LOGLN("MSH: radio init failed");
            return false;
        }

        msh_last_error[0] = '\0';
        mshMarkRenderDirty();
        SERIAL_LOGLN("MSH: powered on (mesh ready)");
        return true;
    }

    if (!meshtasticIsPowered()) {
        snprintf(msh_last_error, sizeof(msh_last_error), "already off");
        return false;
    }

    mshSetStandby();
    msh_radio_ready = false;
    digitalWrite(BOARD_LORA_CS, HIGH);
    digitalWrite(BOARD_LORA_RST, LOW);
    digitalWrite(BOARD_LORA_EN, LOW);
    msh_state = MSH_STATE_OFF;
    mshMarkRenderDirty();
    SERIAL_LOGLN("MSH: powered off");
    return true;
}

bool meshtasticSendTextTo(const char* text, uint32_t to_node, bool want_ack = false, uint8_t hop_limit = MSH_DEFAULT_HOP_LIMIT,
                          uint32_t* out_packet_id = NULL) {
    if (!text || text[0] == '\0') {
        snprintf(msh_last_error, sizeof(msh_last_error), "empty text");
        return false;
    }
    if (to_node == 0U) {
        snprintf(msh_last_error, sizeof(msh_last_error), "bad dest");
        return false;
    }
    if (!meshtasticIsPowered() || !msh_radio_ready) {
        snprintf(msh_last_error, sizeof(msh_last_error), "radio off");
        return false;
    }

    uint8_t data_buf[MSH_MAX_DATA_LEN] = {0};
    size_t data_len = 0;
    if (!mshEncodeTextData(text, data_buf, sizeof(data_buf), &data_len)) {
        snprintf(msh_last_error, sizeof(msh_last_error), "encode failed");
        return false;
    }

    uint32_t packet_id = mshNextPacketId();
    if (!mshCryptCtr(msh_node_num, packet_id, data_buf, data_len)) {
        snprintf(msh_last_error, sizeof(msh_last_error), "crypto failed");
        return false;
    }

    uint8_t frame[MSH_MAX_PACKET_LEN] = {0};
    size_t frame_len = 0;
    if (!mshBuildWirePacket(data_buf, data_len, to_node, packet_id, want_ack, hop_limit, frame, &frame_len)) {
        snprintf(msh_last_error, sizeof(msh_last_error), "packet build failed");
        return false;
    }

    if (!mshTransmitPacket(frame, frame_len)) {
        msh_tx_failures++;
        snprintf(msh_last_error, sizeof(msh_last_error), "tx failed");
        return false;
    }

    msh_tx_packets++;
    msh_last_tx_ms = millis();
    if (out_packet_id) *out_packet_id = packet_id;
    msh_last_error[0] = '\0';
    return true;
}

bool meshtasticSendText(const char* text, uint32_t* out_packet_id = NULL) {
    return meshtasticSendTextTo(text, MSH_NODENUM_BROADCAST, false, MSH_DEFAULT_HOP_LIMIT, out_packet_id);
}

static void mshHandleReceivedFrame(const uint8_t* frame, size_t frame_len, int rssi_dbm, float snr_db) {
    if (!frame || frame_len <= MSH_HEADER_LEN) return;
    if (frame_len > MSH_MAX_PACKET_LEN) return;

    uint32_t to = mshReadLe32(frame + 0);
    uint32_t from = mshReadLe32(frame + 4);
    uint32_t packet_id = mshReadLe32(frame + 8);
    uint8_t channel_hash = frame[13];

    if (from == 0) return;
    if (from == msh_node_num) return;
    if (channel_hash != msh_channel_hash) return;
    if (mshIsDuplicatePacket(from, packet_id)) {
        msh_rx_duplicates++;
        return;
    }

    const uint8_t* enc_payload = frame + MSH_HEADER_LEN;
    size_t enc_len = frame_len - MSH_HEADER_LEN;
    if (enc_len == 0 || enc_len > MSH_MAX_DATA_LEN) return;

    uint8_t plain[MSH_MAX_DATA_LEN];
    memcpy(plain, enc_payload, enc_len);

    bool parsed = false;
    uint32_t portnum = 0;
    const uint8_t* payload = NULL;
    size_t payload_len = 0;

    if (mshCryptCtr(from, packet_id, plain, enc_len)) {
        parsed = mshDecodeData(plain, enc_len, &portnum, &payload, &payload_len);
    }
    if (!parsed) {
        // Support channels with no encryption.
        parsed = mshDecodeData(enc_payload, enc_len, &portnum, &payload, &payload_len);
    }

    if (!parsed) {
        msh_rx_decode_failures++;
        return;
    }

    msh_last_rx_ms = millis();
    msh_last_rx_from = from;
    msh_last_rx_to = to;
    msh_last_rx_id = packet_id;
    msh_last_rx_rssi_dbm = rssi_dbm;
    msh_last_rx_snr_db = snr_db;

    if (portnum == MSH_PORT_TEXT_MESSAGE_APP) {
        msh_rx_text_packets++;
        msh_has_last_text = true;
        mshCopyTextForUi(msh_last_text, sizeof(msh_last_text), payload, payload_len);
        SERIAL_LOGF("MSH RX text from !%08lx: %s\n",
                    (unsigned long)from,
                    msh_last_text);
    }
}

void meshtasticPoll() {
    if (!meshtasticIsPowered() || !msh_radio_ready) return;

    uint32_t now = millis();
    if ((uint32_t)(now - msh_last_poll_ms) < MSH_POLL_INTERVAL_MS) return;
    msh_last_poll_ms = now;

#if defined(BOARD_LORA_DIO1)
    if (BOARD_LORA_DIO1 >= 0 && digitalRead(BOARD_LORA_DIO1) == LOW) {
        return;
    }
#endif

    uint16_t irq = 0;
    if (!mshGetIrqStatus(&irq) || irq == 0) return;

    bool restart_rx = false;
    if (irq & MSH_IRQ_RX_DONE) {
        uint8_t payload_len = 0;
        uint8_t payload_off = 0;
        if (mshGetRxBufferStatus(&payload_len, &payload_off)) {
            if (payload_len >= MSH_HEADER_LEN && payload_len <= MSH_MAX_PACKET_LEN) {
                uint8_t frame[MSH_MAX_PACKET_LEN] = {0};
                if (mshReadBuffer(payload_off, frame, payload_len)) {
                    int rssi = 0;
                    float snr = 0.0f;
                    mshGetPacketStatus(&rssi, &snr);
                    msh_rx_packets++;
                    mshHandleReceivedFrame(frame, payload_len, rssi, snr);
                } else {
                    msh_rx_decode_failures++;
                }
            } else {
                msh_rx_decode_failures++;
            }
        } else {
            msh_rx_decode_failures++;
        }
        restart_rx = true;
    }

    if (irq & (MSH_IRQ_CRC_ERR | MSH_IRQ_HEADER_ERR)) {
        msh_rx_crc_errors++;
        restart_rx = true;
    }

    if (irq & MSH_IRQ_TIMEOUT) {
        restart_rx = true;
    }

    mshClearIrqStatus(irq);
    if (restart_rx) {
        mshStartReceiveMode();
    }
}

static void mshProbe(MeshtasticScanResult* out) {
    if (!out) return;

    uint8_t status_rsp[1] = {0};
    uint8_t pkt_rsp[2] = {0};
    uint8_t err_rsp[3] = {0};
    bool got_status = mshReadCommand(MSH_CMD_GET_STATUS, status_rsp, sizeof(status_rsp));
    bool got_packet = mshReadCommand(MSH_CMD_GET_PACKET_TYPE, pkt_rsp, sizeof(pkt_rsp));
    bool got_errors = mshReadCommand(MSH_CMD_GET_DEVICE_ERRORS, err_rsp, sizeof(err_rsp));

    if (got_status && status_rsp[0] != 0x00 && status_rsp[0] != 0xFF) {
        out->status = status_rsp[0];
        out->has_status = true;
    }
    if (got_packet &&
        !mshAllBytesEqual(pkt_rsp, sizeof(pkt_rsp), 0x00) &&
        !mshAllBytesEqual(pkt_rsp, sizeof(pkt_rsp), 0xFF)) {
        out->packet_type = pkt_rsp[1];
        out->has_packet_type = true;
        if (!out->has_status && pkt_rsp[0] != 0x00 && pkt_rsp[0] != 0xFF) {
            out->status = pkt_rsp[0];
            out->has_status = true;
        }
    }
    if (got_errors &&
        !mshAllBytesEqual(err_rsp, sizeof(err_rsp), 0x00) &&
        !mshAllBytesEqual(err_rsp, sizeof(err_rsp), 0xFF)) {
        out->device_errors = (uint16_t)(((uint16_t)err_rsp[1] << 8) | err_rsp[2]);
        out->has_device_errors = true;
        if (!out->has_status && err_rsp[0] != 0x00 && err_rsp[0] != 0xFF) {
            out->status = err_rsp[0];
            out->has_status = true;
        }
    }

    out->ok = out->has_status && out->has_packet_type;
    out->mesh_ready = msh_radio_ready;
    out->rx_packets = msh_rx_packets;
    out->rx_text_packets = msh_rx_text_packets;
    out->rx_duplicates = msh_rx_duplicates;
    out->tx_packets = msh_tx_packets;
    out->rx_decode_failures = msh_rx_decode_failures;

    if (!out->ok) {
        if (!got_status && !got_packet && !got_errors) {
            snprintf(out->error, sizeof(out->error), "spi read failed");
        } else {
            snprintf(out->error, sizeof(out->error), "radio not responding");
        }
    } else {
        out->error[0] = '\0';
    }
}

bool meshtasticScanStatus(MeshtasticScanResult* out) {
    if (!out) return false;
    memset(out, 0, sizeof(*out));
    out->powered_before = meshtasticIsPowered();

    uint32_t started_ms = millis();
    if (!out->powered_before) {
        if (!meshtasticSetPowered(true)) {
            out->ok = false;
            snprintf(out->error, sizeof(out->error), "power on failed");
            out->elapsed_ms = millis() - started_ms;
            msh_scan_count++;
            msh_scan_fail_count++;
            msh_last_probe_ok = false;
            msh_last_probe_ms = millis();
            snprintf(msh_last_error, sizeof(msh_last_error), "%s", out->error);
            return true;
        }
        out->temporary_power = true;
        delay(5);
    }

    mshProbe(out);

    if (out->temporary_power) {
        meshtasticSetPowered(false);
    } else if (!out->ok) {
        msh_state = MSH_STATE_ERROR;
    } else {
        msh_state = MSH_STATE_ON;
    }

    out->elapsed_ms = millis() - started_ms;
    msh_scan_count++;
    msh_last_probe_ms = millis();
    msh_last_probe_ok = out->ok;
    if (!out->ok) msh_scan_fail_count++;

    msh_has_status = out->has_status;
    msh_status = out->status;
    msh_has_packet_type = out->has_packet_type;
    msh_packet_type = out->packet_type;
    msh_has_device_errors = out->has_device_errors;
    msh_device_errors = out->device_errors;

    if (out->ok) {
        msh_last_error[0] = '\0';
    } else if (out->error[0] != '\0') {
        snprintf(msh_last_error, sizeof(msh_last_error), "%s", out->error);
    } else {
        snprintf(msh_last_error, sizeof(msh_last_error), "scan failed");
    }
    return true;
}

void meshtasticGetSnapshot(MeshtasticSnapshot* out) {
    if (!out) return;
    out->power_on = meshtasticIsPowered();
    out->mesh_ready = msh_radio_ready;
    out->last_probe_ok = msh_last_probe_ok;
    out->has_status = msh_has_status;
    out->status = msh_status;
    out->has_packet_type = msh_has_packet_type;
    out->packet_type = msh_packet_type;
    out->has_device_errors = msh_has_device_errors;
    out->device_errors = msh_device_errors;
    out->last_probe_ms = msh_last_probe_ms;
    out->scan_count = msh_scan_count;
    out->scan_fail_count = msh_scan_fail_count;

    out->node_num = msh_node_num;
    out->channel_num = msh_channel_num;
    out->freq_mhz = msh_freq_mhz;
    out->channel_hash = msh_channel_hash;
    snprintf(out->channel_name, sizeof(out->channel_name), "%s", msh_channel_name);

    out->rx_packets = msh_rx_packets;
    out->rx_text_packets = msh_rx_text_packets;
    out->rx_duplicates = msh_rx_duplicates;
    out->rx_decode_failures = msh_rx_decode_failures;
    out->rx_crc_errors = msh_rx_crc_errors;
    out->tx_packets = msh_tx_packets;
    out->tx_failures = msh_tx_failures;

    out->last_rx_ms = msh_last_rx_ms;
    out->last_tx_ms = msh_last_tx_ms;
    out->last_rx_from = msh_last_rx_from;
    out->last_rx_to = msh_last_rx_to;
    out->last_rx_id = msh_last_rx_id;
    out->last_rx_rssi_dbm = msh_last_rx_rssi_dbm;
    out->last_rx_snr_db = msh_last_rx_snr_db;
    out->has_last_text = msh_has_last_text;
    snprintf(out->last_text, sizeof(out->last_text), "%s", msh_last_text);

    snprintf(out->last_error, sizeof(out->last_error), "%s", msh_last_error);
}

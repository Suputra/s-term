// --- Bluetooth LE HID peripheral (keyboard + trackpad mouse) ---

#include <BLEDevice.h>
#include <BLEHIDDevice.h>
#include <BLEServer.h>
#include <BLESecurity.h>
#include <BLEUtils.h>
#include <esp_gap_ble_api.h>
#include <cstring>
#include <string>

#define US_KEYBOARD
#include <HIDKeyboardTypes.h>

#define BT_SCAN_DURATION_S           4U
#define BT_SCAN_MAX_RESULTS          12
#define BT_REPORT_ID_KEYBOARD        1
#define BT_REPORT_ID_MOUSE           2

#define BT_STATUS_SERVICE_UUID       "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define BT_STATUS_CHAR_UUID          "beb5483e-36e1-4688-b7f5-ea07361b26a8"

enum BtState { BT_STATE_OFF, BT_STATE_ADVERTISING, BT_STATE_CONNECTED, BT_STATE_ERROR };

static volatile BtState bt_state = BT_STATE_OFF;
static volatile bool bt_initialized = false;
static volatile bool bt_connected = false;
static volatile bool bt_advertising = false;
static volatile bool bt_bonded = false;

static BLEServer* bt_server = NULL;
static BLEHIDDevice* bt_hid = NULL;
static BLECharacteristic* bt_kbd_input = NULL;
static BLECharacteristic* bt_mouse_input = NULL;
static BLECharacteristic* bt_kbd_output = NULL;
static BLEService* bt_status_service = NULL;
static BLECharacteristic* bt_status_char = NULL;

static char bt_peer_addr[18] = "";

struct BtScanEntry {
    char addr[18];
    char name[32];
    int rssi;
    bool has_name;
};

static volatile bool bt_scan_running = false;
static volatile bool bt_scan_done = false;
static volatile bool bt_scan_ok = false;
static bool bt_scan_restore_disabled = false;
static bool bt_scan_resume_adv = false;
static int bt_scan_total_found = 0;
static int bt_scan_count = 0;
static BtScanEntry bt_scan_results[BT_SCAN_MAX_RESULTS];

static const uint8_t bt_hid_report_map[] = {
    // Keyboard (Report ID 1)
    0x05, 0x01,       // Usage Page (Generic Desktop)
    0x09, 0x06,       // Usage (Keyboard)
    0xA1, 0x01,       // Collection (Application)
    0x85, BT_REPORT_ID_KEYBOARD,
    0x05, 0x07,       // Usage Page (Keyboard/Keypad)
    0x19, 0xE0,       // Usage Minimum (Keyboard LeftControl)
    0x29, 0xE7,       // Usage Maximum (Keyboard Right GUI)
    0x15, 0x00,       // Logical Minimum (0)
    0x25, 0x01,       // Logical Maximum (1)
    0x75, 0x01,       // Report Size (1)
    0x95, 0x08,       // Report Count (8)
    0x81, 0x02,       // Input (Data,Var,Abs)
    0x95, 0x01,       // Report Count (1)
    0x75, 0x08,       // Report Size (8)
    0x81, 0x01,       // Input (Const,Array,Abs)
    0x95, 0x06,       // Report Count (6)
    0x75, 0x08,       // Report Size (8)
    0x15, 0x00,       // Logical Minimum (0)
    0x25, 0x65,       // Logical Maximum (101)
    0x05, 0x07,       // Usage Page (Keyboard/Keypad)
    0x19, 0x00,       // Usage Minimum (Reserved)
    0x29, 0x65,       // Usage Maximum (Keyboard Application)
    0x81, 0x00,       // Input (Data,Array,Abs)
    0x95, 0x05,       // Report Count (5)
    0x75, 0x01,       // Report Size (1)
    0x05, 0x08,       // Usage Page (LEDs)
    0x19, 0x01,       // Usage Minimum (Num Lock)
    0x29, 0x05,       // Usage Maximum (Kana)
    0x91, 0x02,       // Output (Data,Var,Abs)
    0x95, 0x01,       // Report Count (1)
    0x75, 0x03,       // Report Size (3)
    0x91, 0x01,       // Output (Const,Array,Abs)
    0xC0,             // End Collection

    // Mouse (Report ID 2)
    0x05, 0x01,       // Usage Page (Generic Desktop)
    0x09, 0x02,       // Usage (Mouse)
    0xA1, 0x01,       // Collection (Application)
    0x85, BT_REPORT_ID_MOUSE,
    0x09, 0x01,       // Usage (Pointer)
    0xA1, 0x00,       // Collection (Physical)
    0x05, 0x09,       // Usage Page (Button)
    0x19, 0x01,       // Usage Minimum (Button 1)
    0x29, 0x03,       // Usage Maximum (Button 3)
    0x15, 0x00,       // Logical Minimum (0)
    0x25, 0x01,       // Logical Maximum (1)
    0x95, 0x03,       // Report Count (3)
    0x75, 0x01,       // Report Size (1)
    0x81, 0x02,       // Input (Data,Var,Abs)
    0x95, 0x01,       // Report Count (1)
    0x75, 0x05,       // Report Size (5)
    0x81, 0x01,       // Input (Const,Array,Abs)
    0x05, 0x01,       // Usage Page (Generic Desktop)
    0x09, 0x30,       // Usage (X)
    0x09, 0x31,       // Usage (Y)
    0x09, 0x38,       // Usage (Wheel)
    0x15, 0x81,       // Logical Minimum (-127)
    0x25, 0x7F,       // Logical Maximum (127)
    0x75, 0x08,       // Report Size (8)
    0x95, 0x03,       // Report Count (3)
    0x81, 0x06,       // Input (Data,Var,Rel)
    0xC0,             // End Collection
    0xC0              // End Collection
};

static void btFormatAddr(const uint8_t* addr, char* out, size_t out_len) {
    if (!out || out_len == 0) return;
    if (!addr) {
        out[0] = '\0';
        return;
    }
    snprintf(out, out_len, "%02X:%02X:%02X:%02X:%02X:%02X",
             addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
}

static esp_gatt_perm_t btReadPerm() {
    return ESP_GATT_PERM_READ_ENCRYPTED;
}

static void btSetStatusValue(const char* value) {
    if (!bt_status_char) return;
    bt_status_char->setValue(value ? value : "idle");
}

const char* btStatusShort() {
    if (!config_bt_enabled) return "off";
    if (bt_state == BT_STATE_ERROR) return "err";
    if (bt_connected) return "ok";
    if (bt_advertising) return "adv";
    if (bt_initialized) return "idle";
    return "off";
}

const char* btPeerAddress() {
    return bt_peer_addr;
}

bool btIsConnected() {
    return bt_connected;
}

bool btIsEnabled() {
    return config_bt_enabled;
}

bool btIsBonded() {
    return bt_bonded;
}

static bool btCanSendHid() {
    return bt_connected && bt_initialized && bt_kbd_input && bt_mouse_input;
}

static bool btSendKeyboardReport(uint8_t modifiers, uint8_t usage) {
    if (!btCanSendHid()) return false;
    uint8_t report[8] = { modifiers, 0x00, usage, 0x00, 0x00, 0x00, 0x00, 0x00 };
    bt_kbd_input->setValue(report, sizeof(report));
    bt_kbd_input->notify();
    return true;
}

static void btReleaseKeyboard() {
    if (!btCanSendHid()) return;
    uint8_t report[8] = { 0 };
    bt_kbd_input->setValue(report, sizeof(report));
    bt_kbd_input->notify();
}

bool btSendUsage(uint8_t usage, uint8_t modifiers) {
    if (!btSendKeyboardReport(modifiers, usage)) return false;
    delay(6);
    btReleaseKeyboard();
    return true;
}

bool btTypeChar(char c, uint8_t extra_modifiers) {
    uint8_t index = (uint8_t)c;
    if (index >= KEYMAP_SIZE) return false;
    const KEYMAP& m = keymap[index];
    if (m.usage == 0) return false;
    uint8_t modifiers = (uint8_t)(m.modifier | extra_modifiers);
    return btSendUsage(m.usage, modifiers);
}

size_t btTypeTextN(const char* text, size_t len, uint8_t extra_modifiers) {
    if (!text || len == 0) return 0;
    size_t typed = 0;
    for (size_t i = 0; i < len; i++) {
        if (!btTypeChar(text[i], extra_modifiers)) break;
        typed++;
    }
    return typed;
}

size_t btTypeText(const char* text) {
    if (!text) return 0;
    return btTypeTextN(text, strlen(text), 0);
}

bool btMouseMove(int8_t dx, int8_t dy, int8_t wheel) {
    if (!btCanSendHid()) return false;
    if (dx == 0 && dy == 0 && wheel == 0) return true;
    uint8_t report[4] = { 0x00, (uint8_t)dx, (uint8_t)dy, (uint8_t)wheel };
    bt_mouse_input->setValue(report, sizeof(report));
    bt_mouse_input->notify();
    return true;
}

bool btMouseClick(uint8_t buttons) {
    if (!btCanSendHid()) return false;
    uint8_t report[4] = { (uint8_t)(buttons & 0x07), 0x00, 0x00, 0x00 };
    bt_mouse_input->setValue(report, sizeof(report));
    bt_mouse_input->notify();
    delay(8);
    report[0] = 0;
    bt_mouse_input->setValue(report, sizeof(report));
    bt_mouse_input->notify();
    return true;
}

class BtServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) override {
        (void)pServer;
        bt_connected = true;
        bt_advertising = false;
        bt_state = BT_STATE_CONNECTED;
        btSetStatusValue("connected");
        render_requested = true;
        term_render_requested = true;
        SERIAL_LOGLN("BT: client connected");
    }

    void onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t* param) override {
        onConnect(pServer);
        if (param) {
            btFormatAddr(param->connect.remote_bda, bt_peer_addr, sizeof(bt_peer_addr));
            SERIAL_LOGF("BT: peer=%s\n", bt_peer_addr);
            esp_err_t err = esp_ble_set_encryption(param->connect.remote_bda, ESP_BLE_SEC_ENCRYPT_NO_MITM);
            if (err != ESP_OK) {
                SERIAL_LOGF("BT: set_encryption failed err=%d\n", (int)err);
            }
        }
    }

    void onDisconnect(BLEServer* pServer) override {
        (void)pServer;
        bt_connected = false;
        bt_advertising = false;
        bt_state = BT_STATE_OFF;
        btSetStatusValue("idle");
        render_requested = true;
        term_render_requested = true;
        SERIAL_LOGLN("BT: client disconnected");
    }

    void onDisconnect(BLEServer* pServer, esp_ble_gatts_cb_param_t* param) override {
        onDisconnect(pServer);
        if (param) {
            btFormatAddr(param->disconnect.remote_bda, bt_peer_addr, sizeof(bt_peer_addr));
        }
    }
};

class BtSecurityCallbacks : public BLESecurityCallbacks {
    uint32_t onPassKeyRequest() override {
        return 0;
    }

    void onPassKeyNotify(uint32_t pass_key) override {
        SERIAL_LOGF("BT: passkey notify %06u\n", (unsigned)pass_key);
    }

    bool onSecurityRequest() override {
        return true;
    }

    void onAuthenticationComplete(esp_ble_auth_cmpl_t auth_cmpl) override {
        bt_bonded = auth_cmpl.success;
        btFormatAddr(auth_cmpl.bd_addr, bt_peer_addr, sizeof(bt_peer_addr));
        if (auth_cmpl.success) {
            SERIAL_LOGF("BT: paired with %s\n", bt_peer_addr);
        } else {
            SERIAL_LOGF("BT: pair failed reason=0x%02X\n", auth_cmpl.fail_reason);
        }
        render_requested = true;
        term_render_requested = true;
    }

    bool onConfirmPIN(uint32_t pin) override {
        SERIAL_LOGF("BT: confirm pin %06u\n", (unsigned)pin);
        return true;
    }
};

static BtServerCallbacks bt_server_callbacks;
static BtSecurityCallbacks bt_security_callbacks;
static BLESecurity bt_security;

static void btStartAdvertising() {
    if (!bt_initialized || !config_bt_enabled || bt_connected || bt_scan_running) return;
    BLEDevice::startAdvertising();
    bt_advertising = true;
    bt_state = BT_STATE_ADVERTISING;
    btSetStatusValue("advertising");
    SERIAL_LOGLN("BT: advertising");
}

void btInit() {
    if (!config_bt_enabled || bt_initialized) {
        bt_state = config_bt_enabled ? bt_state : BT_STATE_OFF;
        return;
    }

    BLEDevice::init(config_bt_name);
    BLEDevice::setSecurityCallbacks(&bt_security_callbacks);

    bt_security.setCapability(ESP_IO_CAP_NONE);
    bt_security.setAuthenticationMode(ESP_LE_AUTH_REQ_SC_BOND);
    bt_security.setKeySize(16);
    bt_security.setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
    bt_security.setRespEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);

    bt_server = BLEDevice::createServer();
    if (!bt_server) {
        bt_state = BT_STATE_ERROR;
        SERIAL_LOGLN("BT: createServer failed");
        return;
    }
    bt_server->setCallbacks(&bt_server_callbacks);

    bt_hid = new BLEHIDDevice(bt_server);
    if (!bt_hid) {
        bt_state = BT_STATE_ERROR;
        SERIAL_LOGLN("BT: createHID failed");
        return;
    }

    bt_kbd_input = bt_hid->inputReport(BT_REPORT_ID_KEYBOARD);
    bt_kbd_output = bt_hid->outputReport(BT_REPORT_ID_KEYBOARD);
    bt_mouse_input = bt_hid->inputReport(BT_REPORT_ID_MOUSE);
    if (!bt_kbd_input || !bt_kbd_output || !bt_mouse_input) {
        bt_state = BT_STATE_ERROR;
        SERIAL_LOGLN("BT: create HID reports failed");
        return;
    }

    bt_hid->manufacturer()->setValue("LilyGo");
    bt_hid->pnp(0x02, 0xE502, 0xA111, 0x0210);
    bt_hid->hidInfo(0x00, 0x02);
    bt_hid->reportMap((uint8_t*)bt_hid_report_map, sizeof(bt_hid_report_map));
    bt_hid->startServices();
    bt_hid->setBatteryLevel(100);

    bt_status_service = bt_server->createService(BT_STATUS_SERVICE_UUID);
    if (!bt_status_service) {
        bt_state = BT_STATE_ERROR;
        SERIAL_LOGLN("BT: create status service failed");
        return;
    }

    bt_status_char = bt_status_service->createCharacteristic(BT_STATUS_CHAR_UUID, BLECharacteristic::PROPERTY_READ);
    if (!bt_status_char) {
        bt_state = BT_STATE_ERROR;
        SERIAL_LOGLN("BT: create status characteristic failed");
        return;
    }
    bt_status_char->setAccessPermissions(btReadPerm());
    btSetStatusValue("idle");
    bt_status_service->start();

    BLEAdvertising* adv = BLEDevice::getAdvertising();
    if (!adv) {
        bt_state = BT_STATE_ERROR;
        SERIAL_LOGLN("BT: getAdvertising failed");
        return;
    }
    adv->addServiceUUID(bt_hid->hidService()->getUUID());
    adv->addServiceUUID(bt_status_service->getUUID());
    adv->setAppearance(HID_KEYBOARD);
    adv->setScanResponse(true);
    adv->setMinPreferred(0x06);
    adv->setMaxPreferred(0x12);

    bt_initialized = true;
    bt_connected = false;
    bt_advertising = false;
    bt_bonded = false;
    bt_peer_addr[0] = '\0';

    btStartAdvertising();
    SERIAL_LOGF("BT: HID mode ready name=%s (encrypted bonding)\n", config_bt_name);
}

void btShutdown() {
    if (bt_scan_running) {
        BLEScan* scanner = BLEDevice::getScan();
        if (scanner) scanner->stop();
    }
    bt_scan_running = false;
    bt_scan_done = false;
    bt_scan_ok = false;
    bt_scan_total_found = 0;
    bt_scan_count = 0;
    bt_scan_resume_adv = false;
    bt_scan_restore_disabled = false;

    // Clear report/status characteristic pointers first so callbacks that may
    // fire during disconnect/deinit never touch freed BLE objects.
    bt_kbd_input = NULL;
    bt_mouse_input = NULL;
    bt_kbd_output = NULL;
    bt_status_char = NULL;

    if (bt_connected && bt_server) {
        bt_server->disconnect(bt_server->getConnId());
    }
    if (bt_initialized) {
        BLEDevice::stopAdvertising();
        delay(20);
        BLEDevice::deinit(false);
    }
    bt_server = NULL;
    bt_hid = NULL;
    bt_kbd_input = NULL;
    bt_mouse_input = NULL;
    bt_kbd_output = NULL;
    bt_status_service = NULL;
    bt_status_char = NULL;
    bt_initialized = false;
    bt_connected = false;
    bt_advertising = false;
    bt_state = BT_STATE_OFF;
}

bool btSetEnabled(bool enabled) {
    if (!enabled) {
        btShutdown();
        config_bt_enabled = false;
        bt_bonded = false;
        bt_peer_addr[0] = '\0';
        render_requested = true;
        term_render_requested = true;
        SERIAL_LOGLN("BT: disabled");
        return true;
    }

    config_bt_enabled = true;
    if (!bt_initialized) {
        btInit();
    } else {
        btStartAdvertising();
    }
    if (!bt_initialized) {
        config_bt_enabled = false;
        render_requested = true;
        term_render_requested = true;
        SERIAL_LOGLN("BT: enable failed");
        return false;
    }
    render_requested = true;
    term_render_requested = true;
    return true;
}

void btPoll() {
    if (!config_bt_enabled) return;
    if (!bt_initialized) {
        btInit();
        return;
    }
    if (bt_scan_running) return;
    if (!bt_connected && !bt_advertising) {
        btStartAdvertising();
    }
}

static void btScanCompleteCallback(BLEScanResults results) {
    bt_scan_total_found = results.getCount();
    bt_scan_count = 0;
    int limit = bt_scan_total_found;
    if (limit > BT_SCAN_MAX_RESULTS) limit = BT_SCAN_MAX_RESULTS;

    for (int i = 0; i < limit; i++) {
        BLEAdvertisedDevice dev = results.getDevice(i);
        BtScanEntry* dst = &bt_scan_results[i];
        memset(dst, 0, sizeof(*dst));
        dst->rssi = dev.getRSSI();

        std::string addr = dev.getAddress().toString();
        snprintf(dst->addr, sizeof(dst->addr), "%s", addr.c_str());

        if (dev.haveName()) {
            std::string name = dev.getName();
            snprintf(dst->name, sizeof(dst->name), "%s", name.c_str());
            dst->has_name = (dst->name[0] != '\0');
        } else {
            dst->has_name = false;
        }
        bt_scan_count++;
    }

    BLEScan* scanner = BLEDevice::getScan();
    if (scanner) scanner->clearResults();

    bt_scan_ok = true;
    bt_scan_done = true;
    bt_scan_running = false;
}

bool btScanStartAsync() {
    if (bt_scan_running) return false;
    if (bt_connected) return false;

    bt_scan_done = false;
    bt_scan_ok = false;
    bt_scan_total_found = 0;
    bt_scan_count = 0;
    bt_scan_resume_adv = false;
    bt_scan_restore_disabled = false;
    for (int i = 0; i < BT_SCAN_MAX_RESULTS; i++) {
        bt_scan_results[i].addr[0] = '\0';
        bt_scan_results[i].name[0] = '\0';
        bt_scan_results[i].rssi = 0;
        bt_scan_results[i].has_name = false;
    }

    if (!config_bt_enabled) {
        btSetEnabled(true);
        bt_scan_restore_disabled = true;
        delay(20);
    } else if (!bt_initialized) {
        btInit();
    }

    if (!bt_initialized) {
        if (bt_scan_restore_disabled) btSetEnabled(false);
        bt_scan_restore_disabled = false;
        return false;
    }

    bt_scan_resume_adv = bt_advertising;
    if (bt_scan_resume_adv) {
        BLEDevice::stopAdvertising();
        bt_advertising = false;
        bt_state = BT_STATE_OFF;
        btSetStatusValue("idle");
    }

    BLEScan* scanner = BLEDevice::getScan();
    if (!scanner) {
        if (bt_scan_resume_adv && config_bt_enabled && !bt_connected) btStartAdvertising();
        if (bt_scan_restore_disabled) btSetEnabled(false);
        bt_scan_resume_adv = false;
        bt_scan_restore_disabled = false;
        return false;
    }

    scanner->clearResults();
    scanner->setActiveScan(true);
    scanner->setInterval(160);
    scanner->setWindow(80);

    bt_scan_running = true;
    bool started = scanner->start(BT_SCAN_DURATION_S, btScanCompleteCallback, false);
    if (!started) {
        bt_scan_running = false;
        if (bt_scan_resume_adv && config_bt_enabled && !bt_connected) btStartAdvertising();
        if (bt_scan_restore_disabled) btSetEnabled(false);
        bt_scan_resume_adv = false;
        bt_scan_restore_disabled = false;
        return false;
    }
    return true;
}

bool btScanInProgress() {
    return bt_scan_running;
}

bool btScanTakeResults(BtScanEntry* out, int max_out, int* out_count, int* out_total, bool* out_ok) {
    if (max_out < 0 || (max_out > 0 && !out)) return false;
    if (bt_scan_running || !bt_scan_done) return false;

    int copied = bt_scan_count;
    if (copied > max_out) copied = max_out;
    for (int i = 0; i < copied; i++) {
        out[i] = bt_scan_results[i];
    }

    if (out_count) *out_count = copied;
    if (out_total) *out_total = bt_scan_total_found;
    if (out_ok) *out_ok = bt_scan_ok;

    if (bt_scan_resume_adv && config_bt_enabled && !bt_connected) {
        btStartAdvertising();
    }
    if (bt_scan_restore_disabled) {
        btSetEnabled(false);
    }

    bt_scan_done = false;
    bt_scan_ok = false;
    bt_scan_total_found = 0;
    bt_scan_count = 0;
    bt_scan_resume_adv = false;
    bt_scan_restore_disabled = false;
    return true;
}

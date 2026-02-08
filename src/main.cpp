#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <GxEPD2_BW.h>
#include <Adafruit_TCA8418.h>
#include <Fonts/FreeMono9pt7b.h>

// --- Pin Definitions ---

#define BOARD_I2C_SDA       13
#define BOARD_I2C_SCL       14

#define BOARD_SPI_SCK       36
#define BOARD_SPI_MOSI      33
#define BOARD_SPI_MISO      47

#define BOARD_EPD_CS        34
#define BOARD_EPD_DC        35
#define BOARD_EPD_BUSY      37
#define BOARD_EPD_RST       -1

#define BOARD_LORA_CS       3
#define BOARD_LORA_RST      4
#define BOARD_SD_CS         48

#define BOARD_KEYBOARD_INT  15
#define BOARD_KEYBOARD_LED  42

#define BOARD_LORA_EN       46
#define BOARD_GPS_EN        39
#define BOARD_1V8_EN        38
#define BOARD_6609_EN       41
#define BOARD_A7682E_PWRKEY 40

// --- Display ---

GxEPD2_BW<GxEPD2_310_GDEQ031T10, GxEPD2_310_GDEQ031T10::HEIGHT> display(
    GxEPD2_310_GDEQ031T10(BOARD_EPD_CS, BOARD_EPD_DC, BOARD_EPD_RST, BOARD_EPD_BUSY)
);

// --- Keyboard ---

Adafruit_TCA8418 keypad;

#define KEYPAD_ROWS 4
#define KEYPAD_COLS 10

#define IS_SHIFT(r, c)  ((r) == 3 && ((c) == 5 || (c) == 9))
#define IS_SYM(r, c)    ((r) == 3 && (c) == 8)
#define IS_ALT(r, c)    ((r) == 2 && (c) == 0)
#define IS_MIC(r, c)    ((r) == 3 && (c) == 6)
#define IS_DEAD(r, c)   ((r) == 3 && (c) <= 4)

static const char keymap_lower[KEYPAD_ROWS][KEYPAD_COLS] = {
    { 'q', 'w', 'e', 'r', 't', 'y', 'u', 'i', 'o', 'p' },
    { 'a', 's', 'd', 'f', 'g', 'h', 'j', 'k', 'l', '\b' },
    {  0,  'z', 'x', 'c', 'v', 'b', 'n', 'm', '\b', '\n' },
    {  0,   0,   0,   0,   0,   0,   0,  ' ',   0,   0 },
};

static const char keymap_upper[KEYPAD_ROWS][KEYPAD_COLS] = {
    { 'Q', 'W', 'E', 'R', 'T', 'Y', 'U', 'I', 'O', 'P' },
    { 'A', 'S', 'D', 'F', 'G', 'H', 'J', 'K', 'L', '\b' },
    {  0,  'Z', 'X', 'C', 'V', 'B', 'N', 'M', '\b', '\n' },
    {  0,   0,   0,   0,   0,   0,   0,  ' ',   0,   0 },
};

static const char keymap_sym[KEYPAD_ROWS][KEYPAD_COLS] = {
    { '#', '1', '2', '3', '(', ')', '_', '-', '+', '@' },
    { '*', '4', '5', '6', '/', ':', ';', '\'', '"', '\b' },
    {  0,  '7', '8', '9', '?', '!', ',', '.', '$', '\n' },
    {  0,   0,   0,   0,   0,   0,   0,  ' ',   0,   0 },
};

// --- Editor State (shared between cores, protected by mutex) ---

#define MAX_TEXT_LEN    4096
#define CHAR_W          11
#define CHAR_H          15
#define MARGIN_X        4
#define MARGIN_Y        4
#define SCREEN_W        240
#define SCREEN_H        320
#define STATUS_H        14
#define COLS_PER_LINE   ((SCREEN_W - MARGIN_X * 2) / CHAR_W)
#define ROWS_PER_SCREEN ((SCREEN_H - MARGIN_Y - STATUS_H) / CHAR_H)

static SemaphoreHandle_t state_mutex;
static volatile bool render_requested = false;

// Editor state — written by core 1 (keyboard), read by core 0 (display)
static char text_buf[MAX_TEXT_LEN + 1];
static int  text_len    = 0;
static int  cursor_pos  = 0;
static int  scroll_line = 0;
static bool shift_held  = false;
static bool sym_mode    = false;
static bool alt_mode    = false;

// Display task snapshot — private to core 0
static char snap_buf[MAX_TEXT_LEN + 1];
static int  snap_len       = 0;
static int  snap_cursor    = 0;
static int  snap_scroll    = 0;
static bool snap_shift     = false;
static bool snap_sym       = false;
static bool snap_alt       = false;

// --- Helpers ---

struct LayoutInfo {
    int total_lines;
    int cursor_line;
    int cursor_col;
};

LayoutInfo computeLayoutFrom(const char* buf, int len, int cpos) {
    LayoutInfo info = {0, 0, 0};
    int line = 0, col = 0;

    for (int i = 0; i < len; i++) {
        if (i == cpos) {
            info.cursor_line = line;
            info.cursor_col  = col;
        }
        if (buf[i] == '\n') {
            line++; col = 0;
        } else {
            col++;
            if (col >= COLS_PER_LINE) { line++; col = 0; }
        }
    }
    if (cpos == len) {
        info.cursor_line = line;
        info.cursor_col  = col;
    }
    info.total_lines = line + 1;
    return info;
}

// --- Display Rendering (runs on core 0 only, uses snap_ vars) ---

static int partial_count = 0;

void drawLinesRange(int first_line, int last_line) {
    int text_line = 0, col = 0;

    for (int i = 0; i <= snap_len; i++) {
        int sl = text_line - snap_scroll;

        if (i == snap_cursor && sl >= first_line && sl <= last_line) {
            int x = MARGIN_X + col * CHAR_W;
            int y = MARGIN_Y + sl * CHAR_H;
            display.fillRect(x, y, CHAR_W, CHAR_H, GxEPD_BLACK);
            if (i < snap_len && snap_buf[i] != '\n') {
                display.setTextColor(GxEPD_WHITE);
                display.setCursor(x, y + CHAR_H - 3);
                display.print(snap_buf[i]);
                display.setTextColor(GxEPD_BLACK);
            }
        }

        if (i >= snap_len) break;
        char c = snap_buf[i];
        if (c == '\n') { text_line++; col = 0; continue; }

        if (sl >= first_line && sl <= last_line && i != snap_cursor) {
            int x = MARGIN_X + col * CHAR_W;
            int y = MARGIN_Y + sl * CHAR_H + CHAR_H - 3;
            display.setCursor(x, y);
            display.print(c);
        }

        col++;
        if (col >= COLS_PER_LINE) { text_line++; col = 0; }

        if (text_line - snap_scroll > last_line && i != snap_cursor) break;
    }
}

void drawStatusBar() {
    LayoutInfo info = computeLayoutFrom(snap_buf, snap_len, snap_cursor);
    int bar_y = SCREEN_H - STATUS_H;
    display.fillRect(0, bar_y, SCREEN_W, STATUS_H, GxEPD_BLACK);
    display.setTextColor(GxEPD_WHITE);
    display.setFont(&FreeMono9pt7b);
    display.setCursor(2, SCREEN_H - 3);
    char status[50];
    snprintf(status, sizeof(status), "L%d C%d %dch %s%s%s",
             info.cursor_line + 1, info.cursor_col + 1, snap_len,
             snap_shift ? "[SH]" : "",
             snap_sym ? "[SYM]" : "",
             snap_alt ? "[NAV]" : "");
    display.print(status);
}

void refreshLines(int first_line, int last_line) {
    if (first_line < 0) first_line = 0;
    if (last_line >= ROWS_PER_SCREEN) last_line = ROWS_PER_SCREEN - 1;

    int y_start = MARGIN_Y + first_line * CHAR_H;
    // Extend window from content lines all the way down to include status bar
    // This combines two partial refreshes into one, saving ~700ms BUSY wait
    int region_h = SCREEN_H - y_start;

    partial_count++;

    display.setPartialWindow(0, y_start, SCREEN_W, region_h);
    display.firstPage();
    do {
        display.fillScreen(GxEPD_WHITE);
        display.setTextColor(GxEPD_BLACK);
        display.setFont(&FreeMono9pt7b);
        drawLinesRange(first_line, ROWS_PER_SCREEN - 1);
        drawStatusBar();
    } while (display.nextPage());
}

void refreshAllPartial() {
    partial_count++;

    display.setPartialWindow(0, 0, SCREEN_W, SCREEN_H);
    display.firstPage();
    do {
        display.fillScreen(GxEPD_WHITE);
        display.setTextColor(GxEPD_BLACK);
        display.setFont(&FreeMono9pt7b);
        drawLinesRange(0, ROWS_PER_SCREEN - 1);
        drawStatusBar();
    } while (display.nextPage());
}

void refreshFullClean() {
    partial_count = 0;
    display.setFullWindow();
    display.firstPage();
    do {
        display.fillScreen(GxEPD_WHITE);
        display.setTextColor(GxEPD_BLACK);
        display.setFont(&FreeMono9pt7b);
        drawLinesRange(0, ROWS_PER_SCREEN - 1);
        drawStatusBar();
    } while (display.nextPage());
}

// Take a snapshot of shared state (called with mutex held)
void snapshotState() {
    memcpy(snap_buf, text_buf, text_len + 1);
    snap_len    = text_len;
    snap_cursor = cursor_pos;
    snap_scroll = scroll_line;
    snap_shift  = shift_held;
    snap_sym    = sym_mode;
    snap_alt    = alt_mode;
}

// --- Display Task (Core 0) ---

// Previous snapshot layout for smart diffing
static LayoutInfo prev_layout = {1, 0, 0};

void displayTask(void* param) {
    // Initial full refresh
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    snapshotState();
    xSemaphoreGive(state_mutex);
    refreshFullClean();
    prev_layout = computeLayoutFrom(snap_buf, snap_len, snap_cursor);

    for (;;) {
        // Wait until keyboard core signals a change
        if (!render_requested) {
            vTaskDelay(1);  // yield, ~1ms check interval
            continue;
        }
        render_requested = false;

        // Snapshot state (hold mutex very briefly)
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        snapshotState();
        xSemaphoreGive(state_mutex);

        // Compute layout from snapshot
        LayoutInfo cur = computeLayoutFrom(snap_buf, snap_len, snap_cursor);

        // Ensure cursor visible (updates snap_scroll)
        if (cur.cursor_line < snap_scroll) {
            snap_scroll = cur.cursor_line;
        }
        if (cur.cursor_line >= snap_scroll + ROWS_PER_SCREEN) {
            snap_scroll = cur.cursor_line - ROWS_PER_SCREEN + 1;
        }
        // Write scroll back so keyboard core knows
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        scroll_line = snap_scroll;
        xSemaphoreGive(state_mutex);

        bool scrolled = (snap_scroll != (prev_layout.cursor_line < ROWS_PER_SCREEN ?
                         0 : prev_layout.cursor_line - ROWS_PER_SCREEN + 1));

        // Deghost periodically
        if (partial_count >= 30) {
            refreshFullClean();
            prev_layout = cur;
            continue;
        }

        // Check if scroll position changed significantly
        // Simple heuristic: if cursor jumped more than ROWS_PER_SCREEN/2 lines, full refresh
        int line_delta = abs(cur.cursor_line - prev_layout.cursor_line);
        if (line_delta > ROWS_PER_SCREEN) {
            refreshAllPartial();
            prev_layout = cur;
            continue;
        }

        int old_sl = prev_layout.cursor_line - snap_scroll;
        int new_sl = cur.cursor_line - snap_scroll;

        if (cur.total_lines != prev_layout.total_lines) {
            int from = min(old_sl, new_sl);
            if (from < 0) from = 0;
            refreshLines(from, ROWS_PER_SCREEN - 1);
        } else {
            int min_l = min(old_sl, new_sl);
            int max_l = max(old_sl, new_sl);
            if (min_l < 0) min_l = 0;
            refreshLines(min_l, max_l);
        }

        prev_layout = cur;
    }
}

// --- Keyboard Input (Core 1) ---

void cursorLeft()  { if (cursor_pos > 0) cursor_pos--; }
void cursorRight() { if (cursor_pos < text_len) cursor_pos++; }

void cursorUp() {
    LayoutInfo info = computeLayoutFrom(text_buf, text_len, cursor_pos);
    if (info.cursor_line == 0) return;
    int target_line = info.cursor_line - 1;
    int target_col  = info.cursor_col;
    int line = 0, col = 0;
    for (int i = 0; i <= text_len; i++) {
        if (line == target_line && col == target_col) { cursor_pos = i; return; }
        if (i >= text_len) break;
        if (text_buf[i] == '\n') {
            if (line == target_line) { cursor_pos = i; return; }
            line++; col = 0;
        } else {
            col++;
            if (col >= COLS_PER_LINE) { line++; col = 0; }
        }
    }
}

void cursorDown() {
    LayoutInfo info = computeLayoutFrom(text_buf, text_len, cursor_pos);
    if (info.cursor_line >= info.total_lines - 1) return;
    int target_line = info.cursor_line + 1;
    int target_col  = info.cursor_col;
    int line = 0, col = 0;
    for (int i = 0; i <= text_len; i++) {
        if (line == target_line && col == target_col) { cursor_pos = i; return; }
        if (i >= text_len) break;
        if (text_buf[i] == '\n') {
            if (line == target_line) { cursor_pos = i; return; }
            line++; col = 0;
        } else {
            col++;
            if (col >= COLS_PER_LINE) { line++; col = 0; }
        }
    }
    cursor_pos = text_len;
}

// Returns true if display needs updating
bool handleKeyPress(int event_code) {
    int key_num = (event_code & 0x7F);
    int idx = key_num - 1;
    int row = idx / KEYPAD_COLS;
    int col_raw = idx % KEYPAD_COLS;
    int col_rev = (KEYPAD_COLS - 1) - col_raw;

    if (row < 0 || row >= KEYPAD_ROWS || col_rev < 0 || col_rev >= KEYPAD_COLS) return false;

    if (IS_SHIFT(row, col_rev)) { shift_held = !shift_held; return false; }
    if (IS_SYM(row, col_rev))   { sym_mode = !sym_mode; return true; }
    if (IS_ALT(row, col_rev))   { alt_mode = !alt_mode; return true; }
    if (IS_MIC(row, col_rev))   { return false; }
    if (IS_DEAD(row, col_rev))  { return false; }

    if (alt_mode) {
        char base = keymap_lower[row][col_rev];
        if (base == 'r')      cursorUp();
        else if (base == 'd') cursorLeft();
        else if (base == 'g') cursorDown();
        else if (base == 'c') cursorRight();
        else return false;
        return true;
    }

    char c;
    if (sym_mode)        c = keymap_sym[row][col_rev];
    else if (shift_held) c = keymap_upper[row][col_rev];
    else                 c = keymap_lower[row][col_rev];

    if (c == 0) return false;

    if (c == '\b') {
        if (cursor_pos > 0) {
            memmove(&text_buf[cursor_pos - 1], &text_buf[cursor_pos], text_len - cursor_pos);
            text_len--;
            cursor_pos--;
            text_buf[text_len] = '\0';
            return true;
        }
        return false;
    }

    if (c == '\n' || (c >= ' ' && c <= '~')) {
        if (text_len < MAX_TEXT_LEN) {
            memmove(&text_buf[cursor_pos + 1], &text_buf[cursor_pos], text_len - cursor_pos);
            text_buf[cursor_pos] = c;
            text_len++;
            cursor_pos++;
            text_buf[text_len] = '\0';
            if (shift_held) shift_held = false;
            return true;
        }
    }

    return false;
}

// --- Setup & Loop ---

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("T-Deck Pro Notepad starting...");

    // Disable unused peripherals
    pinMode(BOARD_LORA_EN, OUTPUT);       digitalWrite(BOARD_LORA_EN, LOW);
    pinMode(BOARD_GPS_EN, OUTPUT);        digitalWrite(BOARD_GPS_EN, LOW);
    pinMode(BOARD_1V8_EN, OUTPUT);        digitalWrite(BOARD_1V8_EN, LOW);
    pinMode(BOARD_6609_EN, OUTPUT);       digitalWrite(BOARD_6609_EN, LOW);
    pinMode(BOARD_A7682E_PWRKEY, OUTPUT); digitalWrite(BOARD_A7682E_PWRKEY, LOW);

    // Keyboard backlight off
    pinMode(BOARD_KEYBOARD_LED, OUTPUT);
    digitalWrite(BOARD_KEYBOARD_LED, LOW);

    // SPI CS lines high
    pinMode(BOARD_LORA_CS, OUTPUT);  digitalWrite(BOARD_LORA_CS, HIGH);
    pinMode(BOARD_LORA_RST, OUTPUT); digitalWrite(BOARD_LORA_RST, HIGH);
    pinMode(BOARD_SD_CS, OUTPUT);    digitalWrite(BOARD_SD_CS, HIGH);
    pinMode(BOARD_EPD_CS, OUTPUT);   digitalWrite(BOARD_EPD_CS, HIGH);

    // Init I2C
    Wire.begin(BOARD_I2C_SDA, BOARD_I2C_SCL);

    // Init keyboard
    if (!keypad.begin(0x34, &Wire)) {
        Serial.println("ERROR: TCA8418 keyboard not found!");
    } else {
        keypad.matrix(KEYPAD_ROWS, KEYPAD_COLS);
        keypad.flush();
        Serial.println("Keyboard OK");
    }

    // Init SPI & e-paper
    SPI.begin(BOARD_SPI_SCK, BOARD_SPI_MISO, BOARD_SPI_MOSI);
    display.init(115200, true, 2, false);
    display.setRotation(0);

    // Boost SPI clock from default 4MHz to 20MHz for faster data transfer
    display.epd2.selectSPI(SPI, SPISettings(20000000, MSBFIRST, SPI_MODE0));

    memset(text_buf, 0, sizeof(text_buf));

    // Create mutex
    state_mutex = xSemaphoreCreateMutex();

    // Launch display task on core 0 (Arduino loop runs on core 1)
    xTaskCreatePinnedToCore(
        displayTask,    // function
        "display",      // name
        8192,           // stack size
        NULL,           // parameter
        1,              // priority
        NULL,           // task handle
        0               // core 0
    );

    Serial.println("Ready. Start typing!");
}

// Core 1: keyboard polling — never blocks on display
void loop() {
    while (keypad.available() > 0) {
        int ev = keypad.getEvent();
        if (!(ev & 0x80)) continue;  // skip release events

        xSemaphoreTake(state_mutex, portMAX_DELAY);
        bool needs_render = handleKeyPress(ev);
        xSemaphoreGive(state_mutex);

        if (needs_render) {
            render_requested = true;
        }
    }
}

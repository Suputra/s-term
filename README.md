# T-Deck Pro Notepad + SSH Terminal

Firmware for the LilyGo T-Deck Pro that turns it into a pocket notepad and SSH terminal over WiFi.

## Hardware

- LilyGo T-Deck Pro (ESP32-S3, 3.1" e-paper display, integrated keyboard)

## Prerequisites

Install PlatformIO Core (CLI):

```bash
# Option 1: pip
python3 -m venv ~/.pio-venv
source ~/.pio-venv/bin/activate
pip install platformio

# Option 2: installer script
curl -fsSL -o get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
python3 get-platformio.py
```

## Configuration

Edit the credential defines at the top of `src/main.cpp`:

```cpp
#define WIFI_SSID     "YOUR_SSID"
#define WIFI_PASSWORD "YOUR_PASSWORD"

#define SSH_HOST      "192.168.1.100"
#define SSH_PORT      22
#define SSH_USER      "user"
#define SSH_PASS      "password"
```

## Build & Flash

```bash
# Build only
pio run

# Build and flash (connect T-Deck Pro via USB-C)
pio run -t upload

# Monitor serial output
pio device monitor
```

PlatformIO automatically downloads the ESP32-S3 toolchain and all library dependencies on first build.

## Usage

### Notepad Mode (default)
- Type normally on the keyboard
- **Shift** toggles uppercase (sticky)
- **Sym** toggles symbol layer (numbers, punctuation)
- **Alt + R/D/G/C** arrow keys (up/left/down/right)
- **MIC** switch to terminal mode

### Terminal Mode
- **MIC** switch back to notepad
- WiFi connects automatically on entry
- **Alt+S** connect SSH
- **Alt+Q** disconnect SSH
- **Alt+W** reconnect WiFi
- **Alt + R/G/C/D** arrow keys (shell history, cursor movement)
- Type normally -- keystrokes are sent to the remote shell (no local echo)
- **Enter** sends command, **Backspace** works as expected

The terminal strips ANSI escape sequences, so basic shell commands (`ls`, `cd`, `cat`, `grep`, `python`) work well. Full-screen programs like `vim` or `htop` will not render correctly on e-paper.

## Project Structure

```
tdeck-notepad/
  platformio.ini    # Board, dependencies, build flags
  src/main.cpp      # All application code (~700 lines)
```

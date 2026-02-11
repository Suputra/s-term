# T-Deck Pro Notepad

Firmware for the LilyGo T-Deck Pro — a pocket notepad with SSH terminal, file manager and WireGuard VPN.

## Build & Flash

Requires [PlatformIO](https://platformio.org/). All dependencies are fetched automatically on first build.

### Quick setup (`uv`, no virtualenv)

Run this once:

```bash
uv tool install --force platformio && uv tool update-shell && export PATH="$(uv tool dir --bin):$PATH" && hash -r
```

After that, use plain `pio` commands:

```bash
pio run              # build
pio run -t upload    # flash via USB-C
pio device monitor   # serial output
```

Alternative setup script:

```bash
source scripts/setup-pio.sh
```

## SD Card Config

Credentials are loaded from a `/CONFIG` file on the SD card (FAT32). Section-based format with `#` comments and blank lines ignored. Section headers are `# wifi`, `# ssh`, `# vpn`.

```
# wifi (SSID/password pairs; blank or missing password means open WiFi)
home_ssid
home_password
office_guest_open

phone_hotspot
hotspot_password

# ssh
10.0.0.100
22
user
password
# optional: host to use only when SSH is attempted via VPN
# (useful if direct host is a local shortname that doesn't resolve over tunnel DNS)
10.207.162.10

# vpn (omit section if not using VPN)
<device_private_key_base64>
<server_public_key_base64>
<preshared_key_base64>
<device_vpn_ip>
<endpoint_host_or_ip>
51820
# optional: DNS server to use while VPN is active
10.0.0.1
```

Open WiFi entries: put only the SSID and leave the password line blank (or end the WiFi section right after the SSID).

## Usage

### Notepad (default mode)

Type on the keyboard. Text wraps to the e-ink display.

- **Shift** — sticky uppercase (one letter)
- **Sym** — one-shot symbol/number layer
- **Alt** — toggles nav mode (WASD = arrows, backspace = delete forward)

### Terminal

Double-tap **MIC** to switch to terminal mode. The device connects WiFi, tries SSH directly, and if the host isn't reachable falls back through WireGuard VPN automatically. Double-tap **MIC** again to return to notepad.

- **Alt** — acts as ctrl - alt + space -> esc

### Command Processor

Single-tap **MIC** from either mode to open the command prompt (bottom half of screen).

| Command | Description |
|---------|-------------|
| `l` / `ls` | List files on SD card |
| `e` / `edit [file]` | Edit a file (loads into notepad). With no filename, opens interactive picker (W/S move, A/D page, Enter open). |
| `w` / `save` | Save notepad to current file |
| `n` / `new` | New file (auto-saves current) |
| `r` / `rm <file>` | Delete a file |
| `u` / `upload` | SCP all SD files to `~/tdeck` on SSH host |
| `d` / `download` | SCP `~/tdeck` files to SD card |
| `p` / `paste` | Paste notepad to SSH |
| `dc` | Disconnect SSH |
| `ws` / `scan` | Scan WiFi and retry known APs manually |
| `f` / `refresh` | Force full e-ink refresh |
| `4` / `4g` | Toggle 4G modem |
| `s` / `status` | Show WiFi/SSH/VPN/battery status |
| `?` / `h` / `help` | Show help |

### 4G Modem

**Alt+M** toggles the A7682E cellular modem (off by default). Currently supports AT commands and network registration. PPPoS for SSH-over-cellular is planned.

## Architecture

Single-file firmware (`src/main.cpp`). Two FreeRTOS cores:

- **Core 0** — e-ink display rendering
- **Core 1** — keyboard polling, WiFi/SSH/VPN, file I/O

SPI bus shared between e-ink and SD card via cooperative `sd_busy`/`display_idle` flags (no mutex).

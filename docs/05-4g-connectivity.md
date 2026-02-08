# 4G Connectivity via A7682E Modem

## Status: RESEARCH NEEDED

## Hardware
- SIMCom A7682E cellular modem on the T-Deck Pro board
- Control pins: BOARD_A7682E_PWRKEY (GPIO 40), BOARD_6609_EN (GPIO 41)
- Currently disabled in setup() (both pins LOW)
- Communicates over UART (need to identify TX/RX pins)

## Research TODOs
- [ ] Find T-Deck Pro schematic for A7682E UART pins
- [ ] Determine SIM card slot location and supported bands
- [ ] Test AT command communication (AT, AT+CPIN?, AT+CSQ, AT+CGATT?)
- [ ] Research PPP over serial for TCP/IP connectivity
- [ ] Evaluate TinyGSM library for ESP32 + A7682E

## Approach Options
1. **PPP mode**: Modem provides raw IP via PPP, ESP32 uses lwIP stack normally.
   SSH library works unchanged. Best option if supported.
2. **AT+TCP mode**: Use modem's built-in TCP stack via AT commands.
   Would need to bypass libssh's socket layer. Complex.
3. **TinyGSM + SSLClient**: Use TinyGSM library for AT-command-based networking.
   May not integrate well with libssh which expects POSIX sockets.

## Implementation Plan (once research done)
1. Power on modem, wait for registration
2. Establish PPP connection
3. Use existing WiFi.localIP()-style networking for SSH
4. Add modem status to status bar
5. Fallback: WiFi → 4G priority

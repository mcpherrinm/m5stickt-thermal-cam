# M5 Thermal

Firmware for the M5Stack **M5StickT**: an M5StickC (ESP32-PICO-D4) with a 240×135 ST7789
screen and a FLIR Lepton 3.x thermal camera (160×120). This unit has a Lepton 3.0
(part 500-0726-01), which is not radiometric, so temperatures are estimates.

- The LCD shows the live thermal image (Ironbow palette, auto-ranged) and a battery icon.
- The stick joins your Wi-Fi network (set in `secrets.ini`, DHCP or a static address).
  Open <http://m5-thermal.local/>, or its IP address from the boot log, for the web viewer:
  a display matching the LCD, tap-to-probe, palettes, manual range, °C/°F, mirror/rotate,
  fullscreen, and PNG/video/CSV capture.
- The button on top freezes the frame (on the LCD and for web viewers); press again to
  resume. Turning the thumbwheel shows the network status for a few seconds.
- A long press of the power button turns the stick off, as with the stock firmware.

## Build and flash

```sh
python3 -m venv .venv && .venv/bin/pip install platformio
cp secrets.ini.example secrets.ini      # set wifi_ssid, wifi_password (and static_ip)
.venv/bin/pio run -t upload             # ~1 min at 115200 baud
.venv/bin/pio device monitor            # boot log with the IP address, then stats every 10 s
```

## Finding cameras over USB

`index.html` (at the repo root, for GitHub Pages) uses WebSerial to find cameras plugged
into the computer. It reads each camera's network status (network, IP, signal, viewers),
shows its live log, takes single snapshots over USB (no Wi-Fi needed; about 2.5 s each,
downloadable as PNG or CSV), and links to the camera's viewer. Browsers block embedding the
camera's plain HTTP page in an HTTPS page, so from GitHub Pages the viewer opens in a new
tab; served over HTTP (e.g. `python3 -m http.server`, then <http://localhost:8000/>) it's
embedded too. The page leaves the serial control lines alone, so connecting doesn't restart
the camera.

Serial commands (115200 baud, newline-terminated):

- `status` → one `@status {json}` line (also printed whenever the Wi-Fi connection changes).
- `snapshot` → one `@frame <bytes> <crc32 hex> <base64>` line carrying a frame message in the
  WebSocket format below, or `@frame-error <reason>`. It shows what's on screen (the frozen
  frame, if frozen). Web viewers pause while it's sent. Other log output can occasionally
  land inside the line; check the CRC and retry.

## Layout

- `src/main.cpp`: Lepton capture task (CCI over I²C, VoSPI over SPI), LCD, battery,
  Wi-Fi and WebSocket stream
- `web/index.html`: viewer page, embedded into the firmware at build time
- `index.html`: WebSerial page for finding cameras and reading their logs

Kept locally and not committed (see `.gitignore`):

- `stock/`: M5Stack's original StickT Lepton example, building with today's toolchain
  (only change: two missing `return` statements that newer GCC turns into crashes)
- `backup/`: full flash dump of the firmware the device shipped with

## Hardware notes

| Part                 | Connection                                  |
|----------------------|---------------------------------------------|
| Lepton CCI (control) | I²C 0x2A on the internal bus (G21/G22)      |
| Lepton VoSPI (video) | SPI: SCK G2, MISO G25, CS G0 (mode 3, 20 MHz) |
| Lepton VSYNC         | G38 (Lepton GPIO3, enabled over CCI)        |
| Lepton reset         | G26                                         |
| Thumbwheel encoder   | I²C 0x30 on the internal bus (unused here)  |

The Lepton won't boot if anything else talks on its I²C bus while it starts up, so the
firmware holds the bus for the second after the reset pulse.

## WebSocket frame format (`/ws`, binary, little-endian)

| Offset | Type | Field                                                     |
|-------:|------|-----------------------------------------------------------|
| 0      | u8   | version: 2 = raw pixels, 3 = delta-encoded pixels         |
| 1      | u8   | flags: bit0 sensor ok, bit1 charging, bit2 USB power, bit3 frozen |
| 2      | u8   | battery %                                                 |
| 3      | u8   | reserved                                                  |
| 4      | u16  | battery mV                                                |
| 6      | u16  | frame sequence                                            |
| 8      | u16  | width (160, or 0 for a status-only message)               |
| 10     | u16  | height (120, or 0)                                        |
| 12     | f32  | scale                                                     |
| 16     | f32  | offset: °C = raw × scale + offset                         |
| 20     | …    | pixels, row-major, top-left first                         |

Version 2 pixels are u16 raw values. Version 3 (normally used) stores each pixel as
the difference from its left neighbour, or from the pixel above at the start of a
row (the first pixel is relative to 0). Each difference `d` is zigzag-encoded,
`z = (d << 1) ^ (d >> 31)`, and written as one byte if `z < 0x80`, otherwise as two
bytes `0x80 | z >> 8, z & 0xFF`. Thermal images are smooth, so frames are about half
the size of raw ones, which keeps the ESP32's Wi-Fi stack streaming at full rate.

Viewers share one send buffer. A viewer that stops reading for 3 s (for example a
backgrounded browser tab) is disconnected so it doesn't stall the others; the web
page reconnects automatically.

## Restoring the original firmware

```sh
.venv/bin/pio pkg exec -p tool-esptoolpy -- esptool.py --port /dev/cu.usbserial-5D52DC9912 \
  --baud 115200 write_flash 0 backup/m5stickc-flash-full-4MB.bin
```

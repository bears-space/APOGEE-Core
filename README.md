# APOGEE Core

[![Frontend and Firmware CI](https://github.com/bears-space/APOGEE-Core/actions/workflows/esp-build.yml/badge.svg)](https://github.com/bears-space/APOGEE-Core/actions/workflows/esp-build.yml)
[![pre-commit](https://github.com/bears-space/APOGEE-Core/actions/workflows/pre-commit.yaml/badge.svg)](https://github.com/bears-space/APOGEE-Core/actions/workflows/pre-commit.yaml)

Firmware for the **APOGEE Core module** from [BEARS](https://bears-space.de), targeting **ESP32-P4 and ESP32-S3-Mini**. This repository is based on [Vigilant Engine](https://github.com/bears-space/vigilant-engine) and contains the application, shared device services, an embedded Vue dashboard, and a separate recovery firmware.

## Current scope

The application in [`main/main.c`](main/main.c) currently initializes Vigilant Engine with the name `Vigilant ESP Test` and the `APSTA` network mode. It is a foundation for module-specific functionality; it does not yet implement a P4–S3 communication link or application-level sensor acquisition.

The framework currently provides:

- Wi-Fi AP, station, and combined AP+STA modes on chips with native Wi-Fi.
- An embedded web dashboard with device information, I²C bus information, and live logs over WebSocket.
- Configurable I²C initialization, bus scanning, device registration, and register access.
- Configurable WS2812B status LED support.
- A factory recovery application that accepts firmware uploads into `ota_0` on Wi-Fi-capable targets.

### Target differences

| Target | Current behavior |
| --- | --- |
| ESP32-S3 / S3-Mini | Native Wi-Fi, dashboard access, and Wi-Fi recovery firmware. |
| ESP32-P4 | Wi-Fi initialization and the Wi-Fi recovery build are skipped. The current application does not initialize Ethernet or a Wi-Fi coprocessor, so starting the HTTP server alone does not make the dashboard reachable. |
| ESP32-C6 | Included in the inherited CI build matrix; verify board-specific pins before using it. |

[CI](.github/workflows/esp-build.yml) builds all three targets with ESP-IDF v6.0. Build coverage is not a hardware validation claim.

## Getting started

### Prerequisites

- **ESP-IDF v6.0**, matching CI, with tools installed for your target.
- **Node.js 22 and npm**, matching the frontend CI environment.
- **Git**, including access to the sensor SDK submodule.
- A compatible board and a serial connection for the initial flash.
- Optional: **uv** for the Python development tools and documentation. [`pyproject.toml`](pyproject.toml) requires Python 3.13 or newer.

Use an exported ESP-IDF shell, or the ESP-IDF extension in VS Code. On Linux/macOS, source your ESP-IDF installation before building:

```sh
. /path/to/esp-idf/export.sh
```

### Clone

```sh
git clone --recurse-submodules https://github.com/bears-space/APOGEE-Core.git
cd APOGEE-Core
```

For an existing checkout:

```sh
git submodule update --init --recursive
```

The sensor SDK URL in [`.gitmodules`](.gitmodules) uses SSH (`git@github.com:bears-space/Sensors-SDK_ESP32.git`). Submodule initialization therefore needs working GitHub SSH access even when the main repository was cloned over HTTPS.

### Configure and build

Select the chip you are flashing, then review the board configuration:

```sh
idf.py set-target esp32p4
idf.py menuconfig
idf.py build
```

Use `idf.py set-target esp32s3` for the S3-Mini. Target selection regenerates the SDK configuration; recheck your settings when switching targets.

The default CMake flow builds missing web assets through npm. Keep Node.js and npm available in the same shell as ESP-IDF. The firmware retains the upstream output name:

```text
build/vigilant-engine.bin
```

For Wi-Fi-capable targets, the build also produces:

```text
build/recovery/vigilant-engine-recovery.bin
```

### Flash and monitor

Replace the example port with your device's serial port:

```sh
idf.py -p /dev/ttyACM0 flash monitor
```

The top-level build places the main firmware in `ota_0` and, on Wi-Fi-capable targets, the recovery firmware in `factory`. Use the top-level `flash` target for initial provisioning so the bootloader, partition table, and applicable firmware images are flashed together. Exit the serial monitor with **Ctrl+]**.

If the board starts in recovery, use its **Boot** action to select the main application. You can also explicitly select `ota_0` over serial after flashing:

```sh
python "$IDF_PATH/components/app_update/otatool.py" --port /dev/ttyACM0 switch_ota_partition --slot 0
```

Reset the board after changing the boot selection over serial.

## Board configuration

Review [`sdkconfig.defaults`](sdkconfig.defaults) and the **Vigilant Engine Configuration** menus before flashing. The checked-in defaults are not a universal pinout for every supported target.

| Setting | Checked-in/default value | Where to change it |
| --- | --- | --- |
| Flash size | 4 MB | ESP-IDF flash configuration; `sdkconfig.defaults` |
| Partition table | `partitions.csv` | ESP-IDF partition table configuration |
| I²C enabled | Yes | `CONFIG_VE_ENABLE_I2C` |
| I²C SCL / SDA | GPIO 33 / GPIO 32 | `CONFIG_VE_I2C_SCL_IO` / `CONFIG_VE_I2C_SDA_IO` |
| I²C frequency | 100 kHz | `CONFIG_VE_I2C_FREQ_HZ` |
| Status LED | Enabled, GPIO 8 | `CONFIG_VE_ENABLE_STATUS_LED` / `CONFIG_VE_STATUS_WS2812B_PIN` |
| Main Wi-Fi mode | AP+STA | `VigilantConfig.network_mode` in `main/main.c` |
| Main STA SSID / password | `starstreak` / `starstreak` | `CONFIG_VE_STA_SSID` / `CONFIG_VE_STA_PASSWORD` |
| Main AP prefix / password | `starstreak-` / `starstreak` | `CONFIG_VE_AP_SSID_PREFIX` / `CONFIG_VE_AP_PASSWORD` |
| Recovery mode | AP | Recovery menu |
| Recovery AP prefix / password | `VE-Recovery-` / `starstreak` | Recovery menu |
| Embedded main dashboard | Enabled | `CONFIG_VE_DISABLE_FRONTEND` |

The AP SSIDs append the last two MAC-address bytes as four hexadecimal characters. Configure main and recovery network credentials separately.

The P4 defaults select silicon revisions below v3 and a 360 MHz CPU frequency. Match the revision settings to your actual chip; see the [revision troubleshooting notes](docs/troubleshooting.md#esp32p4-chip-revision-mismatch).

## Dashboard and API

On a Wi-Fi-capable target, connect to the configured AP or use the station IP shown in the serial logs, then open `http://<device-ip>/`.

The main application exposes:

| Method | Path | Purpose |
| --- | --- | --- |
| GET | `/` | Embedded dashboard, or a placeholder when frontend embedding is disabled. |
| GET | `/info` | Device name, network mode, MAC address, SSIDs, and IP addresses. |
| GET | `/i2cinfo` | I²C configuration, registered devices, and detected addresses. |
| WebSocket | `/ws` | Log history and live log streaming; accepts `{"type":"get-logs"}` and `{"type":"ping"}`. |
| GET | `/rebootfactory` | Reboot into recovery; returns HTTP 501 on targets without native Wi-Fi. |

Example read-only requests:

```sh
curl http://<device-ip>/info
curl http://<device-ip>/i2cinfo
```

The HTTP server also retains the framework's sample routes (`/hello`, `/echo`, `/any`, and `/ctrl`).

The current handlers do not implement authentication or HTTPS. Change the default Wi-Fi passwords and keep device management on a trusted network.

## Recovery and firmware updates

Recovery is a separate application in the `factory` partition. **Firmware uploads go to the recovery server, not the main application.**

On ESP32-S3 (or another native Wi-Fi target):

1. Flash both applications with the top-level `idf.py flash` flow.
2. Open `http://<device-ip>/rebootfactory` to reboot into recovery.
3. Connect to the configured recovery network. By default, this is the `VE-Recovery-XXXX` AP; the recovery IP may differ from the main application's IP.
4. Open the recovery page and upload `build/vigilant-engine.bin` built for the same chip.
5. Recovery writes the image to `ota_0`, selects it as the boot partition, and restarts.

The recovery server provides `GET /`, `POST /update` (raw firmware binary), and `POST /boot` (boot the existing `ota_0` application). Do not upload a merged flash image, bootloader, or recovery binary as the main firmware.

The layout in [`partitions.csv`](partitions.csv) is:

| Partition | Offset | Size | Purpose |
| --- | --- | --- | --- |
| `nvs` | `0x9000` | `0x4000` | Nonvolatile settings |
| `otadata` | `0xD000` | `0x2000` | Boot selection |
| `phy_init` | `0xF000` | `0x1000` | PHY initialization data |
| `factory` | `0x10000` | `0x130000` | Recovery application on Wi-Fi targets |
| `ota_0` | `0x140000` | `0x2C0000` | Main application |

There is one main application slot and no `ota_1` rollback slot. ESP32-P4 currently uses serial flashing; Wi-Fi recovery is not built for it.

## Development

### Repository layout

| Path | Contents |
| --- | --- |
| [`main/`](main/) | Application entry point and project configuration menus |
| [`components/vigilant_engine/`](components/vigilant_engine/) | Initialization, networking, HTTP, WebSocket logging, and I²C APIs |
| [`components/status_led/`](components/status_led/) | WS2812B status LED implementation |
| `components/wuerth_sensors_sdk_esp32/` | Würth sensor SDK Git submodule |
| [`vigilant-engine-frontend/`](vigilant-engine-frontend/) | Vue/Vite dashboard and recovery UI |
| [`vigilant-engine-recovery/`](vigilant-engine-recovery/) | Separate recovery firmware |
| [`docs/`](docs/) | Framework documentation and configuration guides |
| [`.github/workflows/`](.github/workflows/) | Build and style checks |

Start module-specific application work in `main/main.c`. The public framework API is declared in [`vigilant.h`](components/vigilant_engine/include/vigilant.h); the [I²C guide](docs/i2c-interface.md) describes device registration and register access.

### Frontend

```sh
cd vigilant-engine-frontend
npm ci
npm run dev:vigilant
```

Use `npm run dev:recovery` for the recovery UI. To rebuild the embedded assets:

```sh
npm run build:vigilant
npm run build:recovery
cd ..
idf.py build
```

The generated files are `build/static/vigilant/vigilant.html` and `build/static/recovery/index.html`. Rebuild the assets explicitly after frontend changes: the firmware configuration can reuse existing HTML.

To supply prebuilt assets, as CI does, run from the repository root after generating both files:

```sh
idf.py -DVE_BUILD_WEB=OFF \
  -DVE_VIGILANT_HTML="$PWD/build/static/vigilant/vigilant.html" \
  -DVE_RECOVERY_HTML="$PWD/build/static/recovery/index.html" \
  build
```

`VE_BUILD_WEB=OFF` disables automatic frontend generation; it does not remove the embedded UI. `CONFIG_VE_DISABLE_FRONTEND` replaces only the main dashboard HTML with a placeholder and leaves the API routes available.

### Development checks and documentation

```sh
uv sync
uv run pre-commit install
uv run pre-commit run --all-files
uv run mkdocs serve
```

Use `uv run mkdocs build` to generate the documentation site. The repository includes inherited ESP-IDF pytest examples; those require their ESP-IDF test dependencies and suitable hardware rather than just `uv sync`.

The [local documentation](docs/index.md) and [upstream Vigilant Engine documentation](https://www.bears-space.de/vigilant-engine/) provide framework background. Some inherited pages describe upstream workflows; use this README and the current CMake/Kconfig files for APOGEE Core setup and target limitations.

## License

This repository uses the **PolyForm Noncommercial License 1.0.0**. See [LICENSE.md](LICENSE.md) for the full terms. Dependencies and submodules retain their respective licenses.

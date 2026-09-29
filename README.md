# Greenhouse BLE Bridge — Clean Static-Analysis Package

This folder contains the final source files for the working prototype path:

```text
DHT11 -> Arduino Mega -> USB serial -> Mac app -> BLE -> ESP32 display
```

The files have been collected into one clean project without virtual environments,
PlatformIO build output, Python caches, screenshots, or older source revisions.

## Final source files

| Component | File | Purpose |
|---|---|---|
| ESP32 | `firmware/src/main.cpp` | BLE server, packet validation, LCD dashboard, and sensor forwarding |
| ESP32 | `firmware/platformio.ini` | PlatformIO board and Arduino GFX dependency configuration |
| Mac app | `mac_app/app.py` | Tkinter dashboard and Mega-to-ESP32 BLE bridge |
| Mac app | `mac_app/requirements.txt` | Python runtime dependencies |
| Arduino Mega | `mega/mega_dht_sensor_hub/mega_dht_sensor_hub.ino` | DHT11 acquisition and simulated auxiliary sensor values |
| SonarQube | `sonar-project.properties` | Static-analysis scope and generated-file exclusions |

## SonarQube scan

Run the scanner from this directory after setting the server URL and token in
your environment or scanner configuration:

```bash
sonar-scanner
```

Change `sonar.projectKey` if your SonarQube server assigns a different key.
The configuration analyzes the Python, C++, and Arduino source and excludes
`.venv`, `.pio`, cache files, and scanner output.

Full C/C++ and Arduino analysis requires a SonarQube installation with the
CFamily analyzer. The `.ino` suffix is assigned to the C++ analyzer in the
included properties file. If that analyzer is unavailable, the Mac Python app
can still be analyzed independently by setting `sonar.sources=mac_app`.

## Runtime setup

### Arduino Mega

Open `mega/mega_dht_sensor_hub/mega_dht_sensor_hub.ino` in Arduino IDE. Install
the Adafruit DHT sensor library, select the Arduino Mega, and upload at 115200
baud. The DHT11 signal pin is configured as digital pin 7.

### ESP32

Open `firmware` as the PlatformIO project and run:

```bash
platformio run --target upload
```

The target is `esp32dev`, and the Arduino GFX library is pinned to version
1.6.1 to match the working display build.

### Mac app

From `mac_app`, create and activate a virtual environment, then run:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python app.py
```

Select the Mega serial port in the dashboard and start the bridge. The Mac app
and ESP32 use the same `Greenhouse-ESP32` name, BLE service UUID, write UUID,
and twelve-field `SEQ`/`END` packet protocol.

#include <Arduino.h>
#include <Arduino_GFX_Library.h>
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

#include <cmath>
#include <cstdlib>
#include <string>

constexpr char DEVICE_NAME[] = "Greenhouse-ESP32";
constexpr char GREENHOUSE_ID[] = "GH-001";
constexpr char SENSOR_SERVICE_UUID[] =
    "7d9e0001-dc6d-4f04-9fb8-11f8d5c7a001";
constexpr char SENSOR_WRITE_UUID[] =
    "7d9e0002-dc6d-4f04-9fb8-11f8d5c7a001";
constexpr char SENSOR_NOTIFY_UUID[] =
    "7d9e0003-dc6d-4f04-9fb8-11f8d5c7a001";
constexpr char DEVICE_ID_UUID[] =
    "7d9e0004-dc6d-4f04-9fb8-11f8d5c7a001";

constexpr int TFT_BACKLIGHT_PIN = 27;
constexpr uint16_t COMPLETE_PACKET_MASK = 0x0FFF;
constexpr unsigned long NOTIFICATION_CHUNK_DELAY_MS = 15;

Arduino_DataBus *bus = new Arduino_HWSPI(
    2,   // DC
    15,  // CS
    14,  // SCK
    13,  // MOSI
    12   // MISO
);

Arduino_GFX *gfx = new Arduino_ST7796(
    bus,
    -1,    // Display reset is shared with ESP32 reset.
    1,     // Landscape rotation.
    false, // This panel is not IPS.
    320,
    480
);

struct SensorState {
  float temperature1F;
  float temperature2F;
  float humidity1Percent;
  float humidity2Percent;
  int occupancy;
  float windSpeedMS;
  int fanRpm;
  float waterLevelPercent;
  float batteryVoltageV;
  float panelVoltageV;
  float solarPowerW;
  float uvIndex;
};

BLEServer *bleServer = nullptr;
BLECharacteristic *sensorNotifyCharacteristic = nullptr;
BLE2902 *sensorNotifyDescriptor = nullptr;

volatile bool connectionChanged = false;
volatile bool advertisingRestartRequested = false;
volatile bool readingReady = false;
uint32_t connectedClientCount = 0;
bool hasReading = false;
bool packetOpen = false;

SensorState sensors{};
SensorState pendingSensors{};
SensorState receivedSensors{};
uint16_t pendingPacketMask = 0;
unsigned long pendingSequence = 0;
unsigned long receivedSequence = 0;
unsigned long readingNumber = 0;
portMUX_TYPE sensorMux = portMUX_INITIALIZER_UNLOCKED;

String displayFloat(float value, int decimals, const char *unit) {
  if (!hasReading) {
    return String("-- ") + unit;
  }
  return String(value, decimals) + " " + unit;
}

String displayInt(int value, const char *unit) {
  if (!hasReading) {
    return String("-- ") + unit;
  }
  return String(value) + " " + unit;
}

void drawStatus(const String &message, uint16_t color) {
  gfx->fillRect(12, 284, 456, 24, RGB565_BLACK);
  gfx->setTextColor(color);
  gfx->setTextSize(1);
  gfx->setCursor(18, 292);
  gfx->print(message);
}

void drawMetric(
    int x,
    int y,
    int width,
    const String &label,
    const String &value,
    uint16_t accentColor
) {
  gfx->drawRoundRect(x, y, width, 34, 5, RGB565_DARKGREY);
  gfx->fillRect(x + 1, y + 6, 3, 22, accentColor);

  gfx->setTextColor(accentColor);
  gfx->setTextSize(1);
  gfx->setCursor(x + 10, y + 5);
  gfx->print(label);

  gfx->setTextColor(RGB565_WHITE);
  gfx->setTextSize(2);
  gfx->setCursor(x + 10, y + 17);
  gfx->print(value);
}

void drawDashboard() {
  gfx->fillScreen(RGB565_BLACK);
  gfx->drawRoundRect(7, 7, 466, 306, 9, RGB565_DARKGREY);

  gfx->setTextColor(RGB565_CYAN);
  gfx->setTextSize(2);
  gfx->setCursor(18, 19);
  gfx->print("GREENHOUSE SENSOR MONITOR");

  gfx->setTextColor(RGB565_LIGHTGREY);
  gfx->setTextSize(1);
  gfx->setCursor(326, 24);
  gfx->print("MAC > ESP > IPHONE");

  const int leftX = 16;
  const int rightX = 245;
  const int cardWidth = 219;
  const int startY = 48;
  const int rowGap = 38;

  drawMetric(
      leftX,
      startY,
      cardWidth,
      "TEMP DHT11 (REAL)",
      displayFloat(sensors.temperature1F, 1, "F"),
      RGB565_ORANGE
  );
  drawMetric(
      rightX,
      startY,
      cardWidth,
      "TEMP ZONE 2",
      displayFloat(sensors.temperature2F, 1, "F"),
      RGB565_ORANGE
  );
  drawMetric(
      leftX,
      startY + rowGap,
      cardWidth,
      "HUMIDITY DHT11 (REAL)",
      displayFloat(sensors.humidity1Percent, 1, "%"),
      RGB565_BLUE
  );
  drawMetric(
      rightX,
      startY + rowGap,
      cardWidth,
      "HUMIDITY ZONE 2",
      displayFloat(sensors.humidity2Percent, 1, "%"),
      RGB565_BLUE
  );
  drawMetric(
      leftX,
      startY + rowGap * 2,
      cardWidth,
      "OCCUPANCY",
      displayInt(sensors.occupancy, "people"),
      RGB565_MAGENTA
  );
  drawMetric(
      rightX,
      startY + rowGap * 2,
      cardWidth,
      "WIND SPEED",
      displayFloat(sensors.windSpeedMS, 1, "m/s"),
      RGB565_CYAN
  );
  drawMetric(
      leftX,
      startY + rowGap * 3,
      cardWidth,
      "FAN ROTATION",
      displayInt(sensors.fanRpm, "RPM"),
      RGB565_GREEN
  );
  drawMetric(
      rightX,
      startY + rowGap * 3,
      cardWidth,
      "WATER LEVEL",
      displayFloat(sensors.waterLevelPercent, 1, "%"),
      RGB565_BLUE
  );
  drawMetric(
      leftX,
      startY + rowGap * 4,
      cardWidth,
      "BATTERY VOLTAGE",
      displayFloat(sensors.batteryVoltageV, 1, "V"),
      RGB565_YELLOW
  );
  drawMetric(
      rightX,
      startY + rowGap * 4,
      cardWidth,
      "PANEL VOLTAGE",
      displayFloat(sensors.panelVoltageV, 1, "V"),
      RGB565_YELLOW
  );
  drawMetric(
      leftX,
      startY + rowGap * 5,
      cardWidth,
      "SOLAR GENERATION",
      displayFloat(sensors.solarPowerW, 0, "W"),
      RGB565_ORANGE
  );
  drawMetric(
      rightX,
      startY + rowGap * 5,
      cardWidth,
      "UV INDEX",
      displayFloat(sensors.uvIndex, 1, ""),
      RGB565_MAGENTA
  );

  if (connectedClientCount >= 2 && hasReading) {
    drawStatus(
        "Mac + iPhone connected | packet #" + String(readingNumber),
        RGB565_GREEN
    );
  } else if (connectedClientCount == 1 && hasReading) {
    drawStatus(
        "1 BLE client connected | packet #" + String(readingNumber),
        RGB565_CYAN
    );
  } else if (connectedClientCount > 0) {
    drawStatus("BLE client connected | waiting for data", RGB565_CYAN);
  } else {
    drawStatus("Waiting for Mac or iPhone Bluetooth...", RGB565_YELLOW);
  }
}

class ServerCallbacks final : public BLEServerCallbacks {
  void onConnect(BLEServer *server) override {
    connectionChanged = true;
    advertisingRestartRequested = true;
    Serial.printf(
        "BLE client connected. Active clients: %lu\n",
        static_cast<unsigned long>(server->getConnectedCount())
    );
  }

  void onDisconnect(BLEServer *server) override {
    connectionChanged = true;
    advertisingRestartRequested = true;
    Serial.printf(
        "BLE client disconnected. Active clients: %lu\n",
        static_cast<unsigned long>(server->getConnectedCount())
    );
  }
};

bool parseFloatValue(const String &rawValue, float &parsedValue) {
  char *endPointer = nullptr;
  parsedValue = strtof(rawValue.c_str(), &endPointer);
  return endPointer != rawValue.c_str() &&
         *endPointer == '\0' &&
         std::isfinite(parsedValue);
}

bool parseLongValue(const String &rawValue, long &parsedValue) {
  char *endPointer = nullptr;
  parsedValue = strtol(rawValue.c_str(), &endPointer, 10);
  return endPointer != rawValue.c_str() && *endPointer == '\0';
}

class SensorWriteCallbacks final : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *characteristic) override {
    const std::string rawValue = characteristic->getValue();
    String message(rawValue.c_str());
    message.trim();

    const int separator = message.indexOf(',');
    if (separator <= 0) {
      return;
    }

    String key = message.substring(0, separator);
    String value = message.substring(separator + 1);
    key.trim();
    value.trim();
    key.toUpperCase();

    if (key == "SEQ") {
      long sequence = 0;
      if (!parseLongValue(value, sequence) || sequence < 0) {
        return;
      }
      pendingSequence = static_cast<unsigned long>(sequence);
      pendingPacketMask = 0;
      packetOpen = true;
      return;
    }

    if (key == "END") {
      long endingSequence = 0;
      if (!parseLongValue(value, endingSequence)) {
        return;
      }
      if (
          packetOpen &&
          static_cast<unsigned long>(endingSequence) == pendingSequence &&
          pendingPacketMask == COMPLETE_PACKET_MASK
      ) {
        portENTER_CRITICAL(&sensorMux);
        receivedSensors = pendingSensors;
        receivedSequence = pendingSequence;
        readingReady = true;
        portEXIT_CRITICAL(&sensorMux);
      }
      packetOpen = false;
      return;
    }

    float floatValue = 0.0F;
    long integerValue = 0;

    if (key == "T1" && parseFloatValue(value, floatValue)) {
      pendingSensors.temperature1F = floatValue;
      pendingPacketMask |= 1U << 0;
    } else if (key == "T2" && parseFloatValue(value, floatValue)) {
      pendingSensors.temperature2F = floatValue;
      pendingPacketMask |= 1U << 1;
    } else if (key == "H1" && parseFloatValue(value, floatValue)) {
      pendingSensors.humidity1Percent = floatValue;
      pendingPacketMask |= 1U << 2;
    } else if (key == "H2" && parseFloatValue(value, floatValue)) {
      pendingSensors.humidity2Percent = floatValue;
      pendingPacketMask |= 1U << 3;
    } else if (key == "OCC" && parseLongValue(value, integerValue)) {
      pendingSensors.occupancy = static_cast<int>(integerValue);
      pendingPacketMask |= 1U << 4;
    } else if (key == "WIND" && parseFloatValue(value, floatValue)) {
      pendingSensors.windSpeedMS = floatValue;
      pendingPacketMask |= 1U << 5;
    } else if (key == "RPM" && parseLongValue(value, integerValue)) {
      pendingSensors.fanRpm = static_cast<int>(integerValue);
      pendingPacketMask |= 1U << 6;
    } else if (key == "WATER" && parseFloatValue(value, floatValue)) {
      pendingSensors.waterLevelPercent = floatValue;
      pendingPacketMask |= 1U << 7;
    } else if (key == "BAT" && parseFloatValue(value, floatValue)) {
      pendingSensors.batteryVoltageV = floatValue;
      pendingPacketMask |= 1U << 8;
    } else if (key == "PANEL" && parseFloatValue(value, floatValue)) {
      pendingSensors.panelVoltageV = floatValue;
      pendingPacketMask |= 1U << 9;
    } else if (key == "SOLAR" && parseFloatValue(value, floatValue)) {
      pendingSensors.solarPowerW = floatValue;
      pendingPacketMask |= 1U << 10;
    } else if (key == "UV" && parseFloatValue(value, floatValue)) {
      pendingSensors.uvIndex = floatValue;
      pendingPacketMask |= 1U << 11;
    }
  }
};

void sendNotificationChunk(const String &message) {
  if (
      sensorNotifyCharacteristic == nullptr ||
      sensorNotifyDescriptor == nullptr ||
      !sensorNotifyDescriptor->getNotifications() ||
      bleServer == nullptr ||
      bleServer->getConnectedCount() == 0
  ) {
    return;
  }

  if (message.length() > 20) {
    Serial.printf(
        "Skipped oversized BLE notification (%u bytes): %s\n",
        static_cast<unsigned int>(message.length()),
        message.c_str()
    );
    return;
  }

  sensorNotifyCharacteristic->setValue(message.c_str());
  sensorNotifyCharacteristic->notify();
  delay(NOTIFICATION_CHUNK_DELAY_MS);
}

void notifySensorReading(
    const SensorState &reading,
    unsigned long sequence
) {
  if (
      sensorNotifyDescriptor == nullptr ||
      !sensorNotifyDescriptor->getNotifications()
  ) {
    return;
  }

  sendNotificationChunk("SEQ," + String(sequence));
  sendNotificationChunk("T1," + String(reading.temperature1F, 1));
  sendNotificationChunk("T2," + String(reading.temperature2F, 1));
  sendNotificationChunk("H1," + String(reading.humidity1Percent, 1));
  sendNotificationChunk("H2," + String(reading.humidity2Percent, 1));
  sendNotificationChunk("OCC," + String(reading.occupancy));
  sendNotificationChunk("WIND," + String(reading.windSpeedMS, 1));
  sendNotificationChunk("RPM," + String(reading.fanRpm));
  sendNotificationChunk("WATER," + String(reading.waterLevelPercent, 1));
  sendNotificationChunk("BAT," + String(reading.batteryVoltageV, 1));
  sendNotificationChunk("PANEL," + String(reading.panelVoltageV, 1));
  sendNotificationChunk("SOLAR," + String(reading.solarPowerW, 0));
  sendNotificationChunk("UV," + String(reading.uvIndex, 1));
  sendNotificationChunk("END," + String(sequence));

  Serial.printf(
      "Notified subscribed iPhone client of packet %lu.\n",
      sequence
  );
}

void setupBluetooth() {
  BLEDevice::init(DEVICE_NAME);

  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new ServerCallbacks());

  BLEService *sensorService =
      bleServer->createService(SENSOR_SERVICE_UUID);

  BLECharacteristic *sensorWriteCharacteristic =
      sensorService->createCharacteristic(
          SENSOR_WRITE_UUID,
          BLECharacteristic::PROPERTY_WRITE |
              BLECharacteristic::PROPERTY_WRITE_NR
      );
  sensorWriteCharacteristic->setCallbacks(new SensorWriteCallbacks());

  sensorNotifyCharacteristic = sensorService->createCharacteristic(
      SENSOR_NOTIFY_UUID,
      BLECharacteristic::PROPERTY_NOTIFY
  );
  sensorNotifyDescriptor = new BLE2902();
  sensorNotifyCharacteristic->addDescriptor(sensorNotifyDescriptor);

  BLECharacteristic *deviceIdCharacteristic =
      sensorService->createCharacteristic(
          DEVICE_ID_UUID,
          BLECharacteristic::PROPERTY_READ
      );
  deviceIdCharacteristic->setValue(GREENHOUSE_ID);

  sensorService->start();

  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(SENSOR_SERVICE_UUID);
  advertising->setScanResponse(true);
  BLEDevice::startAdvertising();

  Serial.printf("BLE device '%s' is advertising.\n", DEVICE_NAME);
  Serial.printf("Mac write characteristic: %s\n", SENSOR_WRITE_UUID);
  Serial.printf("iPhone notify characteristic: %s\n", SENSOR_NOTIFY_UUID);
  Serial.printf(
      "QR pairing identity: %s on characteristic %s\n",
      GREENHOUSE_ID,
      DEVICE_ID_UUID
  );
}

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(TFT_BACKLIGHT_PIN, OUTPUT);
  digitalWrite(TFT_BACKLIGHT_PIN, HIGH);

  gfx->begin();
  drawDashboard();
  setupBluetooth();
}

void loop() {
  SensorState nextReading{};
  unsigned long nextSequence = 0;
  bool hasNextReading = false;

  portENTER_CRITICAL(&sensorMux);
  if (readingReady) {
    nextReading = receivedSensors;
    nextSequence = receivedSequence;
    readingReady = false;
    hasNextReading = true;
  }
  portEXIT_CRITICAL(&sensorMux);

  if (hasNextReading) {
    sensors = nextReading;
    hasReading = true;
    readingNumber++;
    drawDashboard();
    notifySensorReading(sensors, nextSequence);

    Serial.printf(
        "Packet %lu (sequence %lu) | DHT11 %.1f F, %.1f %% | Solar %.0f W\n",
        readingNumber,
        nextSequence,
        sensors.temperature1F,
        sensors.humidity1Percent,
        sensors.solarPowerW
    );
  }

  if (connectionChanged) {
    connectionChanged = false;
    connectedClientCount =
        bleServer == nullptr ? 0 : bleServer->getConnectedCount();
    drawDashboard();
  }

  if (advertisingRestartRequested) {
    advertisingRestartRequested = false;
    delay(75);
    if (bleServer != nullptr) {
      bleServer->startAdvertising();
      Serial.println("BLE advertising restarted for another client.");
    }
  }

  delay(10);
}

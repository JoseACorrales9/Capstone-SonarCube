#include <DHT.h>

constexpr uint8_t DHT_PIN = 7;
constexpr uint8_t DHT_TYPE = DHT11;
constexpr unsigned long UPDATE_INTERVAL_MS = 2500;

DHT dht(DHT_PIN, DHT_TYPE);

unsigned long lastUpdate = 0;
unsigned long readingNumber = 0;

float triangleWave(unsigned long step, unsigned long period) {
  const unsigned long halfPeriod = period / 2;
  const unsigned long position = step % period;

  if (position <= halfPeriod) {
    return static_cast<float>(position) /
           static_cast<float>(halfPeriod);
  }

  return static_cast<float>(period - position) /
         static_cast<float>(halfPeriod);
}

void setup() {
  Serial.begin(115200);
  dht.begin();
  delay(2000);

  Serial.println("Mega DHT11 and simulated sensor hub ready");
}

void loop() {
  const unsigned long now = millis();
  if (now - lastUpdate < UPDATE_INTERVAL_MS) {
    return;
  }
  lastUpdate = now;

  // These are the two physical readings in the current prototype.
  const float humidity1Percent = dht.readHumidity();
  const float temperature1F = dht.readTemperature(true);

  if (isnan(humidity1Percent) || isnan(temperature1F)) {
    Serial.println("ERROR,DHT_READ_FAILED");
    return;
  }

  // The remaining values stay simulated until those sensors are available.
  const float environmentCycle = triangleWave(readingNumber, 40);
  const float daylightCycle = triangleWave(readingNumber, 60);
  const float tankCycle = triangleWave(readingNumber, 120);

  const float temperature2F = temperature1F + 1.0F + environmentCycle;
  const float humidity2Percent = constrain(
      humidity1Percent + 2.0F + environmentCycle,
      0.0F,
      100.0F
  );
  const int occupancy = (readingNumber / 8) % 4;
  const float windSpeedMS = 0.8F + environmentCycle * 3.2F;
  const int fanRpm = 650 + static_cast<int>(environmentCycle * 950.0F);
  const float waterLevelPercent = 84.0F - tankCycle * 22.0F;
  const float batteryVoltageV = 12.2F + daylightCycle * 0.8F;
  const float panelVoltageV = daylightCycle * 20.5F;
  const float solarPowerW = daylightCycle * 360.0F;
  const float uvIndex = daylightCycle * 8.0F;

  // DATA,T1,T2,H1,H2,OCC,WIND,RPM,WATER,BAT,PANEL,SOLAR,UV
  Serial.print("DATA,");
  Serial.print(temperature1F, 1);
  Serial.print(',');
  Serial.print(temperature2F, 1);
  Serial.print(',');
  Serial.print(humidity1Percent, 1);
  Serial.print(',');
  Serial.print(humidity2Percent, 1);
  Serial.print(',');
  Serial.print(occupancy);
  Serial.print(',');
  Serial.print(windSpeedMS, 1);
  Serial.print(',');
  Serial.print(fanRpm);
  Serial.print(',');
  Serial.print(waterLevelPercent, 1);
  Serial.print(',');
  Serial.print(batteryVoltageV, 1);
  Serial.print(',');
  Serial.print(panelVoltageV, 1);
  Serial.print(',');
  Serial.print(solarPowerW, 1);
  Serial.print(',');
  Serial.println(uvIndex, 1);

  readingNumber++;
}

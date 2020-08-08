
// AirClock - Wifi Clock and Air Quality Monitor
// Based on Arduairs
// Github: https://github.com/greenpig/airclock

// Configurarions

// Need to enable this to enable air quality monitoring
#define ENABLE_BME680

// End configuration


#include "bsec.h"
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <WifiUdp.h>
#include <NTPClient.h>
#include <EEPROM.h>
#include <PubSubClient.h>
#include <EasyButton.h>
#include <time.h>
#include <WebServer.h>
#include <AutoConnect.h>
//#include <BigNumbers_I2C.h>

#include "util.h"
#include "HugeNumbers_I2C.h"

#define VERSION "v0.6"


// Controls which pins are the I2C ones.
#define PIN_I2C_SDA 21
#define PIN_I2C_SCL 22

// Controls how often the code persists the BSEC Calibration data
#define STATE_SAVE_PERIOD  UINT32_C(60 * 60 * 1000) // every 60 minutes

const char* mqtt_server = "192.168.1.200";
char ssid[33] = "";
char passwd[33] = "";

// Controls the offset for the temperature sensor. My BME680 was reading 4 degrees higher than another thermometer I trusted more, so my offset is 4.0.
const float tempOffset = 4.0;

// Configuration data for the BSEC library - telling it it is running on a 3.3v supply, that it is read from every 3 seconds, and that the sensor should take into account the last 4 days worth of data for calibration purposes.
const uint8_t bsec_config_iaq[] = {
#include "config/generic_33v_3s_4d/bsec_iaq.txt"
};

// Create an object of the class Bsec
Bsec iaqSensor;

String output;

// This stores a question mark and is eventually replaced with a space character once the BSEC library has decided it has enough calibration data to persist. It is displayed in the bottom right of the LCD as a kind of debug symbol.
String sensorPersisted = "?";

uint8_t bsecState[BSEC_MAX_STATE_BLOB_SIZE] = {0};
uint16_t stateUpdateCounter = 0;

WiFiClient espClient;
//PubSubClient broker(espClient);
WebServer server;

bool wifiConfigured = false;
bool wifiSwitchRequested = false;
unsigned long wifiStatusMillis = 0;

// The I2C address of your LCD, it will likely either be 0x27 or 0x3F
const uint8_t LCD_ADDR = 0x27;

// 20 characters across, 4 lines deep
LiquidCrystal_I2C lcd(LCD_ADDR, 20, 4);
//BigNumbers_I2C bigNum(&lcd);
HugeNumbers_I2C hugeNum(&lcd);

// Current active screen
int screen = 0;       // Current screen
int active = 1;       // Are we "active" (backlight on)
int activeMillis = millis(); // Millis when we become active
// Stay active for this long (millis)
#define ACTIVE_DURATION 10000

// Button setup
#define BUTTON_PIN 12

// NTP and time keeping
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "ntp.aliyun.com");
int timezone = 8;   // Beijing time
unsigned long ntpEpoch = 1577808000;    // 2020-1-1 0:0:0 Beijing time
unsigned long ntpMillis;    // millis() value at NTP sync time
unsigned long ntpAttemptMillis;    // 上一次尝试同步时间的millis，不管成功没成功（可能Wifi不通什么的）

EasyButton button(BUTTON_PIN);

#define SCREEN_CLOCK 0
#define SCREEN_AIR 1
#define SCREEN_WIFI 2

void onButtonPressed() {
  Serial.println("Button pressed");
  if (!active) {
    active = 1;
    lcd.backlight();
  } else {
    screen = (screen + 1) % 3;
#ifndef ENABLE_BME680
    // Skip air quality screen
    if (screen == 1)
      screen = 2;
#endif
    lcd.clear();
    if (screen == 0) {
      hugeNum.begin();
    } else if (screen == 1) {
      // I wanted some iconography, so this function creates some icons in the LCDs memory. They were created using Maxpromers LCD Character Creator
      createLCDSymbols();
    }
    lcd.backlight();
  }
  activeMillis = millis();
}

void onButtonLongPressed() {
  Serial.println("Button long pressed");
  if (screen == SCREEN_WIFI) {
    wifiSwitchRequested = true;
    Serial.println("Wifi switch requested");
  }
}

void rootPage() {
  Serial.println("Serving home page");
  char content[] = "Hello, world!";
  server.send(200, "text/plain", content);
}

bool wifiOn(const char* ssid, const char* password, unsigned long timeout) {
  Serial.printf("Wifi connecting to %s\n", ssid);
  WiFi.mode(WIFI_STA);
  delay(100);
  WiFi.begin(ssid, password);
  unsigned long tm = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - tm > timeout)
      return false;
  }
  return true;
}

bool wifiOff() {
  Serial.println("Wifi off.");
  WiFi.disconnect();
  WiFi.mode(WIFI_OFF);
}

bool wifiAutoConnect() {
  Serial.println("Starting wifi auto connect...");
  AutoConnectConfig config;
  config.immediateStart = true;
  AutoConnect portal(server);
  portal.config(config);
  if (portal.begin()) {
    portal.end();
  }
}

void setup(void)
{
  setCpuFrequencyMhz(80);

  Serial.begin(115200);
  Serial.println("Airmonitor started.\n");
  //  broker.setServer(mqtt_server, 1883);

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

#ifdef ENABLE_BME680
  // Your module MAY use the primary address, which is available as BME680_I2C_ADDR_PRIMARY
  iaqSensor.begin(BME680_I2C_ADDR_SECONDARY, Wire);
  output = "\nBSEC library version " + String(iaqSensor.version.major) + "." + String(iaqSensor.version.minor) + "." + String(iaqSensor.version.major_bugfix) + "." + String(iaqSensor.version.minor_bugfix);
  Serial.println(output);
  checkIaqSensorStatus();
  iaqSensor.setConfig(bsec_config_iaq);

  //loadState here refers to the BSEC Calibration state.
  loadState();

  // List all the sensors we want the bsec library to give us data for
  bsec_virtual_sensor_t sensorList[10] = {
    BSEC_OUTPUT_RAW_TEMPERATURE,
    BSEC_OUTPUT_RAW_PRESSURE,
    BSEC_OUTPUT_RAW_HUMIDITY,
    BSEC_OUTPUT_RAW_GAS,
    BSEC_OUTPUT_IAQ,
    BSEC_OUTPUT_STATIC_IAQ,
    BSEC_OUTPUT_CO2_EQUIVALENT,
    BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
  };

  iaqSensor.setTemperatureOffset(tempOffset);

  // Receive data from sensor list above at BSEC_SAMPLE_RATE_LP rate (every 3 seconds). There is also BSEC_SAMPLE_RATE_ULP - which requires a configuration change above.
  iaqSensor.updateSubscription(sensorList, 10, BSEC_SAMPLE_RATE_LP);
  checkIaqSensorStatus();
#else
  Serial.println("Firmware has air quality sensor disabled.");
#endif

  lcd.begin();
  lcd.backlight();

//  bigNum.begin();
  hugeNum.begin();

  // Set up button
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  button.begin();

  button.onPressed(onButtonPressed);
  button.onPressedFor(2000, onButtonLongPressed);

  Serial.println("Starting Web Server");
  server.on("/", rootPage);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Connecting Wifi...");
  lcd.setCursor(0, 1);
  lcd.print("To setup, connect");
  lcd.setCursor(0, 2);
  lcd.print("your phone to:");
  lcd.setCursor(0, 3);
  lcd.print("  esp32ap/12345678");

  AutoConnectCredential credt;
  station_config_t  config;
  for (int8_t e = credt.entries() - 1; e >= 0; e--) { // looping from most recent to oldest
    credt.load(e, &config);
    if (wifiOn((char *)config.ssid, (char *)config.password, 30000)) {
      strncpy(ssid, (char *)config.ssid, sizeof(ssid) - 1);
      strncpy(passwd, (char *)config.password, sizeof(passwd) - 1);
      break;
    }
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Starting AutoConnect Server");
    wifiAutoConnect();
  }

  Serial.println("HTTP server:" + WiFi.localIP().toString());
  wifiConfigured = true;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Getting time from");
  lcd.setCursor(0, 1);
  lcd.print("server...");

  Serial.println("Syncing time...");
  timeClient.begin();
  while (!timeClient.update())
    timeClient.forceUpdate();
  ntpEpoch = timeClient.getEpochTime();
  ntpAttemptMillis = ntpMillis = millis();
  Serial.printf("NTP epoch = %ul\n", ntpEpoch);

  wifiOff();

  lcd.clear();

  activeMillis = millis();

  // Set up light sleep for saving power
  esp_sleep_enable_timer_wakeup(500000);
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_12, 0);    // Press button to wake. 0 means wake up on high to low

  /* zf: No network for now
    connectToNetwork();
  */
}

void clockScreen() {
  unsigned long nowMillis = millis();
  unsigned long nowEpoch = ntpEpoch + (nowMillis - ntpMillis) / 1000;

  struct tm dt;
  epochToDateTime(nowEpoch, timezone, &dt);

  // print time
//  bigNum.displayLargeInt(dt.tm_hour, 0, 0, 2, true);
//  bigNum.displayLargeInt(dt.tm_min, 7, 0, 2, true);
//  bigNum.displayLargeInt(dt.tm_sec, 14, 0, 2, true);
//  lcd.setCursor(6, 1);
//  lcd.print(":");
//  lcd.setCursor(13, 1);
//  lcd.print(":");
  hugeNum.displayHugeInt(dt.tm_hour, 1, 0, 2, true);
  hugeNum.displayHugeInt(dt.tm_min, 10, 0, 2, true);
//  hugeNum.displayHugeInt(dt.tm_sec, 10, 0, 2, true);
  lcd.setCursor(9, 1);
  lcd.print(":");

  // print date at bottom
  lcd.setCursor(5, 3);
  lcd.printf("%04d-%02d-%02d", dt.tm_year, dt.tm_mon + 1, dt.tm_mday);
}

void airQualityScreen() {
  // iaqSensor.run() will return true once new data becomes available
  if (iaqSensor.run()) {
    displayIAQ(String(iaqSensor.staticIaq));
    displayTemp(String(iaqSensor.temperature));
    displayHumidity(String(iaqSensor.humidity));
    displayPressure(String(iaqSensor.pressure / 100));
    displayCO2(String(iaqSensor.co2Equivalent));
    displayVOC(String(iaqSensor.breathVocEquivalent));
    displaySensorPersisted();
    updateState();
  } else {
    checkIaqSensorStatus();
  }
}

void wifiScreen() {
  if (wifiSwitchRequested) {
    Serial.println("Wifi switch requested.");
    wifiSwitchRequested = false;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("To setup, connect");
    lcd.setCursor(0, 1);
    lcd.print("your phone to:");
    lcd.setCursor(0, 2);
    lcd.print("  esp32ap/12345678");
    wifiAutoConnect();
    lcd.clear();
    Serial.println("Wifi switch finished.");
  } else if (!wifiConfigured) {
    lcd.setCursor(0, 0);
    lcd.print("Wifi not connected");
  } else {
    unsigned long now = millis();
    if (now - wifiStatusMillis > 1000) {
      wifiStatusMillis = now;
      lcd.setCursor(0, 0);
      lcd.print("Connected to:");
      lcd.setCursor(2, 1);
      lcd.print(ssid);
      lcd.setCursor(0, 2);
      lcd.print("Long press to reset");
      lcd.setCursor(0, 3);
      lcd.print(VERSION);
    }
    server.handleClient();
  }
}

// Function that is looped forever
void loop(void)
{
  // 0: clock, 1: air quality, 2: Wifi Setup
  /* zf: disable MQTT for now
    if (!broker.connected()) {
      reconnectToBroker();
    }
    broker.loop();
  */
  //  Serial.println(digitalRead(BUTTON_PIN));
  button.read();

  if (screen == 0 || screen == 1) {
    unsigned long now = millis();
    if (active && now - activeMillis > ACTIVE_DURATION) {
      active = 0;
      lcd.noBacklight();
    }
  }

  switch (screen) {
    case 0: {
        clockScreen();
        break;
      }
    case 1: {
        airQualityScreen();
        break;
      }
    case 2: {
        wifiScreen();
        break;
      }
  }

  // update NTP every 10 mins
  unsigned long now = millis();
  if (now - ntpAttemptMillis > 10 * 60 * 1000) {
    ntpAttemptMillis = now;
    wifiOn(ssid, passwd, 30000);
    if (timeClient.update()) {
      ntpEpoch = timeClient.getEpochTime();
      ntpMillis = millis();
      Serial.printf("NTP update: %ul\n", ntpEpoch);
    }
    wifiOff();
  }

//  if (screen != 2) {
//    delay(100);
//  }
    if (screen != 2) {
    // Go to sleep to save power, wake up 4 times a second
    int ret = esp_light_sleep_start();
//    esp_deep_sleep_start();
    //  Serial.printf("light_sleep: %d\n", ret);
    }
}

// checks to make sure the BME680 Sensor is working correctly.
void checkIaqSensorStatus(void)
{
  if (iaqSensor.status != BSEC_OK) {
    lcd.setCursor(0, 0);
    lcd.print("Sensor error");
    if (iaqSensor.status < BSEC_OK) {
      output = "BSEC error code : " + String(iaqSensor.status);
      Serial.println(output);
    } else {
      output = "BSEC warning code : " + String(iaqSensor.status);
      Serial.println(output);
    }
    delay(5000);
    lcd.clear();
    checkIaqSensorStatus();
  }

  if (iaqSensor.bme680Status != BME680_OK) {
    lcd.setCursor(0, 0);
    lcd.print("Sensor error");
    if (iaqSensor.bme680Status < BME680_OK) {
      output = "BME680 error code : " + String(iaqSensor.bme680Status);
      Serial.println(output);
    } else {
      output = "BME680 warning code : " + String(iaqSensor.bme680Status);
      Serial.println(output);
    }
    delay(5000);
    lcd.clear();
  }
}

// The below display functions display data as well as publishing it to the MQTT broker. They expect the area that they are rendering to to be free of characters.
void displayIAQ(String iaq)
{
  lcd.setCursor(0, 0);
  lcd.write(0);
  lcd.print(iaq);
  char carr[iaq.length()];
  iaq.toCharArray(carr, iaq.length());
  //  broker.publish("bme680/iaq", carr);
}

void displayTemp(String tmp)
{
  lcd.setCursor(14, 0);
  lcd.print(tmp);
  lcd.write(1);
  char carr[tmp.length()];
  tmp.toCharArray(carr, tmp.length());
  //  broker.publish("bme680/temperature", carr);
}

void displayHumidity(String humidity)
{
  lcd.setCursor(0, 1);
  lcd.write(2);
  lcd.print(humidity + "%");
  char carr[humidity.length()];
  humidity.toCharArray(carr, humidity.length());
  //  broker.publish("bme680/humidity", carr);
}

void displayPressure(String pressure)
{
  lcd.setCursor(12, 1);
  lcd.print(pressure);
  lcd.write(3);
  char carr[pressure.length()];
  pressure.toCharArray(carr, pressure.length());
  //  broker.publish("bme680/pressure", carr);
}

void displayCO2(String co)
{
  lcd.setCursor(0, 3);
  lcd.print("CO2 " + co + "ppm");
  char carr[co.length()];
  co.toCharArray(carr, co.length());
  //  broker.publish("bme680/co", carr);
}

void displayVOC(String voc)
{
  lcd.setCursor(0, 2);
  lcd.print("VOC " + voc + "ppm");
  char carr[voc.length()];
  voc.toCharArray(carr, voc.length());
  //  broker.publish("bme680/voc", carr);
}

void displaySensorPersisted()
{
  lcd.setCursor(19, 3);
  lcd.print(sensorPersisted);
}

/*
  void connectToNetwork() {
  lcd.setCursor(0, 0);
  lcd.print("Connecting to");
  lcd.setCursor(0, 1);
  lcd.print(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, passphrase);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Establishing connection to WiFi..");
  }
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Connected");
  Serial.println("Connected to network");
  delay(1000);
  lcd.clear();
  }
*/

/*
  void reconnectToBroker() {
  // Loop until we're reconnected
  while (!broker.connected()) {
    lcd.setCursor(0, 0);
    lcd.print("Connecting MQTT");
    lcd.setCursor(0, 1);
    lcd.print(mqtt_server);
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "aqm-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (broker.connect(clientId.c_str())) {
      Serial.println("connected to broker");
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Connected");
      delay(1000);
    } else {
      lcd.setCursor(0,2);
      lcd.print("Broker failed");
      Serial.print("failed connecting to broker, rc=");
      Serial.print(broker.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
    lcd.clear();
  }
  }
*/

void createLCDSymbols() {
  byte iaqsymbol[] = {
    0x04,
    0x0A,
    0x1F,
    0x11,
    0x0E,
    0x0A,
    0x0E,
    0x02
  };
  lcd.createChar(0, iaqsymbol);
  byte tempSymbol[] = {
    0x18,
    0x18,
    0x07,
    0x04,
    0x04,
    0x04,
    0x04,
    0x07
  };
  lcd.createChar(1, tempSymbol);
  byte humiditySymbol[] = {
    0x04,
    0x04,
    0x0A,
    0x0A,
    0x11,
    0x11,
    0x0A,
    0x04
  };
  lcd.createChar(2, humiditySymbol);
  byte airPressureSymbol[] = {
    0x04,
    0x15,
    0x0E,
    0x04,
    0x01,
    0x1E,
    0x01,
    0x1E
  };
  lcd.createChar(3, airPressureSymbol);
}

// loadState attempts to read the BSEC state from the EEPROM. If the state isn't there yet - it wipes that area of the EEPROM ready to be written to in the future. It'll also set the global variable 'sensorPersisted' to a space, so that the question mark disappears forever from the LCD.
void loadState(void)
{
  if (EEPROM.read(0) == BSEC_MAX_STATE_BLOB_SIZE) {
    // Existing state in EEPROM
    Serial.println("Reading state from EEPROM");

    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE; i++) {
      bsecState[i] = EEPROM.read(i + 1);
      Serial.println(bsecState[i], HEX);
    }
    sensorPersisted = " ";
    iaqSensor.setState(bsecState);
    checkIaqSensorStatus();
  } else {
    // Erase the EEPROM with zeroes
    Serial.println("Erasing EEPROM");

    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE + 1; i++)
      EEPROM.write(i, 0);

    EEPROM.commit();
  }
}

// updateState waits for the in air quality accuracy to hit '3' - and will then write the state to the EEPROM. Then on every STATE_SAVE_PERIOD, it'll update the state.
void updateState(void)
{
  bool update = false;
  /* Set a trigger to save the state. Here, the state is saved every STATE_SAVE_PERIOD with the first state being saved once the algorithm achieves full calibration, i.e. iaqAccuracy = 3 */
  if (stateUpdateCounter == 0) {
    if (iaqSensor.iaqAccuracy >= 3) {
      update = true;
      stateUpdateCounter++;
    }
  } else {
    /* Update every STATE_SAVE_PERIOD milliseconds */
    if ((stateUpdateCounter * STATE_SAVE_PERIOD) < millis()) {
      update = true;
      stateUpdateCounter++;
    }
  }

  if (update) {
    iaqSensor.getState(bsecState);
    checkIaqSensorStatus();

    Serial.println("Writing state to EEPROM");

    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE ; i++) {
      EEPROM.write(i + 1, bsecState[i]);
      Serial.println(bsecState[i], HEX);
    }

    EEPROM.write(0, BSEC_MAX_STATE_BLOB_SIZE);
    EEPROM.commit();
  }
}

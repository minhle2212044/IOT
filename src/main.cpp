  #define LED_PIN 48
  #define SDA_PIN GPIO_NUM_11
  #define SCL_PIN GPIO_NUM_12

  #include <WiFi.h>
  #include <Arduino_MQTT_Client.h>
  #include <ThingsBoard.h>
  #include "DHT20.h"
  #include "DHT.h"
  #include "Wire.h"
  #include <HTTPClient.h>
  #include <Update.h>

  constexpr char WIFI_SSID[] = "Galaxy A114957";
  constexpr char WIFI_PASSWORD[] = "852004lcm";

  constexpr char TOKEN[] = "ElCQRtwf63oV7B1gYxlJ";

  constexpr char THINGSBOARD_SERVER[] = "app.coreiot.io";
  constexpr uint16_t THINGSBOARD_PORT = 1883U;

  constexpr uint32_t MAX_MESSAGE_SIZE = 1024U;
  constexpr uint32_t SERIAL_DEBUG_BAUD = 115200U;

  constexpr char BLINKING_INTERVAL_ATTR[] = "blinkingInterval";
  constexpr char LED_MODE_ATTR[] = "ledMode";
  constexpr char LED_STATE_ATTR[] = "ledState";

  volatile bool attributesChanged = false;
  volatile int ledMode = 0;
  volatile bool ledState = false;

  constexpr uint16_t BLINKING_INTERVAL_MS_MIN = 10U;
  constexpr uint16_t BLINKING_INTERVAL_MS_MAX = 60000U;
  volatile uint16_t blinkingInterval = 1000U;
  uint32_t lastOtaCheck = 0;
  const uint32_t otaCheckInterval = 5000;
  uint32_t previousStateChange;

  constexpr int16_t telemetrySendInterval = 10000U;
  uint32_t previousDataSend;

  constexpr std::array<const char *, 2U> SHARED_ATTRIBUTES_LIST = {
    LED_STATE_ATTR,
    BLINKING_INTERVAL_ATTR
  };

  WiFiClient wifiClient;
  Arduino_MQTT_Client mqttClient(wifiClient);
  ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE);
  #define DHTPIN 6
  #define DHTTYPE DHT11
  DHT dht(DHTPIN, DHTTYPE);

  RPC_Response setValueButton(const RPC_Data &data) {
      Serial.println("Received Switch state");
      bool newState = data;
      Serial.print("Switch state change: ");
      Serial.println(newState);
      digitalWrite(LED_PIN, newState);
      attributesChanged = true;
      return RPC_Response("setValueButton", newState);
  }

  const std::array<RPC_Callback, 1U> callbacks = {
    RPC_Callback{ "setValueButton", setValueButton }
  };

  void processSharedAttributes(const Shared_Attribute_Data &data) {
    for (auto it = data.begin(); it != data.end(); ++it) {
      if (strcmp(it->key().c_str(), BLINKING_INTERVAL_ATTR) == 0) {
        const uint16_t new_interval = it->value().as<uint16_t>();
        if (new_interval >= BLINKING_INTERVAL_MS_MIN && new_interval <= BLINKING_INTERVAL_MS_MAX) {
          blinkingInterval = new_interval;
          Serial.print("Blinking interval is set to: ");
          Serial.println(new_interval);
        }
      } else if (strcmp(it->key().c_str(), LED_STATE_ATTR) == 0) {
        ledState = it->value().as<bool>();
        digitalWrite(LED_PIN, ledState);
        Serial.print("LED state is set to: ");
        Serial.println(ledState);
      }
    }
    attributesChanged = true;
  }

  const Shared_Attribute_Callback attributes_callback(&processSharedAttributes, SHARED_ATTRIBUTES_LIST.cbegin(), SHARED_ATTRIBUTES_LIST.cend());
  const Attribute_Request_Callback attribute_shared_request_callback(&processSharedAttributes, SHARED_ATTRIBUTES_LIST.cbegin(), SHARED_ATTRIBUTES_LIST.cend());

  void InitWiFi() {
    Serial.println("Connecting to AP ...");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("Connected to AP");
  }

  const bool reconnect() {
    const wl_status_t status = WiFi.status();
    if (status == WL_CONNECTED) {
      return true;
    }
    InitWiFi();
    return true;
  }

  TaskHandle_t readSensorTaskHandle = NULL;
  TaskHandle_t Task1Handle = NULL;

  void Task1(void *pvParameters) {
    for (;;) {
      if (!tb.connected()) {
        Serial.println("Connecting to ThingsBoard...");
        if (!tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT)) {
          Serial.println("Failed to connect to ThingsBoard");
          vTaskDelay(pdMS_TO_TICKS(2000));
          continue;
        }

        tb.sendAttributeData("macAddress", WiFi.macAddress().c_str());
        //tb.RPC_Subscribe(callbacks.cbegin(), callbacks.cend());
        tb.Shared_Attributes_Subscribe(attributes_callback);
        tb.Shared_Attributes_Request(attribute_shared_request_callback);
        Serial.println("Connected to ThingsBoard!");
      }

      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }

  void readSensorTask(void *pvParameters) {
    while (true) {
      float temperature = dht.readTemperature();
      float humidity = dht.readHumidity();
      
      if (isnan(temperature) || isnan(humidity)) {
        Serial.println("Failed to read from DHT11 sensor!");
      } else {
        Serial.print("Temperature: ");
        Serial.print(temperature);
        Serial.print(" °C, Humidity: ");
        Serial.print(humidity);
        Serial.println(" %");
        tb.sendTelemetryData("temperature", temperature);
        tb.sendTelemetryData("humidity", humidity);
      }
      tb.sendAttributeData("rssi", WiFi.RSSI());
      tb.sendAttributeData("channel", WiFi.channel());
      tb.sendAttributeData("bssid", WiFi.BSSIDstr().c_str());
      tb.sendAttributeData("localIp", WiFi.localIP().toString().c_str());
      tb.sendAttributeData("ssid", WiFi.SSID().c_str());
      vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
  }

  void checkFirmwareUpdate() {
    const char *firmwareUrl = "http://192.168.234.14:8080/firmware.bin";
  
    Serial.println("Checking for firmware update...");
  
    WiFiClient client;
    HTTPClient http;
    http.begin(client, firmwareUrl);
    int httpCode = http.GET();
  
    if (httpCode == 200) {
      int contentLength = http.getSize();
      bool canBegin = Update.begin(contentLength);
  
      if (canBegin) {
        Serial.println("Begin OTA update...");
        WiFiClient *updateClient = http.getStreamPtr();
        size_t written = Update.writeStream(*updateClient);
  
        if (written == contentLength) {
          Serial.println("OTA written successfully. Rebooting...");
          if (Update.end()) {
            if (Update.isFinished()) {
              Serial.println("Update successfully completed. Rebooting...");
              ESP.restart();
            } else {
              Serial.println("Update not finished.");
            }
          } else {
            Serial.printf("Error Occurred. Error #: %d\n", Update.getError());
          }
        } else {
          Serial.println("Written only partial data. OTA failed.");
        }
      } else {
        Serial.println("Not enough space to begin OTA.");
      }
    } else {
      Serial.printf("Firmware check failed. HTTP code: %d\n", httpCode);
    }
  
    http.end();
  }

  void setup() {
    Serial.begin(SERIAL_DEBUG_BAUD);
    pinMode(LED_PIN, OUTPUT);
    InitWiFi();

    Wire.begin(SDA_PIN, SCL_PIN);
    dht.begin();
    xTaskCreate(Task1, "Task1", 8192, NULL, 1, &Task1Handle);
    xTaskCreate(readSensorTask, "Read Sensor Task", 2048, NULL, 1, &readSensorTaskHandle);
  }

  void loop() {
    tb.loop();
    if (millis() - lastOtaCheck > otaCheckInterval) {
      checkFirmwareUpdate();
      lastOtaCheck = millis();
    }
  }

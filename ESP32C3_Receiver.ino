#include <SPI.h>
#include <mcp2515.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <WiFi.h>
#include <InfluxDbClient.h>
#include <InfluxDbCloud.h>

#define PWM_FREQ 5000
#define PWM_RESOLUTION 8
#define MOTOR_PUMP_PWM_CHANNEL 0
#define FAN_PWM_CHANNEL 1

// OLED config
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_SDA 4
#define OLED_SCL 5
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// CAN MCP2515 SPI config
#define SPI_SCK  6
#define SPI_MOSI 7
#define SPI_MISO 2
#define SPI_CS   10
SPIClass *spiAcc = nullptr;
#undef SPI
#define SPI (*spiAcc)
MCP2515 mcp2515(SPI_CS);
struct can_frame canMsg;

// L298N motor control pins
#define IN1  8
#define IN2  9
#define ENA  3

//L298N fan 
#define IN3 0
#define IN4 1
// #define ENB 18

// Sensor data struct
struct SensorData {
  uint16_t adcRaw;
  uint8_t moisturePercent;
  uint8_t pwm;

  uint8_t airHumidity;
  uint8_t temperature;
  String fanStatus;
};


// Queue để chia sẻ dữ liệu cho nhiều task
QueueHandle_t canQueue;

// Mutexes
SemaphoreHandle_t displayMutex;
SemaphoreHandle_t motorMutex;

// WiFi & InfluxDB config
#define WIFI_SSID "Hoangf"
#define WIFI_PASSWORD "27092003"
#define INFLUXDB_URL "https://us-east-1-1.aws.cloud2.influxdata.com"
#define INFLUXDB_TOKEN "KRBesZPLm3RvXfpLbbmZusBPDTnZH6LJmD6CuOkma7fPbXRtDhPaPoQOkWXFgo2m90wYBXFh_tR6mXSaP-yTKA=="
#define INFLUXDB_ORG "0fadfa6e2208f11a"
#define INFLUXDB_BUCKET "hoang"
#define TZ_INFO "UTC7"

InfluxDBClient client(INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET, INFLUXDB_TOKEN, InfluxDbCloud2CACert);
Point sensor("soil");

// ===== TASKS =====
void readCANTask(void *parameter) {
  for (;;) {
    if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
      SensorData sensorData;
      sensorData.adcRaw = (canMsg.data[1] << 8) | canMsg.data[0];
      sensorData.moisturePercent = canMsg.data[2];
      sensorData.airHumidity = canMsg.data[3];
      sensorData.temperature = canMsg.data[4];

      xQueueOverwrite(canQueue, &sensorData);  // Ghi đè queue để giữ dữ liệu mới nhất

      Serial.print("CAN Data: ADC=");
      Serial.print(sensorData.adcRaw);

      Serial.print(" Moisture=");
      Serial.println(sensorData.moisturePercent);

      Serial.print("%, Temp: "); 
      Serial.print(sensorData.temperature);

      Serial.print("C, Humidity: "); 
      Serial.println(sensorData.airHumidity);
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// void controlMotorTask(void *parameter) {
//   SensorData sensorData;
//   for (;;) {
//     if (xQueueReceive(canQueue, &sensorData, portMAX_DELAY)) {
//       int pwm = 0;
//       if (sensorData.moisturePercent < 20) pwm = 255;
//       else if (sensorData.moisturePercent < 40) pwm = 100;

//       sensorData.pwm = pwm;  //  Thêm giá trị pwm vào struct

//       xSemaphoreTake(motorMutex, portMAX_DELAY);
//       analogWrite(ENA, pwm);
//       xSemaphoreGive(motorMutex);

//       // Ghi lại vào queue để task khác thấy pwm đã cập nhật
//       xQueueOverwrite(canQueue, &sensorData);  //  Cập nhật lại queue

//       Serial.print("Motor PWM: ");
//       Serial.println(pwm);
//     }
//     vTaskDelay(500 / portTICK_PERIOD_MS);
//   }
// }

void controlMotorTask(void *parameter) {
  SensorData sensorData;
  for (;;) {
    if (xQueuePeek(canQueue, &sensorData, portMAX_DELAY)) {
      int pwm = 0;
      if (sensorData.moisturePercent < 20) pwm = 255;
      else if (sensorData.moisturePercent < 40) pwm = 100;

      sensorData.pwm = pwm;

      xSemaphoreTake(motorMutex, portMAX_DELAY);
      if (pwm > 0) {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        analogWrite(ENA, pwm);
      } else {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);  // Dừng motor
        analogWrite(ENA, 0);
      }
      xSemaphoreGive(motorMutex);

      Serial.print("Motor PWM: ");
      Serial.println(pwm);
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}


void controlFanTask(void *parameter) {
  SensorData sensorData;
  for (;;) {
    if (xQueuePeek(canQueue, &sensorData, portMAX_DELAY)) {
      int temp = sensorData.temperature;
      int humi = sensorData.airHumidity;

      String fanStatus = "OFF";
      xSemaphoreTake(motorMutex, portMAX_DELAY);

      if (temp > 30 && humi > 80) {
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        // digitalWrite(ENB, HIGH);
        // analogWrite(ENB, 255);
        // ledcWrite(ENB, 255);
        fanStatus = "REVERSE";
      } else if (temp > 30 && humi < 50) {
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        // digitalWrite(ENB, HIGH);
        // analogWrite(ENB, 255);
        // ledcWrite(ENB, 255);
        fanStatus = "FORWARD";
      } else if (temp >= 25 && temp <= 30 && humi > 80) {
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        // digitalWrite(ENB, HIGH);
        // analogWrite(ENB, 255);
        // ledcWrite(ENB, 255);
        fanStatus = "REVERSE";
      // } else if (temp < 25 && humi < 50) {
      } else {
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, LOW);
        // digitalWrite(ENB, LOW);
        // analogWrite(ENB, 0);
        // ledcWrite(ENB, 0);
        fanStatus = "OFF";
      }

      xSemaphoreGive(motorMutex);

      // Cập nhật trạng thái quạt vào struct
      sensorData.fanStatus = fanStatus;
      xQueueOverwrite(canQueue, &sensorData);
    }

    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}



void displayTask(void *parameter) {
  for (;;) {
    SensorData sensorData;
    if (xQueuePeek(canQueue, &sensorData, portMAX_DELAY)) {
      uint8_t moisture = sensorData.moisturePercent;
      int pwm = (moisture < 20) ? 255 : (moisture < 40) ? 100 : 0;
      String pumpStatus = (pwm > 0) ? "ON" : "OFF";

      xSemaphoreTake(displayMutex, portMAX_DELAY);

      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 0);
      display.print("ADC: "); display.println(sensorData.adcRaw);
      display.setCursor(0, 8);
      display.print("Do am dat: "); display.print(moisture); display.println("%");
      display.setCursor(0, 16);
      display.print("Toc do bom nuoc: "); display.println(pwm);
      // display.setCursor(0, 24);
      // display.print("Bom nuoc: "); display.println(pumpStatus);

      display.setCursor(0, 32);
      display.print("Do am kk: "); display.print(sensorData.airHumidity); display.println("%");

      display.setCursor(0, 40);
      display.print("Nhiet do kk: "); display.print(sensorData.temperature); display.println("C");

      
      display.setCursor(0, 48);
      display.print("Fan: "); display.println(sensorData.fanStatus);

      int bar_air_humidity = map(sensorData.airHumidity, 0, 100, 0, SCREEN_WIDTH - 2);
      display.drawRect(0, 24, SCREEN_WIDTH - 1, 6, SSD1306_WHITE);
      display.fillRect(1, 55, bar_air_humidity, 4, SSD1306_WHITE);


      // Thanh đo độ ẩm
      int barWidth = map(moisture, 0, 100, 0, SCREEN_WIDTH - 2);
      display.drawRect(0, 24, SCREEN_WIDTH - 1, 6, SSD1306_WHITE);
      display.fillRect(1, 25, barWidth, 4, SSD1306_WHITE);

      display.display();
      xSemaphoreGive(displayMutex);
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void grafanaTask(void *parameter) {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("\nWiFi connected");

  timeSync(TZ_INFO, "pool.ntp.org", "time.nis.gov");

  if (client.validateConnection()) {
    Serial.println("Connected to InfluxDB");
  } else {
    Serial.print("InfluxDB connection failed: ");
    Serial.println(client.getLastErrorMessage());
  }

  for (;;) {
    SensorData sensorData;
    if (xQueuePeek(canQueue, &sensorData, portMAX_DELAY)) {
      sensor.clearFields();
      sensor.addField("moisture", sensorData.moisturePercent);
      sensor.addField("adc", sensorData.adcRaw);
      sensor.addField("pwm", sensorData.pwm);
      sensor.addField("temperature", sensorData.temperature);
      sensor.addField("airHumidity", sensorData.airHumidity);
      sensor.addField("fanStatus", sensorData.fanStatus);

      sensor.addTag("device", "ESP32");

      if (!client.writePoint(sensor)) {
        Serial.print("InfluxDB write failed: ");
        Serial.println(client.getLastErrorMessage());
      } else {
        Serial.println("Data sent to InfluxDB");
      }
    }
    vTaskDelay(10000 / portTICK_PERIOD_MS);
  }
}

// ===== SETUP =====
void setup() {
  Serial.begin(9600);
  delay(1000);
  Wire.begin(OLED_SDA, OLED_SCL);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED init failed!");
    while (true);
  }
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Waiting...");
  display.display();

  spiAcc = new SPIClass(FSPI);
  spiAcc->begin(SPI_SCK, SPI_MISO, SPI_MOSI, SPI_CS);
  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  pinMode(IN1, OUTPUT); digitalWrite(IN1, HIGH);
  pinMode(IN2, OUTPUT); digitalWrite(IN2, LOW);
  pinMode(ENA, OUTPUT); analogWrite(ENA, 0);

  pinMode(IN3, OUTPUT); digitalWrite(IN3, LOW);
  pinMode(IN4, OUTPUT); digitalWrite(IN4, LOW);
  // pinMode(ENB, OUTPUT); analogWrite(ENB, 0);

  canQueue = xQueueCreate(1, sizeof(SensorData));
  displayMutex = xSemaphoreCreateMutex();
  motorMutex = xSemaphoreCreateMutex();

  xTaskCreate(readCANTask, "Read CAN", 4096, NULL, 3, NULL);
  xTaskCreate(controlMotorTask, "Motor Ctrl", 4096, NULL, 2, NULL);
  xTaskCreate(displayTask, "Display OLED", 4096, NULL, 2, NULL);
  xTaskCreate(controlFanTask, "Fan Ctrl", 4096, NULL, 2, NULL);
  xTaskCreate(grafanaTask, "Grafana", 8192, NULL, 1, NULL);

  vTaskDelete(NULL);  // Xóa task setup()
}

void loop() {
  // FreeRTOS dùng task thay thế loop()
}

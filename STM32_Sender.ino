#include <SPI.h>
#include <mcp2515.h>
#include <MapleFreeRTOS900.h>
#include <DHT.h>

// CAN
#define SPI_CS PA4  // MCP2515 CS
#define ADC_PIN PA0 // Đọc độ ẩm đất

// DHT11
#define DHT_PIN PA1
#define DHT_TYPE DHT11
DHT dht(DHT_PIN, DHT_TYPE);

// MCP2515
MCP2515* mcp2515;
struct can_frame canMsg;

// Dữ liệu cảm biến
struct SensorData {
  int adcRaw;              // Độ ẩm đất (ADC raw)
  int soilMoisturePercent; // Độ ẩm đất (%)
  int airHumidity;         // Độ ẩm không khí (%)
  int temperature;         // Nhiệt độ (°C)
};
SensorData sdata;

// Mutex + Semaphore
SemaphoreHandle_t dataMutex;
SemaphoreHandle_t canSendSemaphore;

// Chuyển ADC sang % độ ẩm đất
const int sensorMin = 600;
const int sensorMax = 4095;
int mapMoisture(int value) {
  int percent = map(value, sensorMin, sensorMax, 100, 0);
  return constrain(percent, 0, 100);
}

// Task đọc độ ẩm đất + DHT11
void taskReadSensors(void* parameters) {
  dht.begin();
  for (;;) {
    int adcRaw = analogRead(ADC_PIN);
    int soilPercent = mapMoisture(adcRaw);

    float h = dht.readHumidity();
    float t = dht.readTemperature();

    // Kiểm tra lỗi khi đọc DHT
    if (isnan(h) || isnan(t)) {
      h = -1;
      t = -1;
    }

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(300)) == pdTRUE) {
      sdata.adcRaw = adcRaw;
      sdata.soilMoisturePercent = soilPercent;
      sdata.airHumidity = (int)h;
      sdata.temperature = (int)t;
      xSemaphoreGive(dataMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(2000));  // Cập nhật mỗi 2 giây
  }
}

// Task gửi dữ liệu CAN
void taskSendCAN(void* parameters) {
  for (;;) {
    SensorData sendData;

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(300)) == pdTRUE) {
      sendData = sdata;
      xSemaphoreGive(dataMutex);

      // Gói tin CAN: 8 byte
      canMsg.can_id = 0x036;
      canMsg.can_dlc = 8;
      canMsg.data[0] = sendData.adcRaw & 0xFF;
      canMsg.data[1] = (sendData.adcRaw >> 8) & 0xFF;
      canMsg.data[2] = sendData.soilMoisturePercent;
      canMsg.data[3] = sendData.airHumidity;
      canMsg.data[4] = sendData.temperature;
      for (int i = 5; i < 8; i++) canMsg.data[i] = 0;  // Dự phòng

      mcp2515->sendMessage(&canMsg);

      xSemaphoreGive(canSendSemaphore);  // Tùy nhu cầu, có thể bỏ
    }

    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

void setup() {
  Serial.begin(9600);
  delay(1000);

  SPI.begin();
  mcp2515 = new MCP2515(SPI_CS);
  mcp2515->reset();
  mcp2515->setBitrate(CAN_125KBPS, MCP_8MHZ);
  mcp2515->setNormalMode();

  dataMutex = xSemaphoreCreateMutex();
  canSendSemaphore = xSemaphoreCreateBinary();

  xTaskCreate(taskReadSensors, "ReadSensors", 128, NULL, 2, NULL);
  xTaskCreate(taskSendCAN, "SendCAN", 128, NULL, 2, NULL);

  vTaskStartScheduler();
}

void loop() {}

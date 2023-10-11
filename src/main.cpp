#include <esp_now.h>
#include <WiFi.h>
#include <HardwareSerial.h>

HardwareSerial SerialPort(2); // use UART2
QueueHandle_t mac_queue;
QueueHandle_t data_queue;

void OnDataRecv(const uint8_t *mac, const uint8_t *data, int len)
{
  xQueueSend(mac_queue, mac, portMAX_DELAY);
  xQueueSend(data_queue, data, portMAX_DELAY);
}

void consumer_loop(void *pvParameters)
{
  Serial.printf("Serial publisher executing on core %d\n", xPortGetCoreID());
  char data[250];
  char mac[6];
  while (true)
  {
    xQueueReceive(mac_queue, &mac, portMAX_DELAY);
    xQueueReceive(data_queue, &data, portMAX_DELAY);
    SerialPort.printf("esp-now/%02x:%02x:%02x:%02x:%02x:%02x/%s\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], data);
    SerialPort.flush();

    Serial.printf("esp-now/%02x:%02x:%02x:%02x:%02x:%02x/%s\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], data);
  }
}

void setup()
{
  Serial.begin(115200);
  SerialPort.begin(15200, SERIAL_8N1, 16, 17);

  mac_queue = xQueueCreate(42, 6);
  data_queue = xQueueCreate(42, 250);
  if (mac_queue == NULL || data_queue == NULL)
  {
    Serial.println("Error creating the message queue!");
    return;
  }

  TaskHandle_t consumer_loop_task;
  xTaskCreatePinnedToCore(
      consumer_loop,        /* Task function. */
      "consumer_loop_task", /* name of task. */
      10000,                /* Stack size of task */
      NULL,                 /* parameter of the task */
      1,                    /* priority of the task */
      &consumer_loop_task,  /* Task handle to keep track of created task */
      0);                   /* pin task to core 0 */

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != 0)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);
  Serial.printf("ESP-NOW Listener executing on core %d\n", xPortGetCoreID());
  Serial.println("ESP32 NOW->UART started.");
}

void loop()
{
}
#include <Arduino.h>

#include <WiFi.h>
#include <esp_now.h>

// callback when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *data, int len)
{
    Serial.print("Last Packet Recv Data: ");
    for (int i = 0; i < len; i++)
    {
        Serial.print((char)data[i]);
    }
    Serial.println();
}

void setup()
{
    Serial.begin(115200);
    delay(2000);
    Serial.println("ESPNow Receiver");

    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);
    // WiFi.disconnect();

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK)
    {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // Once ESPNow is successfully Init, we will register for recv CB to
    // get recv packer info
    esp_now_register_recv_cb(OnDataRecv);
}

void loop()
{
}
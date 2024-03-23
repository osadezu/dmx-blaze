#include <Arduino.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <WiFiClientSecure.h>
#include <esp_dmx.h>
#include <WebSocketsClient.h>

#include <LocalConfig.h>

WiFiMulti wiFiMulti;
WebSocketsClient webSocket;

#define USE_SERIAL Serial

const int ledPin = 13;
const int transmitPin = 17;
const int receivePin = 16;
const int enablePin = 4;

const dmx_port_t dmxPort = DMX_NUM_1;
uint8_t data[DMX_PACKET_SIZE] = {0};

bool dmxIsConnected = false;
unsigned long lastFrame = 0;

void webSocketEvent(WStype_t type, uint8_t *payload, size_t length)
{
  USE_SERIAL.printf("[WSc] Event Start: %d ", millis());

  switch (type)
  {
  case WStype_DISCONNECTED:
    USE_SERIAL.printf("[WSc] Disconnected! ");
    break;
  case WStype_CONNECTED:
    USE_SERIAL.printf("[WSc] Connected to url: %s ", payload);
    break;
  case WStype_TEXT:
    // USE_SERIAL.printf("[WSc] get text: %s\n", payload);
    USE_SERIAL.printf("[WSc] Got text. ");

    // send message to server
    // webSocket.sendTXT("{\"brightness\": 0.5}");
    break;
  case WStype_BIN:
    USE_SERIAL.printf("[WSc] Got binary length: %u ", length);
    // hexdump(payload, length);
    // send data to server
    // webSocket.sendBIN(payload, length);
    break;
  case WStype_ERROR:
    USE_SERIAL.printf("[WSc] Got error. ");
    break;
  case WStype_FRAGMENT_TEXT_START:
    USE_SERIAL.printf("[WSc] Got fragment text start. ");
    break;
  case WStype_FRAGMENT_BIN_START:
    USE_SERIAL.printf("[WSc] Got fragment binary start. ");
    break;
  case WStype_FRAGMENT:
    USE_SERIAL.printf("[WSc] Got fragment. ");
    break;
  case WStype_FRAGMENT_FIN:
    USE_SERIAL.printf("[WSc] Got fragment fin. ");
    break;
  }
  USE_SERIAL.printf("[WSc] Done: %d\n", millis());
}

void setup()
{
  pinMode(ledPin, OUTPUT);

  Serial.begin(921600);

  const dmx_config_t config = DMX_CONFIG_DEFAULT;
  const int personality_count = 1;
  dmx_personality_t personalities[] = {{11, "11 Channel Mode"}};
  dmx_driver_install(dmxPort, &config, personalities, personality_count);
  dmx_set_pin(dmxPort, transmitPin, receivePin, enablePin);

  wiFiMulti.addAP(WIFI_SSID, WIFI_PW);
  while (wiFiMulti.run() != WL_CONNECTED)
  {
    delay(100);
  }

  webSocket.begin("192.168.0.130", 81, "/");
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000);

  Serial.println("Setup done!");
}

void loop()
{
  dmx_packet_t packet_info;
  int packetSize = dmx_receive(dmxPort, &packet_info, DMX_TIMEOUT_TICK);

  if (packetSize > 0)
  {
    // Serial.println("DMX Received!");
    unsigned long thisFrame = millis();

    if (!packet_info.err)
    {
      if (!dmxIsConnected)
      {
        // Serial.println("DMX connected!");
        digitalWrite(ledPin, HIGH);
        dmxIsConnected = true;
      }

      dmx_read(dmxPort, data, packet_info.size);

      if (thisFrame - lastFrame >= 40)
      {
        float brightness = static_cast<float>(data[26]) / 255;
        char message[22];
        snprintf_P(message, sizeof(message), PSTR("{\"brightness\":%6.4f}"), brightness);
        webSocket.sendTXT(message);

        // Serial.printf("Brightness:%d,", data[26]);
        // Serial.printf("R:%d,", data[27]);
        // Serial.printf("G:%d,", data[28]);
        // Serial.printf("B:%d,", data[29]);
        // Serial.printf("UV:%d,", data[30]);
        // Serial.println();

        Serial.printf("Received: %d ", thisFrame);
        Serial.printf("Data: %d -> ", data[26]);
        Serial.print(brightness);
        // Serial.print(" ");
        // Serial.print(message);
        // Serial.printf(" Done: %d", millis());
        Serial.println();

        lastFrame = thisFrame;
      }
    }
    else
    {
      Serial.println("A DMX error occurred.");
    }
  }
  else if (dmxIsConnected)
  {
    // Serial.println("DMX disconnected.");
    digitalWrite(ledPin, LOW);
    dmxIsConnected = false;
  }

  webSocket.loop();
}

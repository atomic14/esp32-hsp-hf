#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <I2SMEMSSampler.h>
#include <ADCSampler.h>
#include "transports/WebSocketTransport.h"
#include "Application.h"
#include "config.h"

void setup()
{
  Serial.begin(115200);
  Serial.println("Starting up");
  delay(1000);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  // disable WiFi sleep mode
  WiFi.setSleep(WIFI_PS_NONE);

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.print(WiFi.localIP());
  Serial.println("");

  // startup MDNS
  if (!MDNS.begin(MDNS_DOMAIN))
  {
    Serial.println("MDNS.begin failed");
  }
  else
  {
    Serial.printf("MDNS started use http://%s to conect\n", MDNS_DOMAIN);
  }
  Serial.println("Creating microphone");
#ifdef USE_I2S_MIC_INPUT
  I2SSampler *input = new I2SMEMSSampler(I2S_NUM_0, i2s_mic_pins, i2s_mic_Config);
#else
  I2SSampler *input = new ADCSampler(ADC_UNIT_1, ADC1_CHANNEL_7, i2s_adc_config);
#endif
  Application *application = new Application();
  application->begin(input, new WebSocketTransport());
}

void loop()
{
  vTaskDelay(pdMS_TO_TICKS(1000));
}
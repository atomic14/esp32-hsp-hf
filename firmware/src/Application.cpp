#include <FreeRTOS.h>
#include "Application.h"
#include "transports/Transport.h"
#include "I2SSampler.h"

void Application::begin(I2SSampler *input, Transport *transport)
{
  this->input = input;
  this->transport = transport;
  this->input->start();
  this->transport->begin();
  TaskHandle_t task_handle;
  xTaskCreate(Application::streamer_task, "task", 8192, this, 0, &task_handle);
}

void Application::streamer_task(void *param)
{
  Application *app = (Application *)param;
  // now just read from the microphone and send to the clients
  int16_t *samples = (int16_t *)malloc(sizeof(int16_t) * 1024);
  while (true)
  {
    // read from the microphone
    int samples_read = app->input->read(samples, 1024);
    // send to the transport
    app->transport->send(samples, samples_read * sizeof(int16_t));
  }
}

#pragma once

class Transport;
class I2SSampler;

class Application
{
private:
  Transport *transport = NULL;
  I2SSampler *input = NULL;

public:
  void begin(I2SSampler *input, Transport *transport);
  static void streamer_task(void *param);
};
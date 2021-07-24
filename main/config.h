#pragma once

#include <driver/i2s.h>

// if you want to use ADC input then you need to use I2S_NUM_0
#define I2S_MIC_DEVICE I2S_NUM_0

// I2S Microphone Settings
// Which channel is the I2S microphone on? I2S_CHANNEL_FMT_ONLY_LEFT or I2S_CHANNEL_FMT_ONLY_RIGHT
// Generally they will default to LEFT - but you may need to attach the L/R pin to GND
#define I2S_MIC_CHANNEL I2S_CHANNEL_FMT_ONLY_LEFT
// #define I2S_MIC_CHANNEL I2S_CHANNEL_FMT_ONLY_RIGHT
#define I2S_MIC_SERIAL_CLOCK GPIO_NUM_26
#define I2S_MIC_LEFT_RIGHT_CLOCK GPIO_NUM_22
#define I2S_MIC_SERIAL_DATA GPIO_NUM_21

// how often should we check for new samples from the microphone
#define DRIVER_POLL_INTERVAL_MS 5
// DMA settings
#define DMA_BUFFER_COUNT 10
// keep this fairly low to for low latency
#define DMA_BUFFER_SAMPLES 100

// speaker settings
#define I2S_SPEAKER_DEVICE I2S_NUM_1

#define I2S_SPEAKER_SERIAL_CLOCK GPIO_NUM_19
#define I2S_SPEAKER_LEFT_RIGHT_CLOCK GPIO_NUM_27
#define I2S_SPEAKER_SERIAL_DATA GPIO_NUM_18

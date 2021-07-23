/*
 * Copyright (C) 2018 BlueKitchen GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 * 4. Any redistribution, use, or modification is done solely for
 *    personal benefit and not for any commercial purpose or for
 *    monetary gain.
 *
 * THIS SOFTWARE IS PROVIDED BY BLUEKITCHEN GMBH AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL MATTHIAS
 * RINGWALD OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Please inquire about commercial licensing options at 
 * contact@bluekitchen-gmbh.com
 *
 */

#define BTSTACK_FILE__ "btstack_audio_esp32.c"

/*
 *  btstack_audio_esp32.c
 *
 *  Implementation of btstack_audio.h using polling ESP32 I2S driver
 *
 */

#include "btstack_config.h"
#include "btstack_debug.h"
#include "btstack_audio.h"
#include "btstack_run_loop.h"
#include "hal_audio.h"
#include "math.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/i2s.h"

#include "config.h"

// client
static void (*recording_callback)(const int16_t *buffer, uint16_t num_samples);

// timer to fill output ring buffer
static btstack_timer_source_t driver_timer_source;

// current gain
static int source_gain = 1;
// are we currently streaming from the microphone
static bool is_source_streaming;
// samples that are read from the microphone
static int32_t buffer_in[DMA_BUFFER_SAMPLES];
// samples that have been converted to PCM 16 bit
static int16_t samples_in[DMA_BUFFER_SAMPLES];

// read samples from the microphone and send them off to bluetooth
static void copy_samples(void)
{
    // read from I2S - we pass portMAX_DELAY so we don't delay
    size_t bytes_read = 0;
    i2s_read(I2S_MIC_DEVICE, buffer_in, DMA_BUFFER_SAMPLES * sizeof(int32_t), &bytes_read, portMAX_DELAY);
    // how many samples have we read
    int samples_read = bytes_read / sizeof(int32_t);
    if (samples_read > 0)
    {
        // convert the samples to 16 bit
        for (int i = 0; i < samples_read; i++)
        {
            // in theory we should shift to the right by 16 bits, but MEMS microphones have a very
            // high dynamic range, so if we shift all the way we lose a lot of signal
            samples_in[i] = source_gain * (buffer_in[i] >> 11);
        }
        // send the samples off to be processed
        (*recording_callback)((int16_t *)samples_in, samples_read);
    }
}

// callback from the timer
static void driver_timer_handler_source(btstack_timer_source_t *ts)
{
    // if we're streaming from the microphone then copy the samples from the I2S device
    if (recording_callback)
    {
        copy_samples();
    }
    // re-set timer
    btstack_run_loop_set_timer(ts, DRIVER_POLL_INTERVAL_MS);
    btstack_run_loop_add_timer(ts);
}

// setup the I2S driver
static int btstack_audio_esp32_source_init(
    uint8_t channels,
    uint32_t samplerate,
    void (*recording)(const int16_t *buffer, uint16_t num_samples))
{
    recording_callback = recording;
    i2s_config_t config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_RX,
        .sample_rate = samplerate,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = channels == 2 ? I2S_CHANNEL_FMT_RIGHT_LEFT : I2S_MIC_CHANNEL,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .dma_buf_count = DMA_BUFFER_COUNT, // Number of DMA buffers. Max 128.
        .dma_buf_len = DMA_BUFFER_SAMPLES, // Size of each DMA buffer in samples. Max 1024.
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1};
    i2s_pin_config_t pins = {
        .bck_io_num = I2S_MIC_SERIAL_CLOCK,
        .ws_io_num = I2S_MIC_LEFT_RIGHT_CLOCK,
        .data_out_num = -1,
        .data_in_num = I2S_MIC_SERIAL_DATA};

    i2s_driver_install(I2S_MIC_DEVICE, &config, 0, NULL);
    i2s_set_pin(I2S_MIC_DEVICE, &pins);
    i2s_zero_dma_buffer(I2S_MIC_DEVICE);

    return 0;
}

// update the gain
static void btstack_audio_esp32_source_gain(uint8_t gain)
{
    source_gain = gain;
}

// start streaming from the microphone
static void btstack_audio_esp32_source_start_stream()
{
    // start i2s
    i2s_start(I2S_MIC_DEVICE);

    // start timer
    btstack_run_loop_set_timer_handler(&driver_timer_source, &driver_timer_handler_source);
    btstack_run_loop_set_timer(&driver_timer_source, DRIVER_POLL_INTERVAL_MS);
    btstack_run_loop_add_timer(&driver_timer_source);

    // state
    is_source_streaming = true;
}

// stop streaming from the microphone
static void btstack_audio_esp32_source_stop_stream(void)
{
    if (!is_source_streaming)
        return;

    // stop timer
    btstack_run_loop_remove_timer(&driver_timer_source);

    // stop i2s
    i2s_stop(I2S_MIC_DEVICE);

    // state
    is_source_streaming = false;
}

// shutdown the driver
static void btstack_audio_esp32_source_close(void)
{
    if (is_source_streaming)
    {
        btstack_audio_esp32_source_stop_stream();
    }
    // uninstall driver
    i2s_driver_uninstall(I2S_MIC_DEVICE);
}

static const btstack_audio_source_t btstack_audio_source_esp32 = {
    /* int (*init)(..);*/ &btstack_audio_esp32_source_init,
    /* void (*set_gain)(uint8_t gain); */ &btstack_audio_esp32_source_gain,
    /* void (*start_stream(void));*/ &btstack_audio_esp32_source_start_stream,
    /* void (*stop_stream)(void)  */ &btstack_audio_esp32_source_stop_stream,
    /* void (*close)(void); */ &btstack_audio_esp32_source_close};

const btstack_audio_source_t *btstack_audio_esp32_source_get_instance(void)
{
    return &btstack_audio_source_esp32;
}

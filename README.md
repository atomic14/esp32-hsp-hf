[![ko-fi](https://ko-fi.com/img/githubbutton_sm.svg)](https://ko-fi.com/Z8Z734F5Y)

# Intro

You can watch a video of this working here: https://youtu.be/0jR-QNTfydA at around 4:23

[![Demo Video](https://img.youtube.com/vi/0jR-QNTfydA/0.jpg)](https://www.youtube.com/watch?v=0jR-QNTfydA)

This is a slight rework of the headset profile handsfree demo from here: https://github.com/bluekitchen/btstack

I've added in support for microphone input and pulled the speaker output into the demo code so that it can be more easily customised.

I'll try and clean this code up further, but I would not recommend trying to use this code unless you really know what you are doing and know how to use the esp-idf.

# Setup

You'll need the esp-idf setup. Follow the instructions here: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/

You'll also need the [bluekitchen/btstack](https://github.com/bluekitchen/btstack)

Follow the instructions here: https://github.com/bluekitchen/btstack/tree/master/port/esp32

Change the value in `hfp_hf_demo.c` for `device_addr_string` to match the device you want to connect to (use the mac address of the Bluetooth interface)

```
static const char *device_addr_string = "08:c7:29:06:84:27";
```

# Build

In the `esp-idf` folder run:

```
. ./export.sh
```

Then in the projects folder run:

```
make
```

Then to flash, run this command:

```
ESPPORT=/dev/tty.SLAB_USBtoUART make flash
```

And then

```
ESPPORT=/dev/tty.SLAB_USBtoUART make monitor
```

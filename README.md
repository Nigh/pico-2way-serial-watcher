# pico USB CDC+HID template
rpi pico USB CDC+HID template

`PID=0x4005`
`VID=0xACDC`

## Prepare

### Get pico SDK

```shell
git clone https://github.com/raspberrypi/pico-sdk --depth=1 --recurse-submodules --shallow-submodules -j8
cd pico-sdk

## Add pico-sdk Path to your environment
echo export PICO_SDK_PATH=$PWD >> ~/.profile
```

### Install dependencies

```shell
sudo apt update && sudo apt install -y cmake make ninja-build gcc g++ openssl libssl-dev cmake gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib clang-format
```

### build

```shell
# build
make
# clang-format
make format
# clear build
make clean
# rebuild
make rebuild
```

## Usage

- Pico will enumerate two USB devices, specifically a `CDC` device and an `HID` device. 
- The log will be printed out via the `CDC` serial port. 
- Any data frame written to the `HID` device will be printed out from the `CDC` serial port, and the `HID` will also return a data frame with each byte incremented by 1.

STM32MP157C 4xADC RPMSG test firmware
----

This code is based on the cmake template for STM32MP157C which is located [here](https://github.com/dimtass/stm32mp1-cmake-template).

This firmware implements a 4x ADC read using DMA on the CM4. Then it sends those
values to the CA7 using OpenAMP.

> Note: There is a blog post [here](https://www.stupid-projects.com/using-elastic-stack-elk-on-embedded-part-2/)
which explains how to use this firmware. This firmware implements the lower part of the
adcsampler elastic beat which is located [here](https://github.com/dimtass/adcsamplerbeat).

The firmware is able to read the 4x ADCs from the Arduino connector on the STM32MP157C
(A0 - A3) and then sends those values via OpenAMP on the application CPU (CA7). The CA7
is then able to read those values from the `/dev/ttyRPMSG0` serial port.

> Note: The pdf user manual UM2534 (document dm00591354) has wrong pin assignments.
I've raised this issue [here](https://community.st.com/s/question/0D73W000000Uby8/mistake-in-stm32mp1dk2-schematics)
and it seems it will take some time to resolve.

The correct pinmux for the ADC pins on the Arduino connector of the STM32MP157C is:

```
ARD_A0 : PF14 -> ADC2_IN6
ARD_A1 : PF13 -> ADC2_IN2
ARD_A2 : ANA0 -> ADC1_IN0
ARD_A3 : ANA1 -> ADC1_IN1
```

Or also

```
ARD_A0 : PF14 -> ADC2_IN6
ARD_A1 : PF13 -> ADC2_IN2
ARD_A2 : ANA0 -> ADC2_IN0
ARD_A3 : ANA1 -> ADC2_IN1
```

In this firmware I'm using the second one.

## Build the CM firmware
To build the firmware you need to clone the repo in any directory and then inside
that directrory run the command:

```sh
./build.sh
```

The above command assumes that you have a toolchain in your `/opt` folder. In case,
you want to point to a specific toolchain path, then run:

```sh
TOOLCHAIN_DIR=/path/to/toolchain SRC=src_hal ./build.sh
```

Or you can edit the `build.sh` script and add your toolchain path.

It's better to use Docker to build the image. To do that run this command:
```sh
docker run --rm -it -v $(pwd):/tmp -w=/tmp dimtass/stm32-cde-image:latest -c "SRC=src_hal ./build.sh"
```

In order to remove any previous builds, then run:
```sh
docker run --rm -it -v $(pwd):/tmp -w=/tmp dimtass/stm32-cde-image:latest -c "CLEANBUILD=true SRC=src_hal ./build.sh"
```

## Loading the firmware to CM4
To load the firmware on the Cortex-M4 MCU you need to scp the firmware `.elf` file in the
`/lib/firmware` folder of the Linux instance of the STM32MP1. Then you also need to copy the
`fw_cortex_m4.sh` script on the `/home/root` (or anywhere you like) and then run this command
as root.
```sh
./fw_cortex_m4.sh start
```

To stop the firmware run:
```sh
./fw_cortex_m4.sh stop
```

> Note: The console of the STM32MP1 is routed in the micro-USB connector `STLINK CN11` which
in case of my Ubuntu shows up as `/dev/ttyACMx`.

When you copy the `./fw_cortex_m4.sh` you need also to enable the execution flag with:
```sh
chmod +x fw_cortex_m4.sh
```

If the firmware is loaded without problem you should see an output like this:
```sh
fw_cortex_m4.sh: fmw_name=stm32mp157c-rpmsg-test.elf
[70696.118168] remoteproc remoteproc0: powering up m4
[70696.124096] remoteproc remoteproc0: Booting fw image stm32mp157c-rpmsg-test.elf, size 1115364
[70696.184680] rproc-srm-core mlahb:m4@10000000:m4_system_resources: bound mlahb:m4@10000000:m4_system_resources:timer@40000000 (ops 0xc0cfbd7c)
[70696.208147] rproc-srm-core mlahb:m4@10000000:m4_system_resources: bound mlahb:m4@10000000:m4_system_resources:i2c@40015000 (ops 0xc0cfbd7c)
[70696.254391] rproc-srm-core mlahb:m4@10000000:m4_system_resources: bound mlahb:m4@10000000:m4_system_resources:dac@40017000 (ops 0xc0cfbd7c)
[70696.265830] rproc-srm-core mlahb:m4@10000000:m4_system_resources: bound mlahb:m4@10000000:m4_system_resources:serial@40018000 (ops 0xc0cfbd7c)
[70696.299249] rproc-srm-core mlahb:m4@10000000:m4_system_resources: bound mlahb:m4@10000000:m4_system_resources:timer@44000000 (ops 0xc0cfbd7c)
[70696.310675] rproc-srm-core mlahb:m4@10000000:m4_system_resources: bound mlahb:m4@10000000:m4_system_resources:spi@44005000 (ops 0xc0cfbd7c)
[70696.323216] rproc-srm-core mlahb:m4@10000000:m4_system_resources: bound mlahb:m4@10000000:m4_system_resources:dma@48001000 (ops 0xc0cfbd7c)
[70696.335975] rproc-srm-core mlahb:m4@10000000:m4_system_resources: bound mlahb:m4@10000000:m4_system_resources:adc@48003000 (ops 0xc0cfbd7c)
[70696.348405] rproc-srm-core mlahb:m4@10000000:m4_system_resources: bound mlahb:m4@10000000:m4_system_resources:hash@4c002000 (ops 0xc0cfbd7c)
[70696.361033] rproc-srm-core mlahb:m4@10000000:m4_system_resources: bound mlahb:m4@10000000:m4_system_resources:rng@4c003000 (ops 0xc0cfbd7c)
[70696.373738] rproc-srm-core mlahb:m4@10000000:m4_system_resources: bound mlahb:m4@10000000:m4_system_resources:crc@4c004000 (ops 0xc0cfbd7c)
[70696.386138] rproc-srm-core mlahb:m4@10000000:m4_system_resources: bound mlahb:m4@10000000:m4_system_resources:cryp@4c005000 (ops 0xc0cfbd7c)
[70696.398770] rproc-srm-core mlahb:m4@10000000:m4_system_resources: bound mlahb:m4@10000000:m4_system_resources:button (ops 0xc0cfbd7c)
[70696.410813] rproc-srm-core mlahb:m4@10000000:m4_system_resources: bound mlahb:m4@10000000:m4_system_resources:m4_led (ops 0xc0cfbd7c)
[70696.423010]  mlahb:m4@10000000#vdev0buffer: assigned reserved memory node vdev0buffer@10042000
[70696.432182] virtio_rpmsg_bus virtio0: rpmsg host is online
[70696.437167]  mlahb:m4@10000000#vdev0buffer: registered virtio0 (type 7)
[70696.443501] remoteproc remoteproc0: remote processor m4 is now up
[70696.449826] virtio_rpmsg_bus virtio0: creating channel rpmsg-tty-channel addr 0x0
 ```

This means that the firmware is loaded and the virtual tty port is mapped.

## Testing the firmware
When this example firmware loads then two new tty ports will be created in the Linux side,
which are `/dev/ttyRPMSG0` and `/dev/ttyRPMSG1`. Now to test that the firmware is working
properly run these commands on the Linux terminal (in this case we use only one port).

```sh
stty -onlcr -echo -F /dev/ttyRPMSG0
cat /dev/ttyRPMSG0 &
echo "start" >/dev/ttyRPMSG0
```

You should see something like that:
```
ADC[2.2]:4014,3706,4010,4006
ADC[2.1]:3986,3702,4022,4026
ADC[2.2]:3986,3702,4022,4026
ADC[2.1]:4018,3710,4006,4010
ADC[2.2]:4018,3710,4006,4010
```

## Debug serial port
The firmware also supports a debug UART on the CM4. This port is mapped to UART7 and the
Arduino connector pins. The pinmap is the following:

pin | Function
-|-
D0 | Rx
D1 | Tx

You can connect a USB-to-UART module to those pins and the GND and then open the tty port
on your host. The port supports 115200 baudrate. When the firmware loads on the CM4 then
you should see this messages:

```sh
[00000.008][INFO ]Cortex-M4 boot successful with STM32Cube FW version: v1.2.0
[00000.015][INFO ]Virtual UART0 OpenAMP-rpmsg channel creation
[00000.021][INFO ]Virtual UART1 OpenAMP-rpmsg channel creation
```

## Using the cmake template in Yocto
TBD

## License
Just MIT.

## Author
Dimitris Tassopoulos <dimtass@gmail.com>
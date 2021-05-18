## EEL7415/7515 - IoT LoRa

This branch contains the firmware for EEL7415/7515 courses, which is based on [I-CUBE-LRWAN](https://www.st.com/en/embedded-software/i-cube-lrwan.html) project. The LoRaWAN stack version is [1.0.3](https://lora-alliance.org/resource_hub/lorawan-specification-v1-0-3/). The build system is composed by GNU ARM-Toolchain.

#### Folders structure

    ├── Bin                   # Binary files.
    ├── Docs                  # Boards usefull files.
    ├── Drivers               # Sensors and HAL drivers.
    ├── Middlewares           # LoRaWAN stack.
    ├── Projects              # Main source code and Makefile
    └── ...

#### Prerequisites

It is highly recommended to use Linux Ubuntu version 18+ to compile the project.

* **GNU ARM-Toolchain (tested with versions 6.3.1 and 7.3.1)**

```sh
$ sudo apt-get install gcc-arm-none-eabi
```

* **Make**

```sh
$ sudo apt-get install make
```

#### Compiling the code

Go to the folder that contains the Makefile file:

```sh
$ cd Projects/B-L072Z-LRWAN1/Applications/LoRa/End_Node/gcc/B-L072Z-LRWAN1/
```

To compile:

```sh
$ make
```

The binary file will be located in the same folder. To clean the build files, use: ```make clean```.

#### Flashing the binary

To upload the firmware to the board, just copy the binary file to the USB device created when the B-L072Z-LRWAN1 is connected to the computer.

Using the command line, this can be performed as:

```sh
$ cp end_node.bin /media/$your_user_name$/DIS_L072Z
```

#### Visualizing the messages through Serial

The UART interface is used to debug with prints. To access this interface, install the `Cutecom` program:

```sh
$ sudo apt-get install cutecom
```

To open the software:

```sh
$ sudo cutecom
```
The baudrate should be configured as 115200 bps.

#### Updating the code

The most relevant files are:

* Projects/B-L072Z-LRWAN1/Applications/LoRa/End_Node/LoRaWAN/App/src/**main.c**: project main application. This file contains `#define` that can be used to change some device configurations. It also has the functions of LoRaWAN parameters configuration, payload creation, stack callbacks and a FSM to handle the project routines.

* Projects/B-L072Z-LRWAN1/Applications/LoRa/End_Node/LoRaWAN/App/inc/**Commissioning.h**: file where the device keys, address and activation mode (ABP or OTAA) can be configured.

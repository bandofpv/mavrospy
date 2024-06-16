# rospi
Test repo for mavros scripts on board my rpi

## Setup:

### Wiring

This setup connects RPi to the Cube's `TELEM2` port, which is generally recommended for offboard control.

Connect the Pixhawk `TELEM2` `TX`/`RX`/`GND` pins to the complementary `RXD`/`TXD`/`Ground` pins on the RPi GPIO board:

| PX4 TELEM2 Pin | RPi GPIO Pin           |
| -------------- | ---------------------- |
| UART5_TX (2)   | RXD (GPIO 15 - pin 10) |
| UART5_RX (3)   | TXD (GPIO 14 - pin 8)  |
| GND (6)        | Ground (pin 6)         |

This diagram shows the RPi GPIO board pinout:

![](assets/rpi_gpio.png)

The standard `TELEM2` pin assignments are shown below:

| Pins      | Signal          | Voltage |
| --------- |-----------------| ------- |
| 1 (Red)   | VCC (out)       | +5V     |
| 2 (Black) | UART5_TX (out)  | +3.3V   |
| 3 (Black) | UART5_RX (in)   | +3.3V   |
| 4 (Black) | UART5_CTS (out) | +3.3V   |
| 5 (Black) | UART5_RTS (in)  | +3.3V   |
| 6 (Black) | GND             | GND     |

The RPi requires a separate 5V/3A power supply via the `5V power` and `Ground` pins. I would recommend soldering a 
battery elimination circuit (BEC) to your UAV's power leads.

### PX4 Parameters

Assuming you have installed the latest PX4 
[firmware](https://docs.px4.io/main/en/config/firmware.html#install-stable-px4), change the following 
[parameters](https://docs.px4.io/main/en/advanced_config/parameters.html) in QGroundControl:

```
   MAV_1_CONFIG = TELEM2
   UXRCE_DDS_CFG = 0 (Disabled)
   SER_TEL2_BAUD = 57600
```

### Enable UART Communication

This repository runs on ROS1 and is supported on ROS Noetic. If you haven't already, please install 
[Ubuntu MATE 20.04](https://releases.ubuntu-mate.org/20.04/arm64/) for the RPi (arm64) using the 
[Raspberry Pi Imager](https://www.raspberrypi.com/software/). 

Now, we need to enable UART communication on the RPi's GPIO pins.

Open the firmware boot configuration file:

`$ sudo nano /boot/firmware/config.txt`

Append the following text to the end of the file (after the last line):

```
enable_uart=1
dtoverlay=disable-bt
```

Then, save the file and restart the RPi. 

You can check that the serial port is available by issuing this command: 

`$ ls /dev/ttyserila0`

The result of the command should include the RX/TX connection `/dev/ttyAMA0`

### Test Connection

[MAVLink](https://mavlink.io/en/) is the default and stable communication interface for working with PX4.


We can test that the RPi and flight controller are communicating with each other via a simple developer MAVLink GCS 
called `mavproxy`.

Install MAVProxy:

```
$ sudo apt install python3-pip
$ sudo pip3 install mavproxy
$ sudo apt remove modemmanager
```

Run MAVProxy, setting the port to connect to `/dev/ttyserial0` and the baud rate to match the flight controller (57600):

```
$ sudo mavproxy.py --master=/dev/serial0 --baudrate 57600
```

MAVProxy on the RPi should now connect to the Cube via its RX/TX pins. You should be able to see this in the RPi terminal.

### TODO: DO I NEED PX4 INSTALL???

### Install ROS Noetic

[ROS](http://www.ros.org/) is a general purpose robotics library that can be used with PX4 for drone application 
development.
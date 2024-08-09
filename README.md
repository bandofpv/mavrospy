# mavrospy
ROS node to interact with [MAVROS](https://wiki.ros.org/mavros) for basic UAV control.

This current repo is supported on a PX4 flight controller with a RPi4 companion computer running Ubuntu 20.04 MATE with 
ROS Noetic. 

Documentation for motion capture utilizes the 
[Qualisys Track Manager]("https://www.qualisys.com/software/qualisys-track-manager/").  

## Usage:

### TODO: TALK ABOUT HOW TO USE CODE

## Setup:

This repository is supported on ROS Noetic. If you haven't already, please install 
[Ubuntu MATE 20.04](https://releases.ubuntu-mate.org/20.04/arm64/) for the RPi4 (arm64) using the 
[Raspberry Pi Imager](https://www.raspberrypi.com/software/). 

Select the `Log in automatically` option upon first boot. 

**Note:** If prompted to upgrade to a newer version of Ubuntu, selected `Don't Upgrade`. ROS Noetic is primarily 
targeted at the Ubuntu 20.04 (Focal) release.

### Wiring

This setup connects the RPi4 to the flight controller's `TELEM2` port, which is generally recommended for offboard 
control.

Connect the flight controller's `TELEM2` `TX`/`RX`/`GND` pins to the complementary `RXD`/`TXD`/`Ground` pins on the RPi4 
GPIO board:

| PX4 TELEM2 Pin | RPi4 GPIO Pin          |
|----------------|------------------------|
| UART5_TX (2)   | RXD (GPIO 15 - pin 10) |
| UART5_RX (3)   | TXD (GPIO 14 - pin 8)  |
| GND (6)        | Ground (pin 6)         |

This diagram shows the RPi4 GPIO board pinout:

![](assets/rpi_gpio.png)

The standard `TELEM2` pin assignments are shown below:

| Pins      | Signal          | Voltage |
|-----------|-----------------|---------|
| 1 (Red)   | VCC (out)       | +5V     |
| 2 (Black) | UART5_TX (out)  | +3.3V   |
| 3 (Black) | UART5_RX (in)   | +3.3V   |
| 4 (Black) | UART5_CTS (out) | +3.3V   |
| 5 (Black) | UART5_RTS (in)  | +3.3V   |
| 6 (Black) | GND             | GND     |

The RPi4 requires a separate 5V/3A power supply via the `5V power` and `Ground` pins. I would recommend soldering a 
battery elimination circuit (BEC) to your UAV's power leads.

### TODO: Diode for uart pin?? or just wait to plug in after full boot

### PX4 Parameters

Assuming you have installed the latest PX4 
[firmware](https://docs.px4.io/main/en/config/firmware.html#install-stable-px4), change the following 
[parameters](https://docs.px4.io/main/en/advanced_config/parameters.html) in QGroundControl:

```
   MAV_1_CONFIG = TELEM2
   UXRCE_DDS_CFG = 0 
   SER_TEL2_BAUD = 921600
```

### Enable UART Communication

Open the firmware boot configuration file:

```
$ sudo nano /boot/firmware/config.txt
```

Append the following text to the end of the file (after the last line):

```
enable_uart=1
dtoverlay=disable-bt
```

Save and exit the file. 

Run this command to add your user to the `dialout` group:

```
$ sudo adduser ${USER} dialout
```

Restart the RPi. 

You can check that the serial port is available by issuing this command: 

```
$ ls /dev/serial0
```

The result of the command should include the RX/TX connection `/dev/serial0`

### Test Connection

[MAVLink](https://mavlink.io/en/) is the default and stable communication interface for working with PX4.


We can test that the RPi and flight controller are communicating with each other via a MAVLink GCS called `mavproxy`.

Install MAVProxy:

```
$ sudo apt install python3-pip
$ sudo pip3 install mavproxy
$ sudo apt remove modemmanager
```

Run MAVProxy, setting the port to connect to `/dev/serial0` and the baud rate to match the flight controller (57600):

```
$ sudo mavproxy.py --master=/dev/serial0 --baudrate 921600
```

MAVProxy on the RPi should now connect to the flight controller via its RX/TX pins. You should be able to see this in the 
RPi terminal. **CTRL+C** to exit.

### Install ROS Noetic

[ROS](http://www.ros.org/) is a general purpose robotics library that can be used with PX4 for drone application development.

These instructions are a simplified version of the [official installation guide](https://wiki.ros.org/noetic/Installation/Ubuntu).

Set up your computer to accept software from packages.ros.org: 

```
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

Set up your keys: 

```
$ sudo apt install curl 
$ curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

Make sure your Debian package index is up-to-date:

```
$ sudo apt update
```

Install ROS-Base (Bare Bones):

```
$ sudo apt install ros-noetic-ros-base
```

You must source this script in every bash terminal you use ROS in.

```
$ source /opt/ros/noetic/setup.bash
```

It can be convenient to automatically source this script every time a new shell is launched. These commands will do that 
for you:

```
$ echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```

### Test ROS Noetic

You can test ROS was installed properly by verifying its version:

```
$ rosversion -d
```

It should return `noetic`.

### Install MAVROS

[MAVROS](https://wiki.ros.org/mavros) is a ROS 1 package that enables MAVLink extendable communication between computers 
running ROS 1 for any MAVLink enabled autopilot, ground station, or peripheral. MAVROS is the "official" supported 
bridge between ROS 1 and the MAVLink protocol.

These instructions are a simplified version of the 
[official installation guide](https://github.com/mavlink/mavros/blob/master/mavros/README.md#installation)

Enter the following command to install MAVROS:

```
$ sudo apt install ros-noetic-mavros ros-noetic-mavros-extras ros-noetic-mavros-msgs
```

Then install [GeographicLib](https://geographiclib.sourceforge.io/) datasets by running the 
`install_geographiclib_datasets.sh` script:

```
$ wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
$ sudo bash ./install_geographiclib_datasets.sh
```

### Test MAVROS

You can test MAVROS is working correctly by issuing the following command:

```
$ roslaunch mavros px4.launch fcu_url:=/dev/serial0:921600
```

This will launch the mavros node and connect to your flight controller. **CTRL+C** to exit.

### Install MAVROSPY

Create a [catkin workspace](https://wiki.ros.org/catkin/workspaces): 

```
$ mkdir -p ~/catkin_ws/src
```

Clone `mavrospy` in your catkin workspace and build with `catkin_make`: 

```
$ sudo apt install git
$ cd ~/catkin_ws/src
$ git clone https://github.com/bandofpv/mavrospy.git
$ cd ..
$ catkin_make
```

Similar with ROS, you must source this script in every bash terminal you use MAVROSPY in.

```
$ source ~/catkin_ws/devel/setup.bash
```

It can be convenient to automatically source this script every time a new shell is launched. These commands will do that 
for you:

```
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```

### Test MAVROSPY

You can test MAVROSPY is working correctly by issuing the following command: 

```
$ roslaunch mavrospy control_test.launch fcu_url:=/dev/serial0:921600
```

### TODO: write what to expect... note will make breaks after adding mode check

### Start ROS Nodes at Boot

In order to start the mavros node and mavrospy node at boot, we need to create a couple of shell scripts and services. 

Create a separate file that sets your `ROS_MASTER_URI` that we can source when launching ROS Core or any ROS launch:

```
$ sudo mkdir /etc/ros
$ sudo nano /etc/ros/env.sh
```

Put the following lines into the file: 

```
#!/bin/sh
export ROS_MASTER_URI=http://localhost:11311
```

Save and exit the file. 

Create a system service to source all the appropriate environment variables and run ROS core: 

```
$ sudo nano /etc/systemd/system/roscore.service
```

Put the following lines into the file, changing `your_username` to your RPi's username:

```
[Unit]
Description=ROScore service
After=network-online.target

[Service]
Type=forking
User=your_username
ExecStart=/bin/sh -c ". /opt/ros/noetic/setup.sh; . /etc/ros/env.sh; roscore & while ! echo exit | nc localhost 11311 > /dev/null; do sleep 1; done"

[Install]
WantedBy=multi-user.target
```

Save and exit the file.

Create a system service to source all the MAVROSPY environment variables and run ROS launch: 

```
$ sudo nano /usr/sbin/mavrospy_auto_launch
```

Put the following lines into the file, changing both occurrences of `your_username` to your RPi's username:

```
#!/bin/bash
source $(echo ~your_username)/catkin_ws/devel/setup.bash

source /etc/ros/env.sh

export ROS_HOME=$(echo ~your_username)/.ros

roslaunch mavrospy control_test.launch fcu_url:=/dev/serial0:921600 --wait

exit 125
```

Save and exit the file.

**Note:** : you can modify the launch file to whatever .launch file of your choosing. Example launch files can be found 
in the [launch](https://github.com/bandofpv/mavrospy/tree/main/launch) directory. 

Make the file executable:

```
$ sudo chmod +x /usr/sbin/mavrospy_auto_launch
```

Create a system process to call the shell script above:

```
$ sudo nano /etc/systemd/system/ros_package.service
```

Put the following lines into the file, changing `your_username` to your RPi's username:

```
[Unit]
Requires=roscore.service
After=network-online.target roscore.service

[Service]
Type=simple
User=your_username
ExecStart=/usr/sbin/mavrospy_auto_launch

[Install]
WantedBy=multi-user.target
```

To have the services called on start up we need to make the system aware of the new service files and then enable them.

```
$ sudo systemctl daemon-reload
$ sudo systemctl enable roscore.service ros_package.service
```

Restart the RPi. 

You can verify the services are operational by issuing the following commands:

```
$ sudo systemctl status roscore.service ros_package.service
```

You should see `Active: active (running)` somewhere in the status report for both services. **CTRL+C** to exit.

You're all set! When you boot up our UAV, switch to `ONBOARD` mode and watch it fly!

## Qualisys Motion Capture

Motion capture allows for non-GPS navigation by sending pose estimation data to the flight controller's EKF. ROS makes 
this easy as there are drivers and packages that allow us to publish pose topics into the ROS framework. We can then 
relay those topics directly into MAVROS which will then send the poses to the flight controller. 

This assumes that you have already set up your Qualisys motion capture system and configured a rigid body for your UAV. 

QTM is only supported via Windows, so we will have to use another computer utilizing a Linux distro o act as our ROS 
pose publisher. The best practice is to connect these two machines via Ethernet to avoid latency and packet loss. 

### Install ROS Noetic

We need to install ROS on our publishing computer to be able to publish ROS pose topics to your UAV. 

Please note that you must use [Ubuntu 20.04]("https://releases.ubuntu.com/focal/") as that is the supported Ubuntu OS 
supported on ROS Noetic.

Set up your computer to accept software from packages.ros.org:

```
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

Set up your keys:

```
$ sudo apt install curl 
$ curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

Make sure your Debian package index is up-to-date:

```
$ sudo apt update
```

Install Desktop-Full:

```
$ sudo apt install ros-noetic-desktop-full
```

You must source this script in every bash terminal you use ROS in.

```
$ source /opt/ros/noetic/setup.bash
```

It can be convenient to automatically source this script every time a new shell is launched. These commands will do that
for you:

```
$ echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
$ source ~/.bashrc
```

### Install Motion Capture ROS Driver

To integrate QTM with ROS, we can use this [ROS Driver]("https://github.com/KTH-SML/motion_capture_system/tree/master"). 

Create a [catkin workspace](https://wiki.ros.org/catkin/workspaces) in your publishing computer:

```
$ mkdir -p ~/catkin_ws/src
```

Clone the `motion_capture_system` in your catkin workspace and build with `catkin_make`:

```
$ sudo apt install git
$ cd ~/catkin_ws/src
$ git clone https://github.com/KTH-SML/motion_capture_system.git
$ cd ..
$ catkin_make
```

### Setting up SSH

SSH allows us to remotely issue commands to the RPi over Wi-Fi.

Install the `openssh-server` package on your RPi: 

```
$ sudo apt install openssh-server
```

The SSH service will start automatically. We can verify this by entering:

```
$ sudo systemctl status ssh
```

You should see `Active: active (running)` somewhere in the status report. **CTRL+C** to exit.

On your publishing computer, SSH to your RPi. Make sure you change `username` to your RPi's username and `ip_address` to 
your RPi's IP address: 

```
$ ssh username@ip_address
```

Remember, you can always check your IP address via: 

```
$ ifconfig
```

We can now issue commands to our RPi via SSH rather than having it hooked up to a monitor with a mouse and keyboard. 

# TODO Add instructions for flight controller parameters!!!

### Publish Motion Capture Pose Topic

In order to publish ROS pose topics from our computer to the RPi, we must establish a 
[ROS Master]("https://wiki.ros.org/Master"). In our case, we will choose the RPi as the master. 

On your publishing computer, export the following environment variables. Make sure to replace `pub_ip_address` with the 
IP address of your publishing computer and `master_ip_address` with the IP address of the RPi: 

```
$ export ROS_IP=pub_ip_address
$ export ROS_MASTER_URI=http://master_ip_address:11311
```

On your RPi, export the following environment variables. Make sure to replace both instances of `master_ip_address` with 
the IP address of your RPi:

```
$ export ROS_IP=master_ip_address
$ export ROS_MASTER_URI=http://master_ip_address:11311
```

For the pose topic to be understood properly between computers, it's important that both machines are in sync. On both 
systems, run: 

```
$ timedatectl
```

In the output, you should see: 

```
System clock synchronized: yes
NTP service: active
```

If the output indicates that the NTP service isn't active, turn it on via: 

```
$ sudo timedatectl set-ntp on
```

After this, run `timedatectl` again to confirm the network time status. 

If the output indicates that the NTP service is active but not synchronized, it is possible that your firewall is 
blocking access to an NTP server. We can check if this is the case via: 

```

```

# TODO: what does the timeout output look like?

With both systems in sync, we can start publishing pose topics. 

First, kill any pre-existing ROS nodes via:

```
$ rosnode kill -a
```

# TODO: check this command ^^^^

Now start publishing. On your publishing computer, enter: 

```
$ roslaunch mocap_qualisys qualisys.launch  server_address:=qtm_server_address
```

Where `qtm_server_address` is the IP address to your QTM server.

To verify your pose topics are being published, run: 

```
$ rostopic echo /qualisys/rigid_body_name/pose
```

Where `rigid_body_name` is the name of your rigid body in QTM.

You can now start our motion capture control node on our RPi via: 

```
$ roslaunch mavrospy mocap.launch fcu_url:=/dev/serial0:921600
```

We can verify that our RPi is receiving our pose topic via: 

```
$ rostopic echo /mavros/vision_pose/pose
```

You can now switch to OFFBOARD mode and watch it fly!

### Set Up Environment Variables on Boot

Every time you want to launch the motion capture control node on a new boot, you will have to set up your environment 
variables each time. This can be a little tedious. 

Similar to before, we can utilize our system services. Edit our file auto launch file from before.  

```
$ sudo nano /usr/sbin/mavrospy_auto_launch
```

Delete the existing text and add the following, changing both occurrences of `your_username` to your RPi's username, 
`pub_ip_address` is your publishing computer's IP address and `master_ip_address` is your RPi's IP address:

```
#!/bin/bash
source $(echo ~your_username)/catkin_ws/devel/setup.bash

source /etc/ros/env.sh

export ROS_HOME=$(echo ~your_username)/.ros

export ROS_IP=pub_ip_address
export ROS_MASTER_URI=http://master_ip_address:11311

exit 125
```

# TODO: test system service

Restart the RPi. 

Now upon boot, you only need to set up environment variables for the publishing computer. This is ok because you will 
most likely be running other programs on your publishing computer in the future. Creating a system service could 
interfere with this. 

# UPDATED RPI DOC

TODO: integrate into old documentation

This repository is supported on ROS Noetic. If you haven't already, please install [Ubuntu MATE 22.04](https://releases.ubuntu-mate.org/22.04/arm64/) for the RPi4 (arm64) using the [Raspberry Pi Imager](https://www.raspberrypi.com/software/). 

Select the `Log in automatically` option upon first boot.

........

## Development Environment

Setting up a development environment on your desktop machine is helpful for running simulations before testing it on your live UAV. We can easily set up our environment via Docker.

This tutorial will assume you are running on a Linux OS, but Docker can be used on both Mac and Windows via [Docker Desktop](https://docs.docker.com/get-docker/).

First, make sure you installed [Docker Engine](https://docs.docker.com/engine/install/). It is also recommend you follow the [Linux Post-Installation Steps](https://docs.docker.com/engine/install/linux-postinstall/) as well to manage Docker as a non-root user.

We can verify our Docker installation via:

```
docker run hello-world
```

### NVIDIA GPU Acceleration

We can use [Gazebo Classic](https://classic.gazebosim.org/) to simulate our MAVROSPY movment commands. Using an NVIDIA graphics card will signicantly improve simulation performance, but is by no means required.

To use our NVIDIA GPU, verify you have installed its driver:

```
$ nvidia-smi
```

If it's not installed, follow your desktops OS driver installation instructions.

To utilize our NVIDIA GPU inside a Docker container, we must install the NVIDIA Container Toolkit.

This is a summary of the official [installation instructions](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html).

Configure the production repository:

```
$ curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
```

Update the packages list from the repository:

```
$ sudo apt update
```

Install the NVIDIA Container Toolkit packages:

```
$ sudo apt install nvidia-container-toolkit
```

Configure the Docker container runtime by using the `nvidia-ctk` command:

```
$ sudo nvidia-ctk runtime configure --runtime=docker
```

Restart the Docker daemon:

```
$ sudo systemctl restart docker
```

Verify the installation by running a sample CUDA container:

```
$ sudo docker run --rm --runtime=nvidia --gpus all ubuntu nvidia-smi
```

Depending on your PC, you may have an internal graphics card.

To use your NVIDIA graphics card, enter this command:

```
$ sudo prime-select nvidia
```

Then, reboot.

### Build Docker Image

We can now build our Docker image via the following commands:

```
$ cd ~
$ git clone https://github.com/bandofpv/mavrospy.git
$ cd ~/mavrospy/docker
$ docker build -t mavrospy-dev .
```

### Run Docker Container

After building, we can open up our image in a Docker container:

```
$ bash run_docker.sh
```

After successfully opening the docker container, launch a simulation:

```
$ roslaunch mavrospy sim_square.launch fcu_url:=udp://:14540@127.0.0.1:14557
```

Gazebo should open up and display a quadcopter model.

In the container, wait for the messages to stop updating until you see the following line:

```
INFO  [commander] Ready for takeoff!
```

Press `Enter` to open the PX4 shell. You should see a `pxh>` before your cursor.

Enter the following command to change into offboard mode:

```
pxh> commander mode offboard
```

Go back to the Gazebo window and watch drone fly!

The system will automatically clean up after the drone lands.

Enter `exit` to close the Docker container.

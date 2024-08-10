# mavrospy
ROS node to interact with [MAVROS](https://wiki.ros.org/mavros) for basic UAV control.

This current repo is supported on a PX4 flight controller with a RPi4 companion computer running Ubuntu 22.04 MATE with ROS Noetic.

Documentation for motion capture utilizes the [Qualisys Track Manager]("https://www.qualisys.com/software/qualisys-track-manager/").

### Setup:
This repository is supported on ROS Noetic. If you haven't already, please install [Ubuntu MATE 22.04](https://releases.ubuntu-mate.org/22.04/arm64/) for the RPi4 (arm64) using the [Raspberry Pi Imager](https://www.raspberrypi.com/software/). 

Select the `Log in automatically` option upon first boot.

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

### Setting up SSH

SSH allows us to remotely issue commands to the RPi over Wi-Fi.

Install the `openssh-server` package on your RPi:

```
$ sudo apt update
$ sudo apt install openssh-server
```

The SSH service will start automatically. We can verify this by entering:

```
$ sudo systemctl status ssh
```

You should see `Active: active (running)` somewhere in the status report. **CTRL+C** to exit.

On your publishing computer, SSH to your RPi. Make sure you change `username` to your RPi's username and `ip_address` to your RPi's IP address:

```
$ ssh username@ip_address
```

Remember, you can always check your IP address via:

```
$ ifconfig
```

We can now issue commands to our RPi via SSH rather than having it hooked up to a monitor with a mouse and keyboard.

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

### Install Docker

Installing [Docker](https://www.docker.com/) on Ubuntu MATE is very easy and will allow us to build the ROS environment onto the RPi in just a few steps.

This is a summarized version of the offical [installation insructions](https://docs.docker.com/engine/install/ubuntu/#installation-methods).

Setup Docker's `apt` repsitory:

```
# Add Docker's official GPG key:
$ sudo apt update
$ sudo apt install ca-certificates curl
$ sudo install -m 0755 -d /etc/apt/keyrings
$ sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
$ sudo chmod a+r /etc/apt/keyrings/docker.asc

$ echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
$ sudo apt update
```

Install the Docker packages:

```
$ sudo apt install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

Create the `docker` group if not already created:

```
$ sudo groupadd docker
```

Add your user to the `docker` group:

```
$ sudo usermod -aG docker $USER
```

Log out and log back in so that your group membership is re-evaluated.

We can verify our Docker installation via:

```
$ docker run hello-world
```

### Build MAVROSPY Docker image

Now that Docker is installed properlly, we can build our Docker image and run it in a Docker container.

Clone the `mavrospy` repository:

```
$ cd ~
$ git clone https://github.com/bandofpv/mavrospy.git
```

Build via the `run_docker.sh` script:

```
$ cd ~/mavrospy/docker/rpi
$ bash run_docker.sh
```

The script will automatically start building the docker image and run it in a container.

### Outdoor Test

With the MAVROSPY environment setup on your RPi, we can run a simple test script.

# TODO: how to auto run docker... Create bash script to automate this feature

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

# TODO Add instructions for flight controller parameters!!!

### Publish Motion Capture Pose Topic

In order to publish ROS pose topics from our computer to the RPi, we must establish a [ROS Master]("https://wiki.ros.org/Master"). In our case, we will choose the RPi as the master.

On your publishing computer, export the `ROS_MASTER_URI` environment variable. Make sure to replace `master_ip_address` with the IP address of the RPi:

```
$ export ROS_MASTER_URI=http://master_ip_address:11311
```

For the pose topic to be understood properly between computers, it's important that both machines are in sync. On both systems, run:

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

With both systems in sync, we can start publishing pose topics.

First, kill any pre-existing ROS nodes via:

```
$ rosnode kill --all
```

### DO we need this if u don't have ros autstart script? ^^^^

Now start publishing. On your publishing computer, enter:

```
$ roslaunch mocap_qualisys qualisys.launchserver_address:=qtm_server_address
```

Where `qtm_server_address` is the IP address to your QTM server.

To verify your pose topics are being published, run:

```
$ rostopic echo /qualisys/rigid_body_name/pose
```

Where `rigid_body_name` is the name of your rigid body in QTM.

You can now start our motion capture control node on our RPi via:

```
$ roslaunch mavrospy mocap.launch
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

## Development Environment

Setting up a development environment on your desktop machine is helpful for running simulations before testing it on your live UAV. We can easily set up our environment via Docker.

This tutorial will assume you are running on a Linux OS, but Docker can be used on both Mac and Windows via [Docker Desktop](https://docs.docker.com/get-docker/).

First, make sure you installed [Docker Engine](https://docs.docker.com/engine/install/). It is also recommend you follow the [Linux Post-Installation Steps](https://docs.docker.com/engine/install/linux-postinstall/) as well to manage Docker as a non-root user.

We can verify our Docker installation via:

```
$ docker run hello-world
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

Depending on your PC, you may have an internal graphics card. To use your NVIDIA graphics card, enter this command:

```
$ sudo prime-select nvidia
```

Then, reboot.

### Build Docker Image

We can now build our Docker image via the following commands:

```
$ cd ~
$ git clone https://github.com/bandofpv/mavrospy.git
$ cd ~/mavrospy/docker/dev
$ bash run_docker.sh
```

This script will automatically build the Docker image and open it in a Docker container.

### Launch Gazebo Classic Simulator

We can now launch a simulation:

```
$ roslaunch mavrospy gazebo_sim.launch
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

Note: you can test out the movement patterns in the [scripts](https://github.com/bandofpv/mavrospy/tree/main/scripts) directory. Simply specify the `movement` argument.

Ex: `$ roslaunch mavrospy gazebo_sim.launch movement:=circle`

### Visualization with RViz

We can also visualize our simulation and motion capature poses with [RViz](http://wiki.ros.org/rviz)

After starting a simulation or motion caputure publisher, we can launch RViz.

Open a new termial and attach to the development environment container:

```
$ bash ~/mavrospy/docker/dev/run_docker.sh
```

Note: if you are using motion capture, you will have to export the `ROS_MASTER_URI` environment variable again

Launch RViz:

```
$ roslaunch mavrospy rviz.launch
```

ROS will automatically detect whether its a simulation or motion capture and begin plotting the drone's path.

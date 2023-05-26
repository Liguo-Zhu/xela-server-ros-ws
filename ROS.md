01/02/2023 11:48 os: ubuntu 20.04

# Part 1: Xela Server ROS

## step1: installation and requirements

- 1, check the **requirements**
  Python 2.7 (not recommended) or Python 3.4+ websocket-client package
  `pip install websocket-client`
- 2, create **catkin** space and download the code (**xela-server-ros**), while the code has some bugs.

```
https://github.com/mcsix/xela-server-ros.git
```

- 3, open the code and copy the **xela_server** directory into your catkin workspace **src** directory and build as usual. copy the **xela_server** directory into your catkin workspace src directory and build as usual
- 4, download the code (**u20**) from https://xela.lat-d5.com/
- `xela_server` app must be in the PATH variable, like inside the `xela_server` directory

## step2: run

1. `roslaunch xela_server service.launch`
2. `python3 example.py`

```
#!/usr/bin/env python

import rospy from xela_server.srv
import XelaSensorXYZ
import sys

rospy.init_node('use_service')

# wait the service to be advertised, otherwise the service use will fail
rospy.wait_for_service('xServXYZ')

# setup a local proxy for the service (we will ask for X,Y and Z data)
srv=rospy.ServiceProxy('xServXYZ', XelaSensorXYZ)

# use the service and send it a value.
# In this case, I am sending sensor: 1 and taxel: 3
service_example=srv(1, 3)

# print the result from the service
print(service_example)

# close the app
sys.exit(0)
```

# Part 2: XR2244 Sensor Use Manual: visualization

Inductions on how to use the Xela sensors

## step1: Hardware connection:

• Connect the sensor to the USB-CAN adapter
• Connect the extra USB to a port for powering the sensor
• Connect the USB-CAN adapter to the computer

## step2: Software Preparation

### 1, Install 4 dependencies

```
sudo pip3 install python-can
sudo pip3 install pyserial
sudo apt install net-tools
sudo apt-get install can-utils
```

### 2, download the software package `u20` from https://xela.lat-d5.com/

path: software-> Ubuntu-> Ubuntu 20.04

### 3, follow the XELA sensor server software manual

#### 3.1. check the USB port number, and then set up the serial CAN by running the 2 commands

#### 3.1.1, check the port number:

`ll /dev/ | grep ttyUSB`

#### 3.1.2, modify the port number

`sudo slcand -o -s8 -t hw -S 3000000 /dev/ttyUSB0`
`sudo ifconfig slcan0 up`

#### 3.1.3, Inside the file named u20 to run the below code

![Screenshot from 2023-05-25 16-53-46.png](:/4e721508c000499b8a9b36ff977f05ee)

- Run the configuration
  `sudo ./xela_conf -c slcan0`
  If the information is correct, type y and press ‘Enter’ to save the config file.
- Manually change the model from `XR1844`to `XR2244`
  `sudo vim /etc/xela/xServ.ini`
  or:
  `sudo gedit /etc/xela/xServ.ini`

```
[CAN]
bustype = socketcan
channel = slcan0
[viz]
max_offset = 200
max_size = 500
[debug]
sens_print = full
[sensor]
num_brd = 2
ctr_ver = 2
ctrl_id = 1
model = XR1844
channel = 0
[sensor2]
ctr_ver = 2
ctrl_id = 2
model = XR1844
channel = 0
```

#### 3.4.Run the server

    `sudo ./xela_server`

#### 3.5.Run the visualizer

    `sudo ./xela_viz`

#### note:

`vim to modify a file in terminal:`
i-->cirt+c-->:-->wq-->enter

```
header:
  seq: 2338
  stamp:
    secs: 1685022745
    nsecs: 121681690
  frame_id: ''
sensor: 2
model: "XR2244"
points:
  -
    taxels: 0
    point:
      x: 443.0
      y: 33277.0
      z: 40644.0
  -
```

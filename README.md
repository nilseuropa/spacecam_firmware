# SpaceCam Firmware

This is a [platformio](https://platformio.org/install/ide) project for the ESP8266 based spaceCam IR tracker sensor.

### Setting up for ROS
1. Compile and upload the firmware with `pio`, the environment is already set, just choose the right upload port.
2. Once, the device booted, press the user button to activate the configuration portal.
3. Connect to `spacecam` AP without authorization
4. 192.168.4.1 is the default address of the configuration web service, but it should pop-up automatically *(thanks to [tzapu](https://github.com/tzapu))*
5. Press Configure WiFi to setup all parameters:
  * SSID from the list
  * AP password
  * ROS MASTER IP
  * ROS serial server port ( default **11411** )
6. Press save and the device will reboot
7. Serial server is going to be active on the selected port and will advertise on the `/spacecam/trackpoints` topic
7. Run roscore service on the IP address you have provided for ROS master
8. To run serial server type `rosserial_python serial_node.py tcp` *(C++ version is unstable)*

#### Trouble shooting
There are three LEDs on the device itself:
1. WiFi Connected solid / blinks if in AP mode
2. Marker found solid / no light if found none
3. Socket connected solid / no light if not connected

After starting the rosserial over tcp it should look like this:
```bash
$ rosrun rosserial_python serial_node.py tcp
[INFO] [1543498964.266146]: ROS Serial Python Node
Fork_server is:  False
[INFO] [1543498964.274244]: Waiting for socket connections on port 11411
waiting for socket connection
[INFO] [1543498964.280053]: Established a socket connection from 192.168.0.155 on port 6179
[INFO] [1543498964.282473]: calling startSerialClient
[INFO] [1543498966.392099]: Requesting topics...
[INFO] [1543498966.466827]: Note: publish buffer size is 512 bytes
[INFO] [1543498966.470578]: Setup publisher on /spacecam/trackpoints [std_msgs/UInt16MultiArray]
```

Check topic frequency, if all is well it should be around 100Hz:
```bash
$ rostopic hz spacecam/trackpoints
subscribed to [/spacecam/trackpoints]
average rate: 99.997
    min: 0.002s max: 0.043s std dev: 0.00389s window: 95
```

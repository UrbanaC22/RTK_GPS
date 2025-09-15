# RTK_GPS
### We will be using ROS2 along with the RTCM data.
The setup for Jeton Xavier is similar to that of Raspberry PI and can be found [here](https://www.waveshare.com/wiki/LC29H(XX)_GPS/RTK_HAT).
Although, some modifications are required to get things working in Xavier.

## Dependencies
#### The dependencies required are listed below along with commands to install them (other gpsd-related dependencies listed in the link above are not required):
- pynmeagps
- nmea_msgs

```
pip3 install pynmeagps
sudo apt install ros-foxy-nmea-msgs
```
#### Skip the gpsd part entirely in the WaveashareWiki as we will use ROS2 instead.
#### While downloading the source code, if any error appears, use:
```
wget https://files.waveshare.com/wiki/LC29H\(XX\)-GPS-RTK-HAT/Lc29h_gps_rtk_hat_code.zip
```
Modify the obtained scripts to the one given in this repository.

### Give necessary permissions to the serial port
By default, the GPS data from the hat will be available on the port THSO.
Execute the following commands to give read and write permissions:
```
sudo chmod g+rw /dev/ttyTHS0
sudo usermod -aG dialout interplanetar
sudo reboot
```
If serial port can't still be accessed, disable the following service:
```
systemctl stop nvgetty
systemctl disable nvgetty
```
### ROS2 Workspace
Make a ROS2 workspace:
```
mkdir -p rtk_ros_ws/src
```

Create a package with the following dependencies:
```
ros2 pkg create --build-type ament_python rtk_pkg --dependencies rclpy std_msgs nmea_msgs
```
In package.xml, add:
```
<exec_depend>python3-pynmeagps</exec_depend>
<exec_depend>python3-pyserial</exec_depend>
```
In setup.py:
```
  entry_points={
        'console_scripts': [
            'nmea_node= rtk_pkg.main:main',
            'conv_coord=rtk_pkg.coordinate:main'
        ],
    },
```

Paste the modified python scripts inside the rtk_pkg folder (_init_.py directory).

### Additional dependency:
Clone the following into the same directory as the python script as stated above:
```
git clone https://github.com/wondergis/coordTransform.git
```
### Build the workspace
```
colcon build
source install/setup.bash
```
### Run the nodes with the necessary attributed
```
ros2 run rtk_pkg nmea_node -u interplanetar.troubleshoot@gmail.com -p 1234 rtk2go.com 2101 boson
```
In another terminal:
```
ros2 run rtk_pkg conv_coord
```







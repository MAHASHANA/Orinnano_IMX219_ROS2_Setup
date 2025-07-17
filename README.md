# Setting up Arducam IMX219 camera for jetpack 6.2

When updating or upgrading to JetPack 6.2, it is important to note that auto detection for the IMX219 (Raspberry Pi V2 Camera) is disabled by default. This means that users transitioning from older JetPack versions (such as 5.x or earlier 6.x releases) must manually configure the camera using Jetson-IO.

```bash
ls /dev/video*
```
if the above cmd shows '/dev/video*' : No such fiel or directory

If the camera is not detected, you will see no output or an error message indicating that no video devices are available. In this case, proceed with the following steps to enable the IMX219 camera.

Open a terminal on your Jetson device.
```bash
Run the following command:

sudo /opt/nvidia/jetson-io/jetson-io.py
```
Now you will see a new window where you can select the desired configuration 

As this setup is for CSI camera select jetson 24pin CSI connector and then press enter again press enter you will see multiple camera enabling option i am using single camera in  my steup so  i chose Camera IMX219-A

After selecting the appropriate camera option, choose Save and reboot to reconfigure pins.
The system will prompt you to reboot. If it does not, manually reboot using
```bash
sudo reboot
```
------------------------------------------------------------------------
# camera_publisher (Jetson + ROS 2 camera package)

This package captures video from an IMX219 CSI camera (Jetson Orin Nano) using GStreamer and publishes ROS 2 image and camera info topics.

## 📦 How to Build

```bash
git clone https://github.com/MAHASHANA/Orinnano_IMX219_ROS2_Setup
cd Orinnano_IMX219_ROS2_Setup
colcon build
source install/setup.bash
ros2 run camera_publisher camera_node
```

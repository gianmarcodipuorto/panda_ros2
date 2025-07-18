# `kinect_ros2`

## Interface

### Overview
Basic Kinect-v1 (for the Xbox 360) node, with IPC support, based on [libfreenect](https://github.com/OpenKinect/libfreenect).
For now, it only supports a single Kinect device. (If multiple devices present, the first one listed by the `freenect_num_devices` will be selected).

### Published topics
* `~image_raw` - RGB image(rgb8) ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))
* `~camera_info` - RGB camera_info ([sensor_msgs/CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html))
* `~depth/image_raw` - Depth camera image(mono16) ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))
* `~depth/camera_info` - Depth camera_info ([sensor_msgs/CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html))

## Instalation
### 1. Install libfreenect
The package was tested using a manual build from the [libfreenect](https://github.com/OpenKinect/libfreenect) github because the Kinect used had a firmware version that requires specific build flags. 

### 2. Copy the repo
Copy the repo to your workspace source folder.
~~~
cd ~/ws/src
git clone https://github.com/fadlio/kinect_ros2
~~~

### 3. Install any missing ROS packages
Use `rosdep` from the top directory of your workspace to install any missing ROS related dependency.
~~~
cd ~/ws
rosdep install --from-paths src --ignore-src -r -y
~~~

### 4. Build your workspace
From the top directory of your workspace, use `colcon` to build your packages.
~~~
cd ~/ws
colcon build
~~~

## Using this package
Test it by running one of the provided python launch scripts.
~~~
ros2 launch kinect_ros2 pointcloud.launch.py
~~~
Or
~~~
ros2 launch kinect_ros2 showimage.launch.py
~~~

## Devices tested
* Kinect Model 1473
* Kinect Model 1414 (Both Normal and [Lite Mod version](https://medium.com/robotics-weekends/how-to-turn-old-kinect-into-a-compact-usb-powered-rgbd-sensor-f23d58e10eb0))

## Possible issues
### kinect_ros2_node: error while loading shared libraries `libfreenect.so0`
If you get the error message `kinect_ros2_node: error while loading shared libraries: libfreenect.so.0: cannot open shared object file: No such file or directory`
- Double-check your libfreenect intallation first.
- Check if you can see the libfreenect.so.0 file present in `usr/local/lib` path.

If you can see the libfreenect.so.0 file located in lib, there are a few things you could try:

---

#### 1. Add lib to `LD_LIBRARY_PATH`
Temporarily add lib to the `LD_LIBRARY_PATH` environment variable:
```bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
```

To verify, run:
```bash
echo $LD_LIBRARY_PATH
```

Then, try running your node again:
```bash
ros2 launch kinect_ros2 pointcloud.launch.py
```

---

#### 2. Make the Change Permanent
To avoid setting `LD_LIBRARY_PATH` every time, add the export command to your `~/.bashrc` file:
```bash
echo 'export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc
```

---

#### 3. Update the Dynamic Linker Cache
Alternatively, you can add lib to the dynamic linker configuration:

1. Add lib to the linker configuration:
   ```bash
   echo "/usr/local/lib" | sudo tee /etc/ld.so.conf.d/libfreenect.conf
   ```

2. Update the linker cache:
   ```bash
   sudo ldconfig
   ```

3. Verify that the library is now found:
   ```bash
   ldd ~/ws/install/kinect_ros2/lib/kinect_ros2/kinect_ros2_node | grep libfreenect
   ```

---

#### 4. Rebuild and Run
After making these changes, rebuild your workspace and run the node again:
```bash
colcon build --packages-select kinect_ros2
source ~/ws/install/setup.bash
ros2 launch kinect_ros2 pointcloud.launch.py
```

This should resolve the issue.

# rtf_pyopencv_camera

This is a super simple pi camera node using python and opencv.

```bash
colcon build --packages-select rtf_pyopencv_camera
. install/setup.bash
ros2 run rtf_pyopencv_camera pycamera
```

```bash
ros2 param set <node_name> <param_name> <param_value>
ros2 param load <node_name> <parameter_file>
ros2 run <package_name> <executable_name> --ros-args --params-file <file_name>
```

```yaml
/rtf_camera:
  ros__parameters:
    camera_num: 2
    camera_size: 720
```

- `pycamera`
    - topics:
        - `image`: uncompressed raw image
        - `image_compressed`: jpeg compressed image, rectified if calibration info provided
    - CLI arguments
        - `camera_num`: default 0
    - parameters:
        - `camera_size`: image size [height, width], default 320
        - `encoding`: `bgr`, `rgb`, default `mono8`
        - `rectified`: use calibration data to rectify image before sending, default `False`

## Launch

```bash
$ ros2 launch rtf_pyopencv_camera pycamera_foxglove_launch.py
```

## Alternatives

- C++: [OpenCV_Cam](https://github.com/christianrauch/opencv_cam)
- C++: [raspicam2_node](https://github.com/christianrauch/raspicam2_node)

## Reference Work

- [opencv_camera](https://github.com/MomsFriendlyRobotCompany/opencv_camera/tree/master)

## Foxglove

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

## Setup Pi Camera

I am using Ubuntu Server 24.04 on my Raspberry Pi.

- Add `start_x=1` to `/boot/firmware/config.txt`
    - *Note:* making the change to `/boot/firmware/usercfg.txt` doesn't work
- Reboot pi
- You should have `/dev/video0` now

Check all is working:

- `sudo apt install fswebcam`
- Run: `fswebcam --save test.jpg -d /dev/video0 -r 1280x960`
- You should now have a jpeg taken from the camera


# MIT License

**Copyright (c) 2020 Reckless Ted's Funland**

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

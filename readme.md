# rtf_pyopencv_camera

This is a super simple pi camera node using python and opencv.

## Alternatives

- C++: [OpenCV_Cam](https://github.com/christianrauch/opencv_cam)
- C++: [raspicam2_node](https://github.com/christianrauch/raspicam2_node)

## Setup Pi Camera

I am using Ubuntu Server 20.04 on my Raspberry Pi.

- Add `start_x=1` to `\boot\firmware\config.txt`
    - *Note:* making the change to `\boot\firmware\usercfg.txt` doesn't work
- Reboot pi
- You should have `/dev/video0` now

Check all is working:

- `sudo apt install fswebcam`
- Run: `fswebcam --save test.jpg -d /dev/video0 -r 1280x960`
- You should now have a jpeg taken from the camera

## Setup OpenCV in a Virtual Environment

I use a virtual environment and currently OpenCV 4.4 from pypi doesn't build
for the Raspberry Pi on Ubuntu 20.04. So what I do instead is:

- `sudo apt install python3-opencv`
    - Problem is, this is not my my virtual environment
- Now go inside your virtual env folder (for me it is `~/venv/lib/python3.8/site-packages`)
- Run: `ln -s /usr/lib/python3/dist-package/cv2.cpython-38-aarch64-linux-gnu.so  .`
    - Obviously, the library name will change over time, but this is the basic
    idea
- Now start python and try `import cv2` ... it should work without failure
    - Oh, make sure you installed `numpy` and any other requirements in your venv

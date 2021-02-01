# Azure_Kinect_python_Ubuntu16.04
이 레포지토리는 Azure Kinect 카메라의 cpp 함수를 shared object file을 통해 이용할 수 있게 만들어 줍니다.
다음의 다섯가지 데이터를 지원합니다.

### 1.Color Image
### 2.Depth Image
### 3.Color Image to Depth Geometry
### 4.Depth Image to Color Geometry
### 5.Dpeth Image to Color Geometry Pointcloud






# install
```bash
git clone  --recursive -b release/1.1.x https://github.com/microsoft/Azure-Kinect-Sensor-SDK.git
cd Azure-Kinect-Sensor-SDK.git
sudo dpkg --add-architecture amd64
sudo apt update
sudo apt install -y \
    pkg-config \
    ninja-build \
    doxygen \
    clang \
    gcc-multilib \
    g++-multilib \
    python3 \
    nasm

sudo apt install -y \
    libgl1-mesa-dev \
    libsoundio-dev \
    libvulkan-dev \
    libx11-dev \
    libxcursor-dev \
    libxinerama-dev \
    libxrandr-dev \
    libusb-1.0-0-dev \
    libssl-dev \
    libudev-dev \
    mesa-common-dev \
    uuid-dev
    
bash ./Azure-Kinect-Sensor-SDK/scripts/bootstrap-ubuntu.sh
sudo cp Azure-Kinect-Sensor-SDK/scripts/99-k4a.rules /etc/udev/rules.d/

git clone https://github.com/tjdalsckd/Azure_Kinect_python_Ubuntu16.04.git
mkdir -p Azure-Kinect-Sensor-SDK/examples/viewer_shared
cp -r  Azure_Kinect_python_Ubuntu16.04/* Azure-Kinect-Sensor-SDK/examples/viewer_shared/
gedit Azure-Kinect-Sensor-SDK/examples/CMakeLists.txt
```
CMakeLists.txt를 수정
```bash
# Copyright (c) Microsoft Corporation. All rights reserved.
# Licensed under the MIT License.

add_subdirectory(calibration)
add_subdirectory(enumerate)
add_subdirectory(playback_external_sync)
add_subdirectory(fastpointcloud)
add_subdirectory(opencv_compatibility)
add_subdirectory(streaming)
add_subdirectory(transformation)
add_subdirectory(undistort)
add_subdirectory(viewer)
add_subdirectory(viewer_shared)

```
빌드 진행
```bash
cd Azure-Kinect-Sensor-SDK
mkdir build
cd build
cmake ..
make -j16
```
빌드 후 생성된 shared object file을 python_example 폴더로 이동
```bash
cp build/bin/libviewer_opengl_shared.so Azure_Kinect_python_Ubuntu16.04/python_example/viewer_opengl.so
cd  Azure_Kinect_python_Ubuntu16.04/python_example
```
pointcloud를 보기 위해 open3d install

```bash
pip3 install open3d 
pip3 install open3d_azure_kinect_ubuntu1604_fix
python3 capture.py
```

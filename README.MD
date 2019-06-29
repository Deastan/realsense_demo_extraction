Opencv

WORKS:
mkdir build && cd build
make -j8
sudo make install

cmake -DENABLE_PRECOMPILED_HEADERS=OFF ..

cmake -D CMAKE_BUILD_TYPE=RELEASE \
            -D CMAKE_INSTALL_PREFIX=/usr/local \
            -D INSTALL_C_EXAMPLES=ON \
            -D INSTALL_PYTHON_EXAMPLES=ON \
            -D WITH_TBB=ON \
            -D WITH_V4L=ON \
            -D OPENCV_PYTHON3_INSTALL_PATH=$cwd/OpenCV-$cvVersion-py3/lib/python3.5/site-packages \
        -D WITH_QT=ON \
        -D WITH_OPENGL=ON \
        -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
        -D BUILD_EXAMPLES=ON ..

cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D INSTALL_PYTHON_EXAMPLES=ON \
    -D INSTALL_C_EXAMPLES=ON \
    -D OPENCV_ENABLE_NONFREE=ON \
    -D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib/modules \
    -D PYTHON_EXECUTABLE=~/home/jonathan/Documents/Git/venv_tf_gym/python \
    -D BUILD_EXAMPLES=ON ..

cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D INSTALL_PYTHON_EXAMPLES=ON \
    -D INSTALL_C_EXAMPLES=ON \
    -D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib/modules \
    -D BUILD_EXAMPLES=ON ..


opencv_contrib
cmake -DOPENCV_EXTRA_MODULES_PATH=/home/jonathan/Documents/Git/opencv_contrib/modules /home/jonathan/Documents/Git/opencv/ -DBUILD_EXAMPLES=ON

cmake -DENABLE_PRECOMPILED_HEADERS=OFF -DBUILD_EXAMPLES=ON -DOPENCV_EXTRA_MODULES_PATH=/home/jonathan/Documents/Git/opencv_contrib/modules /home/jonathan/Documents/Git/opencv/ 
cmake -DENABLE_PRECOMPILED_HEADERS=OFF -DBUILD_EXAMPLES=ON -DOPENCV_EXTRA_MODULES_PATH=/home/jonathan/Documents/Git/opencv_contrib/modules ..
cmake -DENABLE_PRECOMPILED_HEADERS=OFF -DCMAKE_INSTALL_PREFIX=/usr/local -DBUILD_EXAMPLES=ON -DOPENCV_EXTRA_MODULES_PATH=/home/jonathan/Documents/Git/opencv_contrib/modules ..


CAN BE USEFUL
﻿OpenCV

Prerequisites
sudo apt install build-essential
sudo apt install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
sudo apt install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev

git clone https://github.com/jasperproject/jasper-client.git jasper
chmod +x jasper/jasper.py
pip install --upgrade setuptools
pip install -r jasper/client/requirements.txt
sudo apt install libjasper-dev

sudo apt install pylint libatlas-base-dev libblas-dev liblapack-dev
sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gtk+3.0
sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libavresample-dev libeigen3-dev libeigen3-doc doxygen graphviz
sudo apt install libgphoto2-dev libhdf5-dev x264 v4l-utils libgtk-3-dev qt5-default


Using Anaconda already and wanted to eliminate any personal peeve:
conda config --set auto_activate_base false
Source
git clone https://github.com/opencv/opencv.git
git clone https://github.com/opencv/opencv_contrib.git
Build
cd opencv
mkdir build
cd build

Either
cmake -D CMAKE_BUILD_TYPE=RELEASE \
	-D CMAKE_INSTALL_PREFIX=/usr/local \
	-D INSTALL_C_EXAMPLES=ON \
	-D INSTALL_PYTHON_EXAMPLES=ON \
	-D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
      -D PYTHON_EXECUTABLE=/usr/bin/python3 \
	-D BUILD_EXAMPLES=ON ..

or
cmake-gui

Then

Either
make

or
nproc
# use response from above command to replace the number 8 below
make -j8
# where 8 specifies the number of processors for the build
TBD
Install
sudo make install

Links
sudo sh -c 'echo "/usr/local/lib" >> /etc/ld.so.conf.d/opencv.conf'
sudo ldconfig
find /usr/local/lib/ -type f -name "cv2*.so"
# you may have to replace 3.7 below for your specific installation
cd /usr/local/lib/python3.7/dist-packages/
sudo mv cv2.cpython-36m-x86_64-linux-gnu.so cv2.so
Validation
TBD
References
CMake

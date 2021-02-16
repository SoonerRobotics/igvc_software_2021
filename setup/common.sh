#!/bin/bash

## update sources
sudo apt-get update

## udev

# copy igvc.rules to rule.d
cp etc/igvc.rules /etc/udev/rules.d/igvc.rules
cp etc/51-kinect.rules /etc/udev/rules.d/51-kinect.rules

# restart udev
service udev reload
sleep 2
service udev restart

## vcs
# Install pip
sudo apt-get install python3-pip git -y

# Install vcstool
sudo pip3 install vcstool

# Install dependencies
vcs import < igvc.deps

# Install library dependencies
sudo apt-get install libspatialindex-dev -y

# Install python dependencies
pip3 install -r requirements.txt

## Kinect deps

# build lib
cd ../igvc_ws/src/deps/libfreenect
mkdir build
cd build
cmake -L ..
make
sudo make install
sudo ldconfig /usr/local/lib64/

# install deps
cd ../wrappers/python
sudo apt-get install python3-dev build-essential python3-numpy -y
sudo python3 setup.py install

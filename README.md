# igvc_software_2021

[![IGVC Weekly Compile Test](https://github.com/SoonerRobotics/igvc_software_2021/actions/workflows/igvc_compile_test.yml/badge.svg)](https://github.com/SoonerRobotics/igvc_software_2021/actions/workflows/igvc_compile_test.yml)

Software for the 2021 [Intelligent Ground Vehicle Competition](https://www.igvc.org/), Auto-Nav challenge.

We are using [ROS Noetic](http://wiki.ros.org/noetic) and Ubuntu 20.04 for this project.

# Dev setup

Run `sudo ./dev_setup.sh` from within the `setup` directory the first time you clone this repository to install the needed dependencies, install udev rules, and more automatically. This will also provide a few useful commands to make ROS easier.

# Robot setup

Run `sudo ./robot_setup.sh` from within the `setup` directory. This will install udev rules, init.d rules, dependencies, and more.

## Running roslaunch on boot

As part of the `robot_setup.sh` script, a systemd service named `igvc` will be created. This service is defined in `setup/etc/igvc` in this repo. By default, the setup script will not enable the service to launch on boot. To turn this on, run `systemctl enable igvc`. Similarly, you can run `systemctl disable igvc` to disable it.

# Updating dependencies

To update the dependencies in this repo, modify the `setup/igvc.deps` file, and then run `vcs import < igvc.deps` from the setup folder.
FROM osrf/ros:melodic-desktop-full

# Make the IGVC workspace directory
RUN mkdir -p /igvc/igvc_ws/src
RUN mkdir -p /igvc/setup

# Copy the src and setup folders into the Docker igvc workspace
COPY igvc_ws/src /igvc/igvc_ws/src
COPY setup /igvc/setup

# Build dependencies
WORKDIR /igvc/setup
RUN /bin/bash -c './common.sh'

# Navigate into the workspace and make
WORKDIR /igvc/igvc_ws
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; catkin_make'

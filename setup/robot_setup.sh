#!/bin/bash

bash common.sh

# copy igvc to /etc/init.d/
cp etc/igvc /etc/init.d/igvc

# add to systemctl
update-rc.d igvc defaults

# run the following to run code on startup
#sstemctl enable igvc
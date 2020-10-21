#!/bin/bash

bash common.sh

# copy igvc to /etc/init.d/
cp etc/igvc /etc/init.d/igvc

# add to systemctl
update-rc.d igvc defaults
update-rc.d igvc enable
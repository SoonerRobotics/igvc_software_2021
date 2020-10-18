#!/bin/bash

bash common.sh

echo ""
read -p "Would you like to add the custom IGVC ROS aliases to bashrc [Y\n]? " yn
case $yn in
    [Nn]* ) exit;;
esac

# add IGVC_LOC to .bashrc and igvc_dev to PATH
echo "" >> ~/.bashrc
echo "PATH=\"$PWD/igvc_dev:\$PATH\"" >> ~/.bashrc
cd ..
echo "export IGVC_LOC=$PWD" >> ~/.bashrc

echo "Type 'source ~/.bashrc' to source the new changes."
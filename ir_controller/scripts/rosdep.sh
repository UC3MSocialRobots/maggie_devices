#!/bin/sh
# External, ROS and system package dependencies

# install headers and lib
cd ../third_parties/IRTrans_shlib
sudo sh install.sh
cd ..

echo "Installed. Now execute config_driver.sh script"

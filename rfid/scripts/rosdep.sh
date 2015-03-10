#!/bin/bash

clear
cd /tmp

svn co https://163.117.150.59/repoAD/catkin_projects/devices/rfid/third_parties rfid_third_parties

echo "INSTALLING rfid DRIVER..."
echo
cd rfid_third_parties/ID_ISC.SDK.Linux_V04.06.10
sudo bash install.bash

cd ..
rm -rf rfid_third_parties

echo "FINISH!"
echo

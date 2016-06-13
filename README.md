# neoiv_ros_driver
ROS Driver for neoIV RED device, based on:

  - http://students.asl.ethz.ch/upl_pdf/151-report.pdf
  - https://github.com/intrepidcs/icsneoapi.git

Publishes CAN Bus Data in a custom msg.

# prerequisites & installation

````
sudo apt-get install libudev-dev

# install libconfuse
cd ~/catkin_ws/src/
git clone https://github.com/martinh/libconfuse
cd libconfuse/
./autogen.sh
sh ./configure
make
sudo make install

# install libftdi
sudo apt-get install libusb-1.0-0-dev
sudo apt-get install libconfuse-dev
sudo apt-get install swig python-dev
sudo apt-get install libboost-all-dev
cd ~/catkin_ws/src/
mkdir libftdi
cd libftdi
git clone git://developer.intra2net.com/libftdi
cd libftdi
mkdir build
cd build
cmake  -DCMAKE_INSTALL_PREFIX="/usr" ../
make
sudo make install
sudo apt-get build-dep lirc
````

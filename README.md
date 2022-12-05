# AstroidDriverC
Astroid Driver C is an INDI driver to communicate with a telescope mount based on arduino.

Dependencies:
```
sudo apt install build-essential devscripts debhelper fakeroot cdbs software-properties-common cmake
sudo add-apt-repository ppa:mutlaqja/ppa
sudo apt install libindi-dev libnova-dev libz-dev libgsl-dev
```
Installation:
```
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr -DCMAKE_BUILD_TYPE=Debug ../
make
sudo make install
```

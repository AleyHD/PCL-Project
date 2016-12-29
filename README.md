#PCL-Project  
  
**Compile Instructions**  
Tested on Ubuntu 14.04  
  
**Update Sources**  
sudo apt-get update  
  
**Install Qt**  
cd ~/Downloads  
wget http://download.qt.io/official_releases/qt/5.7/5.7.0/qt-opensource-linux-x64-5.7.0.run  
chmod +x qt-opensource-linux-x64-5.7.0.run  
sudo ./qt-opensource-linux-x64-5.7.0.run  
sudo apt-get install gdb  
  
**Install Dependencies**  
sudo apt-get install build-essential  
sudo apt-get install libgl1-mesa-dev mesa-utils libglew-dev libsdl2-dev libsdl2-image-dev libglm-dev libfreetype6-dev  
sudo apt-get install libboost-all-dev  
sudo apt-get install libeigen3-doc libeigen3-dev libflann-dev  
sudo apt-get install libqhull-dev libqhull-doc  
sudo apt-get install qt4-dev-tools  
sudo apt-get install libvtk6 libvtk6-dev  
  
**Install GIT & CMake**  
sudo apt-get install git cmake  
  
**Install PCL**  
cd ~/Downloads  
git clone https://github.com/PointCloudLibrary/pcl.git  
cd pcl  
mkdir release && cd release  
cmake -DCMAKE_BUILD_TYPE=None -DBUILD_GPU=ON -DBUILD_apps=ON -DBUILD_examples=ON -DCMAKE_INSTALL_PREFIX=/usr ..  
make  
sudo make install  


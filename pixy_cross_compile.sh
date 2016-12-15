ROBORIO_SYSROOT=/usr/arm-frc-linux-gnueabi
BUILD_DIR=$HOME/build
INSTALL_DIR=$HOME/build/frc_pixy
export PATH=$PATH:/usr/bin
 
mkdir -p $INSTALL_DIR/include
mkdir -p $INSTALL_DIR/lib
cd $BUILD_DIR
 
# Get arm-frc-linux toolchain
sudo apt-add-repository ppa:wpilib/toolchain --yes
sudo apt update
sudo apt install frc-toolchain --yes
 
# Get Boost
wget https://sourceforge.net/projects/boost/files/boost/1.62.0/boost_1_62_0.tar.bz2
tar xjf boost_1_62_0.tar.bz2
 
# Get libusb
wget http://sourceforge.net/projects/libusb/files/libusb-1.0/libusb-1.0.9/libusb-1.0.9.tar.bz2
tar xjf libusb-1.0.9.tar.bz2
 
# Get Pixy code
git clone https://github.com/charmedlabs/pixy.git
 
# Build Boost
echo "using gcc : arm : arm-frc-linux-gnueabi-g++ ;" > $HOME/user-config.jam
cd boost_1_62_0/
./bootstrap.sh --prefix=/home/vagrant/boost_install
./b2 --with-chrono --with-thread --with-system link=static toolset=gcc-arm
cd $BUILD_DIR
 
# Build libusb
cd libusb-1.0.9
./configure --host=arm-frc-linux-gnueabi
make
cd $BUILD_DIR
 
echo " SET(CMAKE_SYSTEM_NAME Linux)
SET(BOOST_ROOT /home/vagrant/build/boost_1_62_0)
SET(LIBUSB_1_INCLUDE_DIR /home/vagrant/build/libusb-1.0.9/libusb)
SET(Boost_INCLUDE_DIR  /home/vagrant/build/boost_1_62_0)
set(Boost_LIBRARY_DIR  /home/vagrant/build/boost_1_62_0/stage/lib)
SET(CMAKE_C_COMPILER   /usr/bin/arm-frc-linux-gnueabi-gcc)
SET(CMAKE_CXX_COMPILER /usr/bin/arm-frc-linux-gnueabi-g++)
SET(CMAKE_FIND_ROOT_PATH /usr/arm-frc-linux-gnueabi)
SET(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
SET(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
SET(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)" > $BUILD_DIR/toolchain-arm-frc-linux.cmake
 
# Build libusbpixy
cd pixy/scripts
bash build_libpixyusb.sh -DCMAKE_TOOLCHAIN_FILE=$BUILD_DIR/toolchain-arm-frc-linux.cmake
 
# Copy files
cp $BUILD_DIR/boost_1_62_0/stage/lib/libboost_*.a $INSTALL_DIR/lib
cp $BUILD_DIR/libusb-1.0.9/libusb/.libs/libusb-1.0.a $INSTALL_DIR/lib
cp $BUILD_DIR/pixy/build/libpixyusb/libpixyusb.a $INSTALL_DIR/lib
 
cp $BUILD_DIR/libusb-1.0.9/libusb/libusb.h $INSTALL_DIR/include
cp $BUILD_DIR/pixy/src/common/inc/pixydefs.h $INSTALL_DIR/include
cp $BUILD_DIR/pixy/src/host/libpixyusb/include/pixy.h $INSTALL_DIR/include

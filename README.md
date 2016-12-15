# Vision-2017

## Background
We experimented the the [Pixy CMUCam5](http://cmucam.org/projects/cmucam5/wiki) during our FRC 2017 preseason meetings and were quite impressed with its capabilities. Once we trained the Pixy to track a target, it seemed a simple matter to read the pan/tilt angles, target position, and target dimensions and then control the robot based on those.

Although the Pixy has a USB interface, it is not a USB Webcam -- you can't just plug it into the USB port and make it work. The Pixy can be controlled via I2C, SPI, and RS-232 (all available on the RoboRio), but we had problems getting those interfaces to work correctly.

## Porting to the RoboRio
The [PixyMon](http://cmucam.org/projects/cmucam5/wiki/Install_PixyMon_on_Mac) application (used to configure the Pixy) communicates with the Pixy via a USB port of the host computer (PC, Mac, Linux), so it should be possible to port [libpixyusb](https://github.com/charmedlabs/pixy) to the RoboRio.

[libpixyusb](https://github.com/charmedlabs/pixy) depends on some [Boost](http://www.boost.org) libraries and [libusb](http://www.libusb.org), so all of these needed to be built using the [arm-frc-linux toolchain](http://first.wpi.edu/FRC/roborio/toolchains). The necessary header files and cross-compiled libraries are included in the Eclipse project.

## PixyTracker
The PixyTracker class is an example implementation for controlling the Pixy and tracking one target [based on the [Pan/Tilt demo code](https://github.com/charmedlabs/pixy/tree/master/src/host/pantilt_in_c).

## Cross Compiling libpixyusb
No code modifications were necessary, is was just a matter of cross-compiling Boost, libusb, and libpixyusb. For the sake of consistency, I used Vagrant to create a Virtual Machine to perform the build. If you want to build these libraries yourself, these are the steps:

1. Install Vagrant (https://www.vagrantup.com)
2. Download the Vagrantfile and pixy_cross_compile.sh files
3. Start the vagrant machine and build the code!

   ```console
   $ vagrant up
   $ vagrant ssh
   $ sudo bash pixy_cross_compile.sh
   $ exit
   ```
4. The libraries and header files will be in the __frc_pixy__ directory.

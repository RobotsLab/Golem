# Golem

Robot control, planning and learning framework.

Overview
--------

Golem packages: ![Golem packages](/docs/packages+dependencies.png)

<table>
  <tr>
    <th>Name</th><th>Description</th><th>CMake options</th>
  </tr>
  <tr>
    <td>Expat</td><td>Mandatory</td><td></td>
  </tr>
  <tr>
    <td>Freeglut</td><td>Mandatory</td><td></td>
  </tr>
  <tr>
    <td>NVIDIA PhysX 2.8</td><td>Optional physics simulation</td><td>BUILD_GOLEM_PHYS</td>
  </tr>
  <tr>
    <td>ZeroC Ice</td><td>Optional Tiny Interface</td><td>BUILD_GOLEM_TINY_ICE</td>
  </tr>
  <tr>
    <td>Boost</td><td>Optional (advanced) Neuronics Katana 450 driver</td><td>BUILD_GOLEM_CTRL_KATANA450</td>
  </tr>
  <tr>
    <td>PCAN driver</td><td>Optional (advanced) Neuronics Katana 450 driver</td><td>BUILD_GOLEM_CTRL_KATANA450</td>
  </tr>
</table>


Ubuntu installation
-------------------

This is an example installation on Ubuntu 14.04 LTS.

Download.

* `mkdir Golem`
* `git clone https://github.com/RobotsLab/Golem Golem`

Install mandatory dependencies.

* Update database `sudo apt-get update`
* `sudo apt-get install cmake cmake-qt-gui build-essential libpopt-dev zlib1g-dev`
* `sudo apt-get install libexpat-dev freeglut3-dev`

Install optional dependencies.

* NVIDIA Physx SDK 2.8.* . Download and install from [here](https://www.dropbox.com/sh/2o9e4sgt6xp0e5c/AACEpqgBnc8HozLZN6oXC-Fka?dl=0): unpack `*.deb` files and run `sudo dpkg -i libphysx*.deb`.
* Tiny Interface (using ZeroC Ice). Run `sudo apt-get install ice35-translators libzeroc-ice35-dev`
* Neuronics Katana 450 driver. Boost libraries: `sudo apt-get install libboost-system-dev libboost-thread-dev`. Download and install PCAN drivers from [PEAK Systems website](http://www.peak-system.com/fileadmin/media/linux/index.htm#download).

Configure.

* `cd Golem`
* `mkdir BUILD`
* `cd BUILD`
* `cmake-gui ..`
* Press **Configure** and then choose your favourite target (default: Unix makefiles).
* Set **Advanced** to enable editing advanced options.
* If required, specify a temporary location of files during build. Edit **ARCHIVE_OUTPUT_DIRECTORY**, **LIBRARY_OUTPUT_DIRECTORY** and **RUNTIME_OUTPUT_DIRECTORY**.
* Press **Configure** and then **Generate**.

Build.

* `make`

Install (optional).

* `sudo make install`

Notes

* Make sure that the system is informed about locations of libraries, which are controlled by system variable `LD_LIBRARY_PATH`. For example if **CMAKE_INSTALL_PREFIX** is set to `/usr`, then `LD_LIBRARY_PATH` must contain `/usr/bin`. Open bash config `gedit ~/.bashrc`, add at the end of the file `export LD_LIBRARY_PATH=/usr/bin:${LD_LIBRARY_PATH}`, then update `source ~/.bashrc`.
* The default xml configuration files of Golem applications assume that they are all in the same working _application directory_ together with shared Golem libraries and corresponding xml files. By default it is a temporary location of files during build (see Ubuntu installation -> Configure), as well as the directory for executables pointed by **CMAKE_INSTALL_PREFIX** (typically `/usr/bin`).
* In some hardware and software configurations (e.g. in virtual machines) direct OpenGL rendering may need to be disabled. Open bash config `gedit ~/.bashrc`, add at the end of the file `export LIBGL_ALWAYS_INDIRECT=yes`, then update `source ~/.bashrc`.
* When operating real robots, all applications should be launched as `root` to allow runnig the robot drivers' threads in a real-time priority mode.

# Golem

Robot control, planning and learning framework.

Overview
--------

Golem packages: ![Golem packages](/docs/packages+dependencies.png)

Golem major dependencies:

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
    <td>Boost >= 1.58</td><td>Mandatory</td><td></td>
  </tr>
  <tr>
    <td>OpenCV 2.4.X</td><td>Mandatory</td><td></td>
  </tr>
  <tr>
    <td>PCL >= 1.7</td><td>Mandatory</td><td></td>
  </tr>
  <tr>
    <td>NVIDIA PhysX 2.8.X</td><td>Optional physics simulation</td><td>GOLEM_BUILD_\*_PHYS</td>
  </tr>
  <tr>
    <td>ZeroC Ice</td><td>Optional Tiny Interface</td><td>GOLEM_BUILD_\*_PTINY</td>
  </tr>
</table>


Ubuntu installation
------------------

This is an example installation on Ubuntu 16.04.06 LTS.

Update package database.

* `sudo apt-get update`

Install essential c++ development packages.

* `sudo apt-get install build-essential cmake git`

Install mandatory dependencies.

* `sudo apt-get install libpopt-dev zlib1g-dev libxmu-dev libxi-dev libproj-dev libboost-dev libexpat-dev freeglut3-dev libpcl-dev libopencv-dev`

Install optional dependencies (Optional).

* NVIDIA Physx SDK 2.8.* . Download and install PhysX from [NVIDIA site](https://www.nvidia.co.uk/object/physx_archives.html): unpack `*.deb` files and run `sudo dpkg -i libphysx*.deb`.
* Tiny Interface (using ZeroC Ice). Run `sudo apt-get install ice35-translators libzeroc-ice35-dev`

Create project workspace.

* `mkdir Projects`
* `cd Projects`

Download Golem.

* `mkdir Golem`
* `git clone https://github.com/RobotsLab/Golem Golem`

Configure Golem (without optional dependencies).

* `cd Golem`
* `mkdir build`
* `cd build`
* `mkdir build && cd build && cmake build .. -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_INSTALL_PREFIX=/usr/local`

Build and install Golem.

* `sudo make install -j`

Notes

* Make sure that the system is informed about locations of libraries, which are controlled by system variable `LD_LIBRARY_PATH`. For example if **CMAKE_INSTALL_PREFIX** is set to `/usr/local`, then `LD_LIBRARY_PATH` must contain `/usr/local/bin`. Open bash config `gedit ~/.bashrc`, add at the end of the file `export LD_LIBRARY_PATH=/usr/local/bin:${LD_LIBRARY_PATH}`, then update `source ~/.bashrc`.
* The default xml configuration files of Golem applications assume that they are all in the same working _application directory_ together with shared Golem libraries and corresponding xml files. By default it is a temporary location of files during build (see Ubuntu installation -> Configure), as well as the directory for executables pointed by **CMAKE_INSTALL_PREFIX** (typically `/usr/local/bin`).
* In some hardware and software configurations (e.g. in virtual machines) direct OpenGL rendering may need to be disabled. Open bash config `gedit ~/.bashrc`, add at the end of the file `export LIBGL_ALWAYS_INDIRECT=yes`, then update `source ~/.bashrc`.
* When operating real robots, all applications should be launched as `root` to allow runnig the robot drivers' threads in a real-time priority mode.
* Some Golem applications require write permissions to the currunt directory.


Example applications
-------------------

Update library search path

* `export LD_LIBRARY_PATH=/usr/local/bin:${LD_LIBRARY_PATH}`

Golem Grasp application.

* `GolemAppGrasp /usr/local/bin/GolemAppGrasp_RobotBoris.xml GolemAppGrasp.log stdout`

Golem Grasp Neural Network client.

* `GolemAppGrasp /usr/local/bin/GolemAppGrasp_RobotBorisNNLearning.xml GolemAppGrasp.log stdout`
* Press `R` then `G` in the OpenGL window to run the client.


Docker installation
------------------

We assume here that you have Docker Engine already installed on your system. There are several available tutorials on the Web, for example [Install Docker Engine on Ubuntu](https://docs.docker.com/engine/install/ubuntu/).

Copy all Golem docker files ![Golem docker](/docker/) to some local directory.

Create docker image `golem_test`.

* `./Dockerfile.sh golem_test`

Run Golem Grasp application for Birmingham Boris robot.

* `./run_Grasp.sh golem_test Boris`

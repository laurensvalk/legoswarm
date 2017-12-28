# Lego Swarm
Swarm robotics with LEGO: Multiple LEGO robot vehicles collaborating to harvest and sort LEGO balls spread out in a field.

# General Concept
- Decentralized Control: Each agent (Skid steering rover) has simple local rules for emerging "intelligent" behavior
  - Driving around while collecting any balls that are "accidentally" in its path
  - Control by virtual springs
    - Virtual springs with repulsive spring force between neighbors
    - Virtual springs with attractive spring force to valuable locations (see below)
    - The forces are based on information broadcast by other agents
  - Each robot is completely oblivious to what it is doing (it doesn't even detect the resource until actually finding it), yet is part of a useful network
- Agents communicate only with neighboring agents, broadcasting:
  - 1) Its approximate location
    - Repulsive forces prevent most collisions
  - 2) Its estimate of the local value/resources
    - Each time an agent finds a ball (by accident), it increases its localized value
    - The value slowly decays as time passes without finding any new balls
    - An agent in an resource rich area broadcasts a high value, thus attracting other agents through the virtual springs
  - Robots have an (artificially) limited line of sight.
    - Only close neighbors can communicate
    - Information can be re-broadcast to hop through each node (agent) in the network
- The swarm is robust
  - Continues operation when agents die or new agents are added.
    - Repulsive virtual forces ensure repositioning when the network changes
  - Adapts when the network is disturbed
    - When a robot is pushed away (e.g. visitor interaction, or a rogue robot controlled by the public)
    - When new resources are added the network dynamically approaches the new resources
- Network must also collaboratively explore. Randomly moving around while maintaining coverage of the area.

# About those virtual springs
- The virtual springs represent virtual forces between the robots.
- The robot turns its motors such that it moves as if real springs pulled/pushed on it.
- All forces (from neighboring agents) are added up
  - Nett force along positive heading controls the speed
  - Nett torque controls yaw speed
- The virtual springs are attached to a virtual point ahead of the wheel axles
  - This way, an attractive or repulsive force "drags" the robot in the right direction, thus automatically steering it
  - Spring then controls both heading and position (almost) without singularities  

# Implementation Aspects
- A way for each agent to know its approximate location and orientation
  - Heading:
    - Gyroscope
    - Compass
  - Positioning
    - Beacons "indoor GPS"
      - Simple least squares to merge redundant beacon data
    - Precise positioning is not necessary due to the simple move rules
    - Local QR code stickers on field
      - Could be a LEGO mosaic with unique patterns
      - So many 1x1 tiles....
- A way for communication between the agents, without central network
  - Bluetooth/IR?
  - Publicly broadcast location and resource values
- A simple robot that collects and stores LEGO balls. (And then build several)
  - Transport:
    - Skid/Tank steering
    - Omniwheel probably too expensive, and above steering is actually fun to implement
  - Collecting balls:
    - Somewhat like vacuum cleaner, collecting anything in its path without sensing
    - Color is detected upon collection
    - Stores the ball in order, into zamor shooter storage
    - That gives a FIFO array of colors
    - Can travel towards drop off point and then expel each one by one
    - Generation 2.0 could do sorting on board, for less visits to the dump site  
  - Returning to charge point would be nice


# Installation
## MacOS
This is how I installed it on Mac OS X. If you manage to install openCV on other systems, 
please document it here and do a pull request.

For Mac you need the brilliant [homebrew](https://brew.sh/) package installer. You might need XQuartz, but I'm not sure. 
It's on my system and I haven't tried uninstalling it.
```
brew reinstall opencv3 --with-python3 --c++11 --with-contrib --without-python --with-qt --with-tbb --with-opengl
brew install opencv3 --with-contrib --c++11 --with-python3 --without-python --with-qt --with-tbb --with-opengl
brew link --force opencv3
https://github.com/Polyconseil/zbarlight/
```


`--c++11` is a for modern compiler. Limited speed improvements.

`--with-qt` is a UI and window manager. Needed for openGL

`--with-tbb` is for multithreading. Speeds up the process a lot.

`--with-opengl` Speeds up the video preview a LOT! Code works without opengl, but at 8-10 fps... 

And then of course `git clone` this repository.

## Raspberry Pi
The position server probably runs from a Raspberry Pi 3 with Picamera too. Installation is a little harder.
Here's what you do:
1. Download a raspbian stretch lite image from here
2. Burn it with Etcher (or similar) to a microSD
3. In the boot directory add an empty file named 'ssh' to enable headless ssh installation
4. Boot the pi on a wired connection and log in to it with `ssh pi@raspberrypi.local` and password `raspberry`
5. `passwd` and choose a new one
6. `sudo raspi-config` and enable OpenGL, PiCamera and expand the filesystem (all under adavanced). 
Now might also be a good time to setup WiFi under Networking.
7. `sudo apt-get update`
8. `sudo apt-get upgrade`
9. `sudo apt-get install build-essential checkinstall cmake pkg-config yasm`
10. `sudo apt-get install git gfortran`
11. `sudo apt-get install libjpeg8-dev libjasper-dev libpng12-dev`
12. ?? `sudo apt-get install x264 v4l-utils`
12. `sudo apt-get install python3-numpy python3-pip libpython3-dev python3.5-dev`
13. `sudo pip3 install picamera[array]`
17. `git clone https://github.com/opencv/opencv_contrib.git`
18. `cd opencv_contrib/`
15. `git checkout 3.4.0`
16. `cd ..`
13. `git clone https://github.com/opencv/opencv.git`
14. `cd opencv`
15. `git checkout 3.4.0`
16. `mkdir build`
17. `cd build`
18. ```cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D CMAKE_INSTALL_PREFIX=/usr/local \
      -D WITH_PYTHON3=ON \
      -D WITH_EIGEN=ON \
      -D INSTALL_C_EXAMPLES=ON \
      -D INSTALL_PYTHON_EXAMPLES=ON \
      -D WITH_TBB=ON \
      -D WITH_V4L=ON \
      -D WITH_OPENGL=ON \
      -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
      -D BUILD_EXAMPLES=ON \
      -D PYTHON_EXECUTABLE=/usr/bin/python3 \
      -D PYTHON_DEFAULT_EXECUTABLE=/usr/bin/python3 ..```
2. `make -j4`
3. press ctrl-Z
4. `disown -h`
5. `bg`
19. Get some sleep. Do something else. This takes hours.



## EV3 brick with ev3dev
- Create an ev3dev micro SD card as documented on www.ev3dev.org

- On the brick you need numpy:
`apt-get install python-numpy`
... and a lot of patience.

- In some cases you need to install some more stuff before it works.
`apt-get install python-pip python-dev python-numpy`


# Running
- Print the templates and stick them on robots
- Plug in a webcam into your Mac/Pc and run the position_server_triangles.py
- Run the robot scripts on the robot and put the robots under the webcam
 
# Testing
The repository includes a bare server with dummy data and a bare client for your testing convenience.

# Selling the concept
- As in the real deal, all communication and intelligence is local to the agents. There is no global oversight.
- But to understand and visualize what is going on (for our debugging, and the public), here's an idea:
  - Camera top view
  - _Live, visual overlay of what's going on_
  - All agents actually do broadcast to a central hub
- Dots signifying agent positions (which upon overlay immediately reveal the estimation error)  
- Transparent circles signifying lines of sight
- Lines between any neighbors currently in communication: produces a "tree" with all kinds of branches. And some disconnected branches or nodes
- Vectors at each node representing each force (soft color) + nett force, torque (bold), indicating the current velocity and yaw rate
- Colored circle to represent resource quantity that is being broadcast
- Flashes each time a new ball is collected
- Bar plots of stats
  - Balls collected so far
  - Distance traveled, etc
  - Collisions
- Fast sped up video should make for some mesmerizing results (hopefully), revealing emergent behavior

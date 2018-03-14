# Lego Swarm
Swarm robotics with LEGO: Multiple LEGO robot vehicles collaborating to harvest and sort 
LEGO balls spread out in a field.

_If you are serious about replicating this, feel free to post an issue to share your interest 
and we may be able to help you get started. (And update this read me)._

There is also a [video of the result](https://www.youtube.com/watch?v=GEoh_08qnJM).


# Installation
You need a server computer with a webcam, a wifi network that connects robots and server and Ev3 robots 
running EV3dev. We used this version: 

## MacOS Server Installation
This is how I installed it on Mac OS X. If you manage to install openCV on other systems, 
please document it here and do a pull request.

For Mac you need the brilliant [homebrew](https://brew.sh/) package installer. You might need XQuartz, but I'm not sure. 
It's on my system and I haven't tried uninstalling it.
```
brew install opencv3 --with-python3 --c++11 --with-contrib --without-python --with-qt --with-tbb --with-opengl
brew link --force opencv3
```

`--c++11` is a for modern compiler. Limited speed improvements.

`--with-qt` is a UI and window manager. Needed for openGL

`--with-tbb` is for multithreading. Speeds up the process a lot.

`--with-opengl` Speeds up the video preview a LOT! Code works without opengl, but at 8-10 fps... 

And then of course `git clone` this repository.

## Windows server installation
I also managed to it on windows, instructions will follow. It was pretty easy.


# Running
- Print the templates and stick them on robots
- Plug in a webcam into your Mac/Pc and run the position server
- Run the agent main script on the robot and put the robots under the webcam
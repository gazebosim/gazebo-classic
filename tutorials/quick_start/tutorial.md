#Tutorial: Quick Start# 

### Run an empty world

First make sure Gazebo is [installed](http://gazebosim.org/#download).

Next, open a terminal.

Then start gazebo with an empty world, by entering the following at the command prompt.

~~~
gazebo
~~~

### Run an world with a robot

Let's simulate something a bit more interesting by loading a world with a pioneer2dx.

At a terminal prompt, enter the following command.

~~~
gazebo worlds/pioneer2d.world
~~~

### Where are the worlds located?

You may have noticed the mysterious `worlds/pioneer2dx.world` argument in the aobove command. This instructs gazebo to find the `pioneer2dx.world` file, and load it on start.

These world files are located in versioned system directory, such as `/usr/share/gazebo-3.0`. If you have Gazebo 3.0 install, in a terminal type the following to see a complete list of worlds.

~~~
ls /usr/share/gazebo-3.0/worlds
~~~

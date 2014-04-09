#Tutorial: Quick Start# 

### Run an empty world

1. [[install | Install Gazebo]].

2. Test out the server. This will start the physics engine with an empty world

~~~
gzserver
~~~

  Expected output:

  ~~~
  Gazebo multi-robot simulator, version 1.9.5
  Copyright (C) 2013 Open Source Robotics Foundation.
  Released under the Apache 2 License.
  http://gazebosim.org

  Msg Waiting for master
  Msg Connected to gazebo master @ http://127.0.0.1:11345
  Msg Publicized address: 192.168.xxx.yyy
  ~~~

  4. Test out the GUI client. This will connect to the server and give you a graphical display of the simulation.

  ~~~
  gzclient
  ~~~

    Expected output:

    ~~~
    Gazebo multi-robot simulator, version 1.9.5
    Copyright (C) 2013 Open Source Robotics Foundation.
    Released under the Apache 2 License.
    http://gazebosim.org

    Msg Waiting for master
    Msg Connected to gazebo master @ http://127.0.0.1:11345
    Msg Publicized address: 192.168.xxx.yyy
    ~~~


    5. Alternatively, you can launch the client and the server with a single command:

    ~~~
    gazebo
    ~~~


Embedded window example

This is very simple example of how to embed gzclient's window into another
application.

## Build Instructions

From this directory:

    mkdir build
    cd build
    cmake ..
    make

## Execute Instructions

* Launch gzserver as usual and place it on the background:

        gzserver &

    > If you start the window before, it will hang until there is a server

* Then from the build directory above, run the example client:

        ./embeded_window

* You'll see a gzclient window embedded into a custom window which has other
  widgets:

    ![Example](https://bytebucket.org/osrf/gazebo/raw/default/examples/stand_alone/embedded_window/example.png)

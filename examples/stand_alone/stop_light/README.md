This example changes lights on a `stop_light_post` model.

# Build

~~~
mkdir build
cd build
cmake ..
make
~~~

# Run

1. Start Gazebo from a terminal

        gazebo

1. Using the GUI, insert a `stop_light_post` model

1. In a new terminal, run this program

        ./stop_light stop_light_post red

1. The lights will turn red

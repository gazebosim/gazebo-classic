This example program will trigger a manual transporter.

# Build

~~~
mkdir build
cd build
cmake ../
make
~~~

# Run

1. Start Gazebo from a termnial
~~~
gazebo worlds/transporter.world
~~~
1. Using the GUI, place and object, such as a sphere, at the origin.
1. In a new terminal, run the transporter

~~~
./transporter
~~~

1. The object should move to a new location

# Model Movement Plugin

This plugin example demonstates moving a model to a specified set of
goal points.

The movement is not kinematic aware as it uses Pose Animation
described in this
[tutorial](http://gazebosim.org/tutorials?tut=animated_box).

## Build Instructions

From this directory

```
$ mkdir build
$ cd build
$ cmake ../
$ make
```

## Run Instructions

There are several ways to interact with the animation. Below you can find
how to run the standalone example, which defines the animation
programatically. It is also possible to define the animation parameters in
the SDF file.

### Run the standalone publisher

1. Go to the build directory and run gazebo with the example world

    ```
    $ cd build
    ```

    ```
    $ gazebo ../model_move.world
    ```

1. Publish the path publisher

    ```
    open a new terminal
    ```

    ```
    $ cd build
    ```

    ```
    $ ./path_publisher
    ```

### Use predefined animation from the SDF file

Go to the build directory and run gazebo with the example world

```
$ cd build
$ gazebo ../model_move_with_movement.world
```

To change the input parameters, you can modify the world file sdf:

```
<goals> container of the list of poses to be reached by the model
<pose> 6 floats (position, orientation) to be treated as goals
```

Example (in sdf file):

```
<plugin name="model_move" filename="libmodel_move.so">
  <goals>
    <pose>5 5 0 0 0 0</pose>
    <pose>5 -5 0 0 0 0</pose>
    <pose>0 0 0 0 0 0</pose>
  </goals>
</plugin>
```

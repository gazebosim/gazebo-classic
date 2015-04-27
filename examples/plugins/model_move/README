# Model Movement Plugin

This is a simple plugin example for moving a model to a specified set of goal points.

The displacement is not kinematic aware, it uses the Pose Animation as described in
the tutorial: http://gazebosim.org/tutorials?tut=animated_box 

## Build Instructions

* From this directory

```
$ mkdir build
$ cd build
$ cmake ../
$ make
```

## Run Instructions

There are several ways to interact with the animation. Below you can find how to run
the standalone example, which defines the animation programatically. It is also 
possible to create  

### Run then standalone publisher

* Go to the build directory and run gazebo with the example world
```
$ cd build
$ GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}/${PWD} gazebo ../model_move.world &
```

* Publish the path publisher

```
$ cd build
$ ./path_publisher
```

### Use predefined animation from the SDF file

* Go to the build directory and run gazebo with the example world
```
$ cd build
$ GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}/${PWD} gazebo ../model_move_with_movement.world &
```

To change the input parameters, you can modify the world file sdf:

    <path> is the sequence of goal point, as a string of space-separated integers, 
           six ints per point as per gazebo Pose.
    <n_points> is the number of specified goal points.

    Example (in sdf file):
    
    <plugin name="model_move" filename="libmodel_move.so">
        <path>5 5 0 0 0 0
              5 -5 0 0 0 0
              0 0 0 0 0 0</path>
	<n_points>3</n_points>
    </plugin>

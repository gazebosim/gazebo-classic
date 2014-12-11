# Installation

1. mkdir build

1. cd build

1. cmake ../

1. make

1. sudo make install

# Running the cafe world

1. gazebo worlds/cafe.world

# Modifying the actors

1. Edit <gazebo_sources>/worlds/cafe.world

1. Toward the bottom are two `<actor>` elements.

1. Modify the `<pose>` element to change the actor's starting location.

1. Modify the `<target>` element to change the actor's first target location.

1. The `<[obstacle/target]_weight>` are used to tune the vector field.

1. Each `<model>` under `<ignore_obstacles>` is ignored by the vector field algorithm. 

# Notes to the next developer

1. The gazebo/physics/Actor class has been brutally modified.

    1. The ability to <script> a motion (see worlds/actor.world) has been ripped out.

    1. For this work, we need the ability to control motion via a plugin (see plugins/Actor.*).

    1. Ideally we make the Actor class handle both scripting, and programmatic control

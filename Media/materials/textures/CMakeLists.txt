SET (files 
            box_create_blue.png
            box_create_grey.png
            cave.png
            ceiling_tiled.jpg
            clouds.jpg
            CMakeLists.txt
            control_end_blue.png
            control_end.png
            control_pause_blue.png
            control_pause.png
            control_play_blue.png
            control_play.png
            cursor.png
            cylinder_create_blue.png
            directionallight.png
            gazebo_icon.png
            gazebo_light_icons.svg
            grayGrid.png
            hand_cursor.png
            hanoi_blue_disk.tga
            hanoi_green_disk.tga
            hanoi_red_disk.tga
            hardwood_floor.jpg
            paintedWall.jpg
            pioneerBody.jpg
            pointlight.png
            sphere_create_blue.png
            sphere_create_grey.png
            spotlight.png
            terrain_detail.jpg
            terrain.png
            terrain_texture.jpg
            willowMap.png
            WoodPallet.png 
            test.png
            redSelect.png
            greenSelect.png
            blueSelect.png
)


INSTALL(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/Media/materials/textures/)


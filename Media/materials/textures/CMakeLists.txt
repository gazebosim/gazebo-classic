SET (files 
            ceiling_tiled.jpg
            clouds.jpg
            hardwood_floor.jpg
            paintedWall.jpg
            pioneerBody.jpg
            terrain_detail.jpg
            terrain.png
            willowMap.png
            WoodPallet.png 
            wood.jpg
            beigeWall.jpg
            granite.jpg
            granite2.jpg
            grass.jpg
            dirt_diffusespecular.png
            dirt_normal.png
            grass_diffusespecular.png
            grass_normal.png
            fungus_diffusespecular.png
            fungus_normal.png
            heightmap_bowl.png
            projection_filter.png
            stereo_projection_pattern_high_res.png
            stereo_projection_pattern_high_res_red.png
)


INSTALL(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/Media/materials/textures/)

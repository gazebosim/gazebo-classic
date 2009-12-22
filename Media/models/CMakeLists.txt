ADD_SUBDIRECTORY(bandit)
ADD_SUBDIRECTORY(blender)
ADD_SUBDIRECTORY(epuck)
ADD_SUBDIRECTORY(PioneerArm)
ADD_SUBDIRECTORY(pioneer2dx)
ADD_SUBDIRECTORY(pioneer2at)
ADD_SUBDIRECTORY(sonyvid30)
ADD_SUBDIRECTORY(rovio)

SET (files column.mesh
           geosphere4500.mesh
           geosphere8000.mesh
           kitchen.mesh
           knot.mesh
           razor.mesh
           robot.mesh
           sicklms200.mesh
           sphere.mesh
           unitpyramid.mesh
           wheel.mesh
           WoodPallet.mesh
           willowgarage.dae
           hanoi_red_disk.dae
           hanoi_green_disk.dae
           hanoi_blue_disk.dae
)

INSTALL(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo/Media/models)

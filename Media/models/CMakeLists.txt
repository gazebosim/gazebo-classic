add_subdirectory(bandit)
add_subdirectory(blender)
add_subdirectory(epuck)
add_subdirectory(PioneerArm)
add_subdirectory(pioneer2dx)
add_subdirectory(pioneer2at)
add_subdirectory(sonyvid30)
add_subdirectory(rovio)
add_subdirectory(pr2_gripper)
add_subdirectory(pr2)

set (files column.mesh
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
           cup.mesh
           test.dae
)

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/Media/models)

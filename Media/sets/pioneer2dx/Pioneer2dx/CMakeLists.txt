SET (files chassis.mesh
           chassis_top.mesh
           hub.mesh
           Scene.material
           sonarbank.mesh
           sonar.mesh
           tire.mesh
           wheel.mesh
)

INSTALL(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo/Media/sets/pioneer2dx/Pioneer2dx)

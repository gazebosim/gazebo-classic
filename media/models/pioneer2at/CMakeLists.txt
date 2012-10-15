SET (files chassis.mesh
           chassis_top.mesh
           Cylinder.012.mesh.xml
           hub.mesh
           Scene.material
           sonarbank.mesh
           sonar.mesh
           tire.mesh
           wheel.mesh
)

INSTALL(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/media/models/Pioneer2at)

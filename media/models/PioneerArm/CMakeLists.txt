SET (files Base.mesh
           Base.mesh.xml
           Gripper.mesh
           Gripper.mesh.xml
           Link01.mesh
           Link01.mesh.xml
           Link02.mesh
           Link02.mesh.xml
           Link03.mesh
           Link03.mesh.xml
           Link04.mesh
           Link04.mesh.xml
           Link05.mesh
           Link05.mesh.xml
           PioneerArm.blend
           Scene.material
)

INSTALL(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/media/models/PioneerArm)

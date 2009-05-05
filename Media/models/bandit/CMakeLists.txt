SET (files bicep.mesh
           body_model_1.mesh
           eyebrow.mesh
           head_1.mesh
           left_forearm.mesh
           left_hand.mesh
           left_shoulder.mesh
           lowerlip.mesh
           neck_1.mesh
           right_forearm.mesh
           right_hand.mesh
           right_shoulder.mesh
           shoulder_mounting_1.mesh
           thumb.mesh
           upperlip.mesh
           wrist.mesh
)

INSTALL(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo/Media/models/bandit)

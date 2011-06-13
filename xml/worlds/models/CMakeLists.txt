SET (files athene.model
           bandit.model
           epuck_new.model
           generalshop.model
           hokuyo-urg-04l.model
           imu.model
           pioneer2at.model
           pioneer2dx.model
           pioneer2gripper.model
           RZR-002.model
           sicklms200.model
           simplecar.model
           sonyvid30.model
           wizbot.model
)

INSTALL(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo/models)

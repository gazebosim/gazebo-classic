## Gazebo 5.0

### Gazebo 5.1.0
1. Added Joint Msg-to-SDF conversion functions and test.
    * [Pull request #1419](https://bitbucket.org/osrf/gazebo/pull-request/1419)

1. Added Visual, Material Msg-to-SDF conversion functions and ShaderType to string conversion functions.
    * [Pull request #1415](https://bitbucket.org/osrf/gazebo/pull-request/1415)

1. Implement Coulomb joint friction for BulletSliderJoint
  * [Pull request #1221](https://bitbucket.org/osrf/gazebo/pull-request/1452)
  * [Issue #1348](https://bitbucket.org/osrf/gazebo/issue/1348)

### Gazebo 5.0.0
1. Support for using [digital elevation maps](http://gazebosim.org/tutorials?tut=dem) has been added to debian packages.

1. C++11 support (C++11 compatible compiler is now required)
    * [Pull request #1340](https://bitbucket.org/osrf/gazebo/pull-request/1340)

1. Implemented private data pointer for the World class.
    * [Pull request #1383](https://bitbucket.org/osrf/gazebo/pull-request/1383)

1. Implemented private data pointer for the Scene class.
    * [Pull request #1385](https://bitbucket.org/osrf/gazebo/pull-request/1385)

1. Added a events::Event::resetWorld event that is triggered when World::Reset is called.
    * [Pull request #1332](https://bitbucket.org/osrf/gazebo/pull-request/1332)
    * [Issue #1375](https://bitbucket.org/osrf/gazebo/issue/1375)

1. Fixed `math::Box::GetCenter` functionality.
    * [Pull request #1278](https://bitbucket.org/osrf/gazebo/pull-request/1278)
    * [Issue #1327](https://bitbucket.org/osrf/gazebo/issue/1327)

1. Added a GUI timer plugin that facilitates the display and control a timer inside the Gazebo UI.
    * [Pull request #1270](https://bitbucket.org/osrf/gazebo/pull-request/1270)

1. Added ability to load plugins via SDF.
    * [Pull request #1261](https://bitbucket.org/osrf/gazebo/pull-request/1261)

1. Added GUIEvent to hide/show the left GUI pane.
    * [Pull request #1269](https://bitbucket.org/osrf/gazebo/pull-request/1269)

1. Modified KeyEventHandler and GLWidget so that hotkeys can be suppressed by custom KeyEvents set up by developers
    * [Pull request #1251](https://bitbucket.org/osrf/gazebo/pull-request/1251)

1. Added ability to read the directory where the log files are stored.
    * [Pull request #1277](https://bitbucket.org/osrf/gazebo/pull-request/1277)

1. Implemented a simulation cloner
    * [Pull request #1180](https://bitbucket.org/osrf/gazebo/pull-request/1180/clone-a-simulation)

1. Added GUI overlay plugins. Users can now write a Gazebo + QT plugin that displays widgets over the render window.
  * [Pull request #1181](https://bitbucket.org/osrf/gazebo/pull-request/1181)

1. Change behavior of Joint::SetVelocity, add Joint::SetVelocityLimit(unsigned int, double)
  * [Pull request #1218](https://bitbucket.org/osrf/gazebo/pull-request/1218)
  * [Issue #964](https://bitbucket.org/osrf/gazebo/issue/964)

1. Implement Coulomb joint friction for ODE
  * [Pull request #1221](https://bitbucket.org/osrf/gazebo/pull-request/1221)
  * [Issue #381](https://bitbucket.org/osrf/gazebo/issue/381)

1. Implement Coulomb joint friction for BulletHingeJoint
  * [Pull request #1221](https://bitbucket.org/osrf/gazebo/pull-request/1317)
  * [Issue #1348](https://bitbucket.org/osrf/gazebo/issue/1348)

1. Implemented camera lens distortion.
  * [Pull request #1213](https://bitbucket.org/osrf/gazebo/pull-request/1213)

1. Kill rogue gzservers left over from failed INTEGRATION_world_clone tests
   and improve robustness of `UNIT_gz_TEST`
  * [Pull request #1232](https://bitbucket.org/osrf/gazebo/pull-request/1232)
  * [Issue #1299](https://bitbucket.org/osrf/gazebo/issue/1299)

1. Added RenderWidget::ShowToolbar to toggle visibility of top toolbar.
  * [Pull request #1248](https://bitbucket.org/osrf/gazebo/pull-request/1248)

1. Fix joint axis visualization.
  * [Pull request #1258](https://bitbucket.org/osrf/gazebo/pull-request/1258)

1. Change UserCamera view control via joysticks. Clean up rate control vs. pose control.
   see UserCamera::OnJoyPose and UserCamera::OnJoyTwist. Added view twist control toggle
   with joystick button 1.
  * [Pull request #1249](https://bitbucket.org/osrf/gazebo/pull-request/1249)

1. Added RenderWidget::GetToolbar to get the top toolbar and change its actions on ModelEditor.
    * [Pull request #1263](https://bitbucket.org/osrf/gazebo/pull-request/1263)

1. Added accessor for MainWindow graphical widget to GuiIface.
    * [Pull request #1250](https://bitbucket.org/osrf/gazebo/pull-request/1250)

1. Added a ConfigWidget class that takes in a google protobuf message and generates widgets for configuring the fields in the message
    * [Pull request #1285](https://bitbucket.org/osrf/gazebo/pull-request/1285)

1. Added GLWidget::OnModelEditor when model editor is triggered, and MainWindow::OnEditorGroup to manually uncheck editor actions.
    * [Pull request #1283](https://bitbucket.org/osrf/gazebo/pull-request/1283)

1. Added Collision, Geometry, Inertial, Surface Msg-to-SDF conversion functions.
    * [Pull request #1315](https://bitbucket.org/osrf/gazebo/pull-request/1315)

1. Added "button modifier" fields (control, shift, and alt) to common::KeyEvent.
    * [Pull request #1325](https://bitbucket.org/osrf/gazebo/pull-request/1325)

1. Added inputs for environment variable GAZEBO_GUI_INI_FILE for reading a custom .ini file.
    * [Pull request #1252](https://bitbucket.org/osrf/gazebo/pull-request/1252)

1. Fixed crash on "permission denied" bug, added insert_model integration test.
    * [Pull request #1329](https://bitbucket.org/osrf/gazebo/pull-request/1329/)

1. Enable simbody joint tests, implement `SimbodyJoint::GetParam`, create
   `Joint::GetParam`, fix bug in `BulletHingeJoint::SetParam`.
    * [Pull request #1404](https://bitbucket.org/osrf/gazebo/pull-request/1404/)

1. Building editor updates
    1. Fixed inspector resizing.
        * [Pull request #1230](https://bitbucket.org/osrf/gazebo/pull-request/1230)
        * [Issue #395](https://bitbucket.org/osrf/gazebo/issue/395)

    1. Doors and windows move proportionally with wall.
        * [Pull request #1231](https://bitbucket.org/osrf/gazebo/pull-request/1231)
        * [Issue #368](https://bitbucket.org/osrf/gazebo/issue/368)

    1. Inspector dialogs stay on top.
        * [Pull request #1229](https://bitbucket.org/osrf/gazebo/pull-request/1229)
        * [Issue #417](https://bitbucket.org/osrf/gazebo/issue/417)

    1. Make model name editable on palette.
        * [Pull request #1239](https://bitbucket.org/osrf/gazebo/pull-request/1239)

    1. Import background image and improve add/delete levels.
        * [Pull request #1214](https://bitbucket.org/osrf/gazebo/pull-request/1214)
        * [Issue #422](https://bitbucket.org/osrf/gazebo/issue/422)
        * [Issue #361](https://bitbucket.org/osrf/gazebo/issue/361)

    1. Fix changing draw mode.
        * [Pull request #1233](https://bitbucket.org/osrf/gazebo/pull-request/1233)
        * [Issue #405](https://bitbucket.org/osrf/gazebo/issue/405)

    1. Tips on palette's top-right corner.
        * [Pull request #1241](https://bitbucket.org/osrf/gazebo/pull-request/1241)

    1. New buttons and layout for the palette.
        * [Pull request #1242](https://bitbucket.org/osrf/gazebo/pull-request/1242)

    1. Individual wall segments instead of polylines.
        * [Pull request #1246](https://bitbucket.org/osrf/gazebo/pull-request/1246)
        * [Issue #389](https://bitbucket.org/osrf/gazebo/issue/389)
        * [Issue #415](https://bitbucket.org/osrf/gazebo/issue/415)

    1. Fix exiting and saving, exiting when there's nothing drawn, fix text on popups.
        * [Pull request #1296](https://bitbucket.org/osrf/gazebo/pull-request/1296)

    1. Display measure for selected wall segment.
        * [Pull request #1291](https://bitbucket.org/osrf/gazebo/pull-request/1291)
        * [Issue #366](https://bitbucket.org/osrf/gazebo/issue/366)

    1. Highlight selected item's 3D visual.
        * [Pull request #1292](https://bitbucket.org/osrf/gazebo/pull-request/1292)

    1. Added color picker to inspector dialogs.
        * [Pull request #1298](https://bitbucket.org/osrf/gazebo/pull-request/1298)

    1. Snapping on by default, off holding Shift. Improved snapping.
        * [Pull request #1304](https://bitbucket.org/osrf/gazebo/pull-request/1304)

    1. Snap walls to length increments, moved scale to SegmentItem and added Get/SetScale, added SegmentItem::SnapAngle and SegmentItem::SnapLength.
        * [Pull request #1311](https://bitbucket.org/osrf/gazebo/pull-request/1311)

    1. Make buildings available in "Insert Models" tab, improve save flow.
        * [Pull request #1312](https://bitbucket.org/osrf/gazebo/pull-request/1312)

    1. Added EditorItem::SetHighlighted.
        * [Pull request #1308](https://bitbucket.org/osrf/gazebo/pull-request/1308)

    1. Current level is transparent, lower levels opaque, higher levels invisible.
        * [Pull request #1303](https://bitbucket.org/osrf/gazebo/pull-request/1303)

    1. Detach all child manips when item is deleted, added BuildingMaker::DetachAllChildren.
        * [Pull request #1316](https://bitbucket.org/osrf/gazebo/pull-request/1316)

    1. Added texture picker to inspector dialogs.
        * [Pull request #1306](https://bitbucket.org/osrf/gazebo/pull-request/1306)

    1. Measures for doors and windows. Added RectItem::angleOnWall and related Get/Set.
        * [Pull request #1322](https://bitbucket.org/osrf/gazebo/pull-request/1322)
        * [Issue #370](https://bitbucket.org/osrf/gazebo/issue/370)

    1. Added Gazebo/BuildingFrame material to display holes for doors and windows on walls.
        * [Pull request #1338](https://bitbucket.org/osrf/gazebo/pull-request/1338)

    1. Added Gazebo/Bricks material to be used as texture on the building editor.
        * [Pull request #1333](https://bitbucket.org/osrf/gazebo/pull-request/1333)

    1. Pick colors from the palette and assign on 3D view. Added mouse and key event handlers to BuildingMaker, and events to communicate from BuildingModelManip to EditorItem.
        * [Pull request #1336](https://bitbucket.org/osrf/gazebo/pull-request/1336)

    1. Pick textures from the palette and assign in 3D view.
        * [Pull request #1368](https://bitbucket.org/osrf/gazebo/pull-request/1368)

1. Model editor updates
    1. Fix adding/removing event filters .
        * [Pull request #1279](https://bitbucket.org/osrf/gazebo/pull-request/1279)

    1. Enabled multi-selection and align tool inside model editor.
        * [Pull request #1302](https://bitbucket.org/osrf/gazebo/pull-request/1302)
        * [Issue #1323](https://bitbucket.org/osrf/gazebo/issue/1323)

    1. Enabled snap mode inside model editor.
        * [Pull request #1331](https://bitbucket.org/osrf/gazebo/pull-request/1331)
        * [Issue #1318](https://bitbucket.org/osrf/gazebo/issue/1318)

    1. Implemented copy/pasting of links.
        * [Pull request #1330](https://bitbucket.org/osrf/gazebo/pull-request/1330)

1. GUI publishes model selection information on ~/selection topic.
    * [Pull request #1318](https://bitbucket.org/osrf/gazebo/pull-request/1318)

## Gazebo 4.0

### Gazebo 4.x.x (yyyy-mm-dd)

### Gazebo 4.1.0 (2014-11-20)

1. Add ArrangePlugin for arranging groups of models.
   Also add Model::ResetPhysicsStates to call Link::ResetPhysicsStates
   recursively on all links in model.
    * [Pull request #1208](https://bitbucket.org/osrf/gazebo/pull-request/1208)
1. The `gz model` command line tool will output model info using either `-i` for complete info, or `-p` for just the model pose.
    * [Pull request #1212](https://bitbucket.org/osrf/gazebo/pull-request/1212)
    * [DRCSim Issue #389](https://bitbucket.org/osrf/drcsim/issue/389)
1. Added SignalStats class for computing incremental signal statistics.
    * [Pull request #1198](https://bitbucket.org/osrf/gazebo/pull-request/1198)
1. Add InitialVelocityPlugin to setting the initial state of links
    * [Pull request #1237](https://bitbucket.org/osrf/gazebo/pull-request/1237)
1. Added Quaternion::Integrate function.
    * [Pull request #1255](https://bitbucket.org/osrf/gazebo/pull-request/1255)
1. Added ConvertJointType functions, display more joint info on model list.
    * [Pull request #1259](https://bitbucket.org/osrf/gazebo/pull-request/1259)
1. Added ModelListWidget::AddProperty, removed unnecessary checks on ModelListWidget.
    * [Pull request #1271](https://bitbucket.org/osrf/gazebo/pull-request/1271)
1. Fix loading collada meshes with unsupported input semantics.
    * [Pull request #1319](https://bitbucket.org/osrf/gazebo/pull-request/1319)
1. Fix race condition with ImuSensor not publishing after Reset World.
    * [Pull request #1448](https://bitbucket.org/osrf/gazebo/pull-request/1448)
    * [Pull request #1446](https://bitbucket.org/osrf/gazebo/pull-request/1446)
    * [Issue #236](https://bitbucket.org/osrf/gazebo/issue/236)
1. Fixed heightmap on OSX
    * [Pull request #1455](https://bitbucket.org/osrf/gazebo/pull-request/1455)

### Gazebo 4.0.2 (2014-09-23)

1. Fix and improve mechanism to generate pkgconfig libs
    * [Pull request #1027](https://bitbucket.org/osrf/gazebo/pull-request/1027)
    * [Issue #1284](https://bitbucket.org/osrf/gazebo/issue/1284)
1. Added arat.world
    * [Pull request #1205](https://bitbucket.org/osrf/gazebo/pull-request/1205)
1. Update gzprop to output zip files.
    * [Pull request #1197](https://bitbucket.org/osrf/gazebo/pull-request/1197)
1. Make Collision::GetShape a const function
    * [Pull requset #1189](https://bitbucket.org/osrf/gazebo/pull-request/1189)
1. Install missing physics headers
    * [Pull requset #1183](https://bitbucket.org/osrf/gazebo/pull-request/1183)
1. Remove SimbodyLink::AddTorque console message
    * [Pull requset #1185](https://bitbucket.org/osrf/gazebo/pull-request/1185)
1. Fix log xml
    * [Pull requset #1188](https://bitbucket.org/osrf/gazebo/pull-request/1188)

### Gazebo 4.0.0 (2014-08-08)

1. Added lcov support to cmake
    * [Pull request #1047](https://bitbucket.org/osrf/gazebo/pull-request/1047)
1. Fixed memory leak in image conversion
    * [Pull request #1057](https://bitbucket.org/osrf/gazebo/pull-request/1057)
1. Removed deprecated function
    * [Pull request #1067](https://bitbucket.org/osrf/gazebo/pull-request/1067)
1. Improved collada loading performance
    * [Pull request #1066](https://bitbucket.org/osrf/gazebo/pull-request/1066)
    * [Pull request #1082](https://bitbucket.org/osrf/gazebo/pull-request/1082)
    * [Issue #1134](https://bitbucket.org/osrf/gazebo/issue/1134)
1. Implemented a collada exporter
    * [Pull request #1064](https://bitbucket.org/osrf/gazebo/pull-request/1064)
1. Force torque sensor now makes use of sensor's pose.
    * [Pull request #1076](https://bitbucket.org/osrf/gazebo/pull-request/1076)
    * [Issue #940](https://bitbucket.org/osrf/gazebo/issue/940)
1. Fix Model::GetLinks segfault
    * [Pull request #1093](https://bitbucket.org/osrf/gazebo/pull-request/1093)
1. Fix deleting and saving lights in gzserver
    * [Pull request #1094](https://bitbucket.org/osrf/gazebo/pull-request/1094)
    * [Issue #1182](https://bitbucket.org/osrf/gazebo/issue/1182)
    * [Issue #346](https://bitbucket.org/osrf/gazebo/issue/346)
1. Fix Collision::GetWorldPose. The pose of a collision would not update properly.
    * [Pull request #1049](https://bitbucket.org/osrf/gazebo/pull-request/1049)
    * [Issue #1124](https://bitbucket.org/osrf/gazebo/issue/1124)
1. Fixed the animate_box and animate_joints examples
    * [Pull request #1086](https://bitbucket.org/osrf/gazebo/pull-request/1086)
1. Integrated Oculus Rift functionality
    * [Pull request #1074](https://bitbucket.org/osrf/gazebo/pull-request/1074)
    * [Pull request #1136](https://bitbucket.org/osrf/gazebo/pull-request/1136)
    * [Pull request #1139](https://bitbucket.org/osrf/gazebo/pull-request/1139)
1. Updated Base::GetScopedName
    * [Pull request #1104](https://bitbucket.org/osrf/gazebo/pull-request/1104)
1. Fix collada loader from adding duplicate materials into a Mesh
    * [Pull request #1105](https://bitbucket.org/osrf/gazebo/pull-request/1105)
    * [Issue #1180](https://bitbucket.org/osrf/gazebo/issue/1180)
1. Integrated Razer Hydra functionality
    * [Pull request #1083](https://bitbucket.org/osrf/gazebo/pull-request/1083)
    * [Pull request #1109](https://bitbucket.org/osrf/gazebo/pull-request/1109)
1. Added ability to copy and paste models in the GUI
    * [Pull request #1103](https://bitbucket.org/osrf/gazebo/pull-request/1103)
1. Removed unnecessary inclusion of gazebo.hh and common.hh in plugins
    * [Pull request #1111](https://bitbucket.org/osrf/gazebo/pull-request/1111)
1. Added ability to specify custom road textures
    * [Pull request #1027](https://bitbucket.org/osrf/gazebo/pull-request/1027)
1. Added support for DART 4.1
    * [Pull request #1113](https://bitbucket.org/osrf/gazebo/pull-request/1113)
    * [Pull request #1132](https://bitbucket.org/osrf/gazebo/pull-request/1132)
    * [Pull request #1134](https://bitbucket.org/osrf/gazebo/pull-request/1134)
    * [Pull request #1154](https://bitbucket.org/osrf/gazebo/pull-request/1154)
1. Allow position of joints to be directly set.
    * [Pull request #1097](https://bitbucket.org/osrf/gazebo/pull-request/1097)
    * [Issue #1138](https://bitbucket.org/osrf/gazebo/issue/1138)
1. Added extruded polyline geometry
    * [Pull request #1026](https://bitbucket.org/osrf/gazebo/pull-request/1026)
1. Fixed actor animation
    * [Pull request #1133](https://bitbucket.org/osrf/gazebo/pull-request/1133)
    * [Pull request #1141](https://bitbucket.org/osrf/gazebo/pull-request/1141)
1. Generate a versioned cmake config file
    * [Pull request #1153](https://bitbucket.org/osrf/gazebo/pull-request/1153)
    * [Issue #1226](https://bitbucket.org/osrf/gazebo/issue/1226)
1. Added KMeans class
    * [Pull request #1147](https://bitbucket.org/osrf/gazebo/pull-request/1147)
1. Added --summary-range feature to bitbucket pullrequest tool
    * [Pull request #1156](https://bitbucket.org/osrf/gazebo/pull-request/1156)
1. Updated web links
    * [Pull request #1159](https://bitbucket.org/osrf/gazebo/pull-request/1159)
1. Update tests
    * [Pull request #1155](https://bitbucket.org/osrf/gazebo/pull-request/1155)
    * [Pull request #1143](https://bitbucket.org/osrf/gazebo/pull-request/1143)
    * [Pull request #1138](https://bitbucket.org/osrf/gazebo/pull-request/1138)
    * [Pull request #1140](https://bitbucket.org/osrf/gazebo/pull-request/1140)
    * [Pull request #1127](https://bitbucket.org/osrf/gazebo/pull-request/1127)
    * [Pull request #1115](https://bitbucket.org/osrf/gazebo/pull-request/1115)
    * [Pull request #1102](https://bitbucket.org/osrf/gazebo/pull-request/1102)
    * [Pull request #1087](https://bitbucket.org/osrf/gazebo/pull-request/1087)
    * [Pull request #1084](https://bitbucket.org/osrf/gazebo/pull-request/1084)

## Gazebo 3.0

### Gazebo 3.x.x (yyyy-mm-dd)

1. Fixed sonar and wireless sensor visualization
    * [Pull request #1254](https://bitbucket.org/osrf/gazebo/pull-request/1254)
1. Update visual bounding box when model is selected
    * [Pull request #1280](https://bitbucket.org/osrf/gazebo/pull-request/1280)

### Gazebo 3.1.0 (2014-08-08)

1. Implemented Simbody::Link::Set*Vel
    * [Pull request #1160](https://bitbucket.org/osrf/gazebo/pull-request/1160)
    * [Issue #1012](https://bitbucket.org/osrf/gazebo/issue/1012)
1. Added World::RemoveModel function
    * [Pull request #1106](https://bitbucket.org/osrf/gazebo/pull-request/1106)
    * [Issue #1177](https://bitbucket.org/osrf/gazebo/issue/1177)
1. Fix exit from camera follow mode using the escape key
    * [Pull request #1137](https://bitbucket.org/osrf/gazebo/pull-request/1137)
    * [Issue #1220](https://bitbucket.org/osrf/gazebo/issue/1220)
1. Added support for SDF joint spring stiffness and reference positions
    * [Pull request #1117](https://bitbucket.org/osrf/gazebo/pull-request/1117)
1. Removed the gzmodel_create script
    * [Pull request #1130](https://bitbucket.org/osrf/gazebo/pull-request/1130)
1. Added Vector2 dot product
    * [Pull request #1101](https://bitbucket.org/osrf/gazebo/pull-request/1101)
1. Added SetPositionPID and SetVelocityPID to JointController
    * [Pull request #1091](https://bitbucket.org/osrf/gazebo/pull-request/1091)
1. Fix gzclient startup crash with ogre 1.9
    * [Pull request #1098](https://bitbucket.org/osrf/gazebo/pull-request/1098)
    * [Issue #996](https://bitbucket.org/osrf/gazebo/issue/996)
1. Update the bitbucket_pullrequests tool
    * [Pull request #1108](https://bitbucket.org/osrf/gazebo/pull-request/1108)
1. Light properties now remain in place after move by the user via the GUI.
    * [Pull request #1110](https://bitbucket.org/osrf/gazebo/pull-request/1110)
    * [Issue #1211](https://bitbucket.org/osrf/gazebo/issue/1211)
1. Allow position of joints to be directly set.
    * [Pull request #1096](https://bitbucket.org/osrf/gazebo/pull-request/1096)
    * [Issue #1138](https://bitbucket.org/osrf/gazebo/issue/1138)

### Gazebo 3.0.0 (2014-04-11)

1. Fix bug when deleting the sun light
    * [Pull request #1088](https://bitbucket.org/osrf/gazebo/pull-request/1088)
    * [Issue #1133](https://bitbucket.org/osrf/gazebo/issue/1133)
1. Fix ODE screw joint
    * [Pull request #1078](https://bitbucket.org/osrf/gazebo/pull-request/1078)
    * [Issue #1167](https://bitbucket.org/osrf/gazebo/issue/1167)
1. Update joint integration tests
    * [Pull request #1081](https://bitbucket.org/osrf/gazebo/pull-request/1081)
1. Fixed false positives in cppcheck.
    * [Pull request #1061](https://bitbucket.org/osrf/gazebo/pull-request/1061)
1. Made joint axis reference frame relative to child, and updated simbody and dart accordingly.
    * [Pull request #1069](https://bitbucket.org/osrf/gazebo/pull-request/1069)
    * [Issue #494](https://bitbucket.org/osrf/gazebo/issue/494)
    * [Issue #1143](https://bitbucket.org/osrf/gazebo/issue/1143)
1. Added ability to pass vector of strings to SetupClient and SetupServer
    * [Pull request #1068](https://bitbucket.org/osrf/gazebo/pull-request/1068)
    * [Issue #1132](https://bitbucket.org/osrf/gazebo/issue/1132)
1. Fix error correction in screw constraints for ODE
    * [Pull request #1159](https://bitbucket.org/osrf/gazebo/pull-request/1159)
    * [Issue #1159](https://bitbucket.org/osrf/gazebo/issue/1159)
1. Improved pkgconfig with SDF
    * [Pull request #1062](https://bitbucket.org/osrf/gazebo/pull-request/1062)
1. Added a plugin to simulate aero dynamics
    * [Pull request #905](https://bitbucket.org/osrf/gazebo/pull-request/905)
1. Updated bullet support
    * [Issue #1069](https://bitbucket.org/osrf/gazebo/issue/1069)
    * [Pull request #1011](https://bitbucket.org/osrf/gazebo/pull-request/1011)
    * [Pull request #996](https://bitbucket.org/osrf/gazebo/pull-request/966)
    * [Pull request #1024](https://bitbucket.org/osrf/gazebo/pull-request/1024)
1. Updated simbody support
    * [Pull request #995](https://bitbucket.org/osrf/gazebo/pull-request/995)
1. Updated worlds to SDF 1.5
    * [Pull request #1021](https://bitbucket.org/osrf/gazebo/pull-request/1021)
1. Improvements to ODE
    * [Pull request #1001](https://bitbucket.org/osrf/gazebo/pull-request/1001)
    * [Pull request #1014](https://bitbucket.org/osrf/gazebo/pull-request/1014)
    * [Pull request #1015](https://bitbucket.org/osrf/gazebo/pull-request/1015)
    * [Pull request #1016](https://bitbucket.org/osrf/gazebo/pull-request/1016)
1. New command line tool
    * [Pull request #972](https://bitbucket.org/osrf/gazebo/pull-request/972)
1. Graphical user interface improvements
    * [Pull request #971](https://bitbucket.org/osrf/gazebo/pull-request/971)
    * [Pull request #1013](https://bitbucket.org/osrf/gazebo/pull-request/1013)
    * [Pull request #989](https://bitbucket.org/osrf/gazebo/pull-request/989)
1. Created a friction pyramid class
    * [Pull request #935](https://bitbucket.org/osrf/gazebo/pull-request/935)
1. Added GetWorldEnergy functions to Model, Joint, and Link
    * [Pull request #1017](https://bitbucket.org/osrf/gazebo/pull-request/1017)
1. Preparing Gazebo for admission into Ubuntu
    * [Pull request #969](https://bitbucket.org/osrf/gazebo/pull-request/969)
    * [Pull request #998](https://bitbucket.org/osrf/gazebo/pull-request/998)
    * [Pull request #1002](https://bitbucket.org/osrf/gazebo/pull-request/1002)
1. Add method for querying if useImplicitStiffnessDamping flag is set for a given joint
    * [Issue #629](https://bitbucket.org/osrf/gazebo/issue/629)
    * [Pull request #1006](https://bitbucket.org/osrf/gazebo/pull-request/1006)
1. Fix joint axis frames
    * [Issue #494](https://bitbucket.org/osrf/gazebo/issue/494)
    * [Pull request #963](https://bitbucket.org/osrf/gazebo/pull-request/963)
1. Compute joint anchor pose relative to parent
    * [Issue #1029](https://bitbucket.org/osrf/gazebo/issue/1029)
    * [Pull request #982](https://bitbucket.org/osrf/gazebo/pull-request/982)
1. Cleanup the installed worlds
    * [Issue #1036](https://bitbucket.org/osrf/gazebo/issue/1036)
    * [Pull request #984](https://bitbucket.org/osrf/gazebo/pull-request/984)
1. Update to the GPS sensor
    * [Issue #1059](https://bitbucket.org/osrf/gazebo/issue/1059)
    * [Pull request #984](https://bitbucket.org/osrf/gazebo/pull-request/984)
1. Removed libtool from plugin loading
    * [Pull request #981](https://bitbucket.org/osrf/gazebo/pull-request/981)
1. Added functions to get inertial information for a link in the world frame.
    * [Pull request #1005](https://bitbucket.org/osrf/gazebo/pull-request/1005)

## Gazebo 2.0

### Gazebo 2.2.3 (2014-04-29)

1. Removed redundant call to World::Init
    * [Pull request #1107](https://bitbucket.org/osrf/gazebo/pull-request/1107)
    * [Issue #1208](https://bitbucket.org/osrf/gazebo/issue/1208)
1. Return proper error codes when gazebo exits
    * [Pull request #1085](https://bitbucket.org/osrf/gazebo/pull-request/1085)
    * [Issue #1178](https://bitbucket.org/osrf/gazebo/issue/1178)
1. Fixed Camera::GetWorldRotation().
    * [Pull request #1071](https://bitbucket.org/osrf/gazebo/pull-request/1071)
    * [Issue #1087](https://bitbucket.org/osrf/gazebo/issue/1087)
1. Fixed memory leak in image conversion
    * [Pull request #1073](https://bitbucket.org/osrf/gazebo/pull-request/1073)

### Gazebo 2.2.0 (2014-01-10)

1. Fix compilation when using OGRE-1.9 (full support is being worked on)
    * [Issue #994](https://bitbucket.org/osrf/gazebo/issue/994)
    * [Issue #995](https://bitbucket.org/osrf/gazebo/issue/995)
    * [Issue #996](https://bitbucket.org/osrf/gazebo/issue/996)
    * [Pull request #883](https://bitbucket.org/osrf/gazebo/pull-request/883)
1. Added unit test for issue 624.
    * [Issue #624](https://bitbucket.org/osrf/gazebo/issue/624).
    * [Pull request #889](https://bitbucket.org/osrf/gazebo/pull-request/889)
1. Use 3x3 PCF shadows for smoother shadows.
    * [Pull request #887](https://bitbucket.org/osrf/gazebo/pull-request/887)
1. Update manpage copyright to 2014.
    * [Pull request #893](https://bitbucket.org/osrf/gazebo/pull-request/893)
1. Added friction integration test .
    * [Pull request #885](https://bitbucket.org/osrf/gazebo/pull-request/885)
1. Fix joint anchor when link pose is not specified.
    * [Issue #978](https://bitbucket.org/osrf/gazebo/issue/978)
    * [Pull request #862](https://bitbucket.org/osrf/gazebo/pull-request/862)
1. Added (ESC) tooltip for GUI Selection Mode icon.
    * [Issue #993](https://bitbucket.org/osrf/gazebo/issue/993)
    * [Pull request #888](https://bitbucket.org/osrf/gazebo/pull-request/888)
1. Removed old comment about resolved issue.
    * [Issue #837](https://bitbucket.org/osrf/gazebo/issue/837)
    * [Pull request #880](https://bitbucket.org/osrf/gazebo/pull-request/880)
1. Made SimbodyLink::Get* function thread-safe
    * [Issue #918](https://bitbucket.org/osrf/gazebo/issue/918)
    * [Pull request #872](https://bitbucket.org/osrf/gazebo/pull-request/872)
1. Suppressed spurious gzlog messages in ODE::Body
    * [Issue #983](https://bitbucket.org/osrf/gazebo/issue/983)
    * [Pull request #875](https://bitbucket.org/osrf/gazebo/pull-request/875)
1. Fixed Force Torque Sensor Test by properly initializing some values.
    * [Issue #982](https://bitbucket.org/osrf/gazebo/issue/982)
    * [Pull request #869](https://bitbucket.org/osrf/gazebo/pull-request/869)
1. Added breakable joint plugin to support breakable walls.
    * [Pull request #865](https://bitbucket.org/osrf/gazebo/pull-request/865)
1. Used different tuple syntax to fix compilation on OSX mavericks.
    * [Issue #947](https://bitbucket.org/osrf/gazebo/issue/947)
    * [Pull request #858](https://bitbucket.org/osrf/gazebo/pull-request/858)
1. Fixed sonar test and deprecation warning.
    * [Pull request #856](https://bitbucket.org/osrf/gazebo/pull-request/856)
1. Speed up test compilation.
    * Part of [Issue #955](https://bitbucket.org/osrf/gazebo/issue/955)
    * [Pull request #846](https://bitbucket.org/osrf/gazebo/pull-request/846)
1. Added Joint::SetEffortLimit API
    * [Issue #923](https://bitbucket.org/osrf/gazebo/issue/923)
    * [Pull request #808](https://bitbucket.org/osrf/gazebo/pull-request/808)
1. Made bullet output less verbose.
    * [Pull request #839](https://bitbucket.org/osrf/gazebo/pull-request/839)
1. Convergence acceleration and stability tweak to make atlas_v3 stable
    * [Issue #895](https://bitbucket.org/osrf/gazebo/issue/895)
    * [Pull request #772](https://bitbucket.org/osrf/gazebo/pull-request/772)
1. Added colors, textures and world files for the SPL RoboCup environment
    * [Pull request #838](https://bitbucket.org/osrf/gazebo/pull-request/838)
1. Fixed bitbucket_pullrequests tool to work with latest BitBucket API.
    * [Issue #933](https://bitbucket.org/osrf/gazebo/issue/933)
    * [Pull request #841](https://bitbucket.org/osrf/gazebo/pull-request/841)
1. Fixed cppcheck warnings.
    * [Pull request #842](https://bitbucket.org/osrf/gazebo/pull-request/842)

### Gazebo 2.1.0 (2013-11-08)
1. Fix mainwindow unit test
    * [Pull request #752](https://bitbucket.org/osrf/gazebo/pull-request/752)
1. Visualize moment of inertia
    * Pull request [#745](https://bitbucket.org/osrf/gazebo/pull-request/745), [#769](https://bitbucket.org/osrf/gazebo/pull-request/769), [#787](https://bitbucket.org/osrf/gazebo/pull-request/787)
    * [Issue #203](https://bitbucket.org/osrf/gazebo/issue/203)
1. Update tool to count lines of code
    * [Pull request #758](https://bitbucket.org/osrf/gazebo/pull-request/758)
1. Implement World::Clear
    * Pull request [#785](https://bitbucket.org/osrf/gazebo/pull-request/785), [#804](https://bitbucket.org/osrf/gazebo/pull-request/804)
1. Improve Bullet support
    * [Pull request #805](https://bitbucket.org/osrf/gazebo/pull-request/805)
1. Fix doxygen spacing
    * [Pull request #740](https://bitbucket.org/osrf/gazebo/pull-request/740)
1. Add tool to generate model images for thepropshop.org
    * [Pull request #734](https://bitbucket.org/osrf/gazebo/pull-request/734)
1. Added paging support for terrains
    * [Pull request #707](https://bitbucket.org/osrf/gazebo/pull-request/707)
1. Added plugin path to LID_LIBRARY_PATH in setup.sh
    * [Pull request #750](https://bitbucket.org/osrf/gazebo/pull-request/750)
1. Fix for OSX
    * [Pull request #766](https://bitbucket.org/osrf/gazebo/pull-request/766)
    * [Pull request #786](https://bitbucket.org/osrf/gazebo/pull-request/786)
    * [Issue #906](https://bitbucket.org/osrf/gazebo/issue/906)
1. Update copyright information
    * [Pull request #771](https://bitbucket.org/osrf/gazebo/pull-request/771)
1. Enable screen dependent tests
    * [Pull request #764](https://bitbucket.org/osrf/gazebo/pull-request/764)
    * [Issue #811](https://bitbucket.org/osrf/gazebo/issue/811)
1. Fix gazebo command line help message
    * [Pull request #775](https://bitbucket.org/osrf/gazebo/pull-request/775)
    * [Issue #898](https://bitbucket.org/osrf/gazebo/issue/898)
1. Fix man page test
    * [Pull request #774](https://bitbucket.org/osrf/gazebo/pull-request/774)
1. Improve load time by reducing calls to RTShader::Update
    * [Pull request #773](https://bitbucket.org/osrf/gazebo/pull-request/773)
    * [Issue #877](https://bitbucket.org/osrf/gazebo/issue/877)
1. Fix joint visualization
    * [Pull request #776](https://bitbucket.org/osrf/gazebo/pull-request/776)
    * [Pull request #802](https://bitbucket.org/osrf/gazebo/pull-request/802)
    * [Issue #464](https://bitbucket.org/osrf/gazebo/issue/464)
1. Add helpers to fix NaN
    * [Pull request #742](https://bitbucket.org/osrf/gazebo/pull-request/742)
1. Fix model resizing via the GUI
    * [Pull request #763](https://bitbucket.org/osrf/gazebo/pull-request/763)
    * [Issue #885](https://bitbucket.org/osrf/gazebo/issue/885)
1. Simplify gzlog test by using sha1
    * [Pull request #781](https://bitbucket.org/osrf/gazebo/pull-request/781)
    * [Issue #837](https://bitbucket.org/osrf/gazebo/issue/837)
1. Enable cppcheck for header files
    * [Pull request #782](https://bitbucket.org/osrf/gazebo/pull-request/782)
    * [Issue #907](https://bitbucket.org/osrf/gazebo/issue/907)
1. Fix broken regression test
    * [Pull request #784](https://bitbucket.org/osrf/gazebo/pull-request/784)
    * [Issue #884](https://bitbucket.org/osrf/gazebo/issue/884)
1. All simbody and dart to pass tests
    * [Pull request #790](https://bitbucket.org/osrf/gazebo/pull-request/790)
    * [Issue #873](https://bitbucket.org/osrf/gazebo/issue/873)
1. Fix camera rotation from SDF
    * [Pull request #789](https://bitbucket.org/osrf/gazebo/pull-request/789)
    * [Issue #920](https://bitbucket.org/osrf/gazebo/issue/920)
1. Fix bitbucket pullrequest command line tool to match new API
    * [Pull request #803](https://bitbucket.org/osrf/gazebo/pull-request/803)
1. Fix transceiver spawn errors in tests
    * [Pull request #811](https://bitbucket.org/osrf/gazebo/pull-request/811)
    * [Pull request #814](https://bitbucket.org/osrf/gazebo/pull-request/814)

### Gazebo 2.0.0 (2013-10-08)
1. Refactor code check tool.
    * [Pull Request #669](https://bitbucket.org/osrf/gazebo/pull-request/669)
1. Added pull request tool for Bitbucket.
    * [Pull Request #670](https://bitbucket.org/osrf/gazebo/pull-request/670)
    * [Pull Request #691](https://bitbucket.org/osrf/gazebo/pull-request/671)
1. New wireless receiver and transmitter sensor models.
    * [Pull Request #644](https://bitbucket.org/osrf/gazebo/pull-request/644)
    * [Pull Request #675](https://bitbucket.org/osrf/gazebo/pull-request/675)
    * [Pull Request #727](https://bitbucket.org/osrf/gazebo/pull-request/727)
1. Audio support using OpenAL.
    * [Pull Request #648](https://bitbucket.org/osrf/gazebo/pull-request/648)
    * [Pull Request #704](https://bitbucket.org/osrf/gazebo/pull-request/704)
1. Simplify command-line parsing of gztopic echo output.
    * [Pull Request #674](https://bitbucket.org/osrf/gazebo/pull-request/674)
    * Resolves: [Issue #795](https://bitbucket.org/osrf/gazebo/issue/795)
1. Use UNIX directories through the user of GNUInstallDirs cmake module.
    * [Pull Request #676](https://bitbucket.org/osrf/gazebo/pull-request/676)
    * [Pull Request #681](https://bitbucket.org/osrf/gazebo/pull-request/681)
1. New GUI interactions for object manipulation.
    * [Pull Request #634](https://bitbucket.org/osrf/gazebo/pull-request/634)
1. Fix for OSX menubar.
    * [Pull Request #677](https://bitbucket.org/osrf/gazebo/pull-request/677)
1. Remove internal SDF directories and dependencies.
    * [Pull Request #680](https://bitbucket.org/osrf/gazebo/pull-request/680)
1. Add minimum version for sdformat.
    * [Pull Request #682](https://bitbucket.org/osrf/gazebo/pull-request/682)
    * Resolves: [Issue #818](https://bitbucket.org/osrf/gazebo/issue/818)
1. Allow different gtest parameter types with ServerFixture
    * [Pull Request #686](https://bitbucket.org/osrf/gazebo/pull-request/686)
    * Resolves: [Issue #820](https://bitbucket.org/osrf/gazebo/issue/820)
1. GUI model scaling when using Bullet.
    * [Pull Request #683](https://bitbucket.org/osrf/gazebo/pull-request/683)
1. Fix typo in cmake config.
    * [Pull Request #694](https://bitbucket.org/osrf/gazebo/pull-request/694)
    * Resolves: [Issue #824](https://bitbucket.org/osrf/gazebo/issue/824)
1. Remove gazebo include subdir from pkgconfig and cmake config.
    * [Pull Request #691](https://bitbucket.org/osrf/gazebo/pull-request/691)
1. Torsional spring demo
    * [Pull Request #693](https://bitbucket.org/osrf/gazebo/pull-request/693)
1. Remove repeated call to SetAxis in Joint.cc
    * [Pull Request #695](https://bitbucket.org/osrf/gazebo/pull-request/695)
    * Resolves: [Issue #823](https://bitbucket.org/osrf/gazebo/issue/823)
1. Add test for rotational joints.
    * [Pull Request #697](https://bitbucket.org/osrf/gazebo/pull-request/697)
    * Resolves: [Issue #820](https://bitbucket.org/osrf/gazebo/issue/820)
1. Fix compilation of tests using Joint base class
    * [Pull Request #701](https://bitbucket.org/osrf/gazebo/pull-request/701)
1. Terrain paging implemented.
    * [Pull Request #687](https://bitbucket.org/osrf/gazebo/pull-request/687)
1. Improve timeout error reporting in ServerFixture
    * [Pull Request #705](https://bitbucket.org/osrf/gazebo/pull-request/705)
1. Fix mouse picking for cases where visuals overlap with the laser
    * [Pull Request #709](https://bitbucket.org/osrf/gazebo/pull-request/709)
1. Fix string literals for OSX
    * [Pull Request #712](https://bitbucket.org/osrf/gazebo/pull-request/712)
    * Resolves: [Issue #803](https://bitbucket.org/osrf/gazebo/issue/803)
1. Support for ENABLE_TESTS_COMPILATION cmake parameter
    * [Pull Request #708](https://bitbucket.org/osrf/gazebo/pull-request/708)
1. Updated system gui plugin
    * [Pull Request #702](https://bitbucket.org/osrf/gazebo/pull-request/702)
1. Fix force torque unit test issue
    * [Pull Request #673](https://bitbucket.org/osrf/gazebo/pull-request/673)
    * Resolves: [Issue #813](https://bitbucket.org/osrf/gazebo/issue/813)
1. Use variables to control auto generation of CFlags
    * [Pull Request #699](https://bitbucket.org/osrf/gazebo/pull-request/699)
1. Remove deprecated functions.
    * [Pull Request #715](https://bitbucket.org/osrf/gazebo/pull-request/715)
1. Fix typo in `Camera.cc`
    * [Pull Request #719](https://bitbucket.org/osrf/gazebo/pull-request/719)
    * Resolves: [Issue #846](https://bitbucket.org/osrf/gazebo/issue/846)
1. Performance improvements
    * [Pull Request #561](https://bitbucket.org/osrf/gazebo/pull-request/561)
1. Fix gripper model.
    * [Pull Request #713](https://bitbucket.org/osrf/gazebo/pull-request/713)
    * Resolves: [Issue #314](https://bitbucket.org/osrf/gazebo/issue/314)
1. First part of Simbody integration
    * [Pull Request #716](https://bitbucket.org/osrf/gazebo/pull-request/716)

## Gazebo 1.9

### Gazebo 1.9.6 (2014-04-29)

1. Refactored inertia ratio reduction for ODE
    * [Pull request #1114](https://bitbucket.org/osrf/gazebo/pull-request/1114)
1. Improved collada loading performance
    * [Pull request #1075](https://bitbucket.org/osrf/gazebo/pull-request/1075)

### Gazebo 1.9.3 (2014-01-10)

1. Add thickness to plane to remove shadow flickering.
    * [Pull request #886](https://bitbucket.org/osrf/gazebo/pull-request/886)
1. Temporary GUI shadow toggle fix.
    * [Issue #925](https://bitbucket.org/osrf/gazebo/issue/925)
    * [Pull request #868](https://bitbucket.org/osrf/gazebo/pull-request/868)
1. Fix memory access bugs with libc++ on mavericks.
    * [Issue #965](https://bitbucket.org/osrf/gazebo/issue/965)
    * [Pull request #857](https://bitbucket.org/osrf/gazebo/pull-request/857)
    * [Pull request #881](https://bitbucket.org/osrf/gazebo/pull-request/881)
1. Replaced printf with cout in gztopic hz.
    * [Issue #969](https://bitbucket.org/osrf/gazebo/issue/969)
    * [Pull request #854](https://bitbucket.org/osrf/gazebo/pull-request/854)
1. Add Dark grey material and fix indentation.
    * [Pull request #851](https://bitbucket.org/osrf/gazebo/pull-request/851)
1. Fixed sonar sensor unit test.
    * [Pull request #848](https://bitbucket.org/osrf/gazebo/pull-request/848)
1. Convergence acceleration and stability tweak to make atlas_v3 stable.
    * [Pull request #845](https://bitbucket.org/osrf/gazebo/pull-request/845)
1. Update gtest to 1.7.0 to resolve problems with libc++.
    * [Issue #947](https://bitbucket.org/osrf/gazebo/issue/947)
    * [Pull request #827](https://bitbucket.org/osrf/gazebo/pull-request/827)
1. Fixed LD_LIBRARY_PATH for plugins.
    * [Issue #957](https://bitbucket.org/osrf/gazebo/issue/957)
    * [Pull request #844](https://bitbucket.org/osrf/gazebo/pull-request/844)
1. Fix transceiver sporadic errors.
    * Backport of [pull request #811](https://bitbucket.org/osrf/gazebo/pull-request/811)
    * [Pull request #836](https://bitbucket.org/osrf/gazebo/pull-request/836)
1. Modified the MsgTest to be deterministic with time checks.
    * [Pull request #843](https://bitbucket.org/osrf/gazebo/pull-request/843)
1. Fixed seg fault in LaserVisual.
    * [Issue #950](https://bitbucket.org/osrf/gazebo/issue/950)
    * [Pull request #832](https://bitbucket.org/osrf/gazebo/pull-request/832)
1. Implemented the option to disable tests that need a working screen to run properly.
    * Backport of [Pull request #764](https://bitbucket.org/osrf/gazebo/pull-request/764)
    * [Pull request #837](https://bitbucket.org/osrf/gazebo/pull-request/837)
1. Cleaned up gazebo shutdown.
    * [Pull request #829](https://bitbucket.org/osrf/gazebo/pull-request/829)
1. Fixed bug associated with loading joint child links.
    * [Issue #943](https://bitbucket.org/osrf/gazebo/issue/943)
    * [Pull request #820](https://bitbucket.org/osrf/gazebo/pull-request/820)

### Gazebo 1.9.2 (2013-11-08)
1. Fix enable/disable sky and clouds from SDF
    * [Pull request #809](https://bitbucket.org/osrf/gazebo/pull-request/809])
1. Fix occasional blank GUI screen on startup
    * [Pull request #815](https://bitbucket.org/osrf/gazebo/pull-request/815])
1. Fix GPU laser when interacting with heightmaps
    * [Pull request #796](https://bitbucket.org/osrf/gazebo/pull-request/796])
1. Added API/ABI checker command line tool
    * [Pull request #765](https://bitbucket.org/osrf/gazebo/pull-request/765])
1. Added gtest version information
    * [Pull request #801](https://bitbucket.org/osrf/gazebo/pull-request/801])
1. Fix GUI world saving
    * [Pull request #806](https://bitbucket.org/osrf/gazebo/pull-request/806])
1. Enable anti-aliasing for camera sensor
    * [Pull request #800](https://bitbucket.org/osrf/gazebo/pull-request/800])
1. Make sensor noise deterministic
    * [Pull request #788](https://bitbucket.org/osrf/gazebo/pull-request/788])
1. Fix build problem
    * [Issue #901](https://bitbucket.org/osrf/gazebo/issue/901)
    * [Pull request #778](https://bitbucket.org/osrf/gazebo/pull-request/778])
1. Fix a typo in Camera.cc
    * [Pull request #720](https://bitbucket.org/osrf/gazebo/pull-request/720])
    * [Issue #846](https://bitbucket.org/osrf/gazebo/issue/846)
1. Fix OSX menu bar
    * [Pull request #688](https://bitbucket.org/osrf/gazebo/pull-request/688])
1. Fix gazebo::init by calling sdf::setFindCallback() before loading the sdf in gzfactory.
    * [Pull request #678](https://bitbucket.org/osrf/gazebo/pull-request/678])
    * [Issue #817](https://bitbucket.org/osrf/gazebo/issue/817)

### Gazebo 1.9.1 (2013-08-20)
* Deprecate header files that require case-sensitive filesystem (e.g. Common.hh, Physics.hh) [https://bitbucket.org/osrf/gazebo/pull-request/638/fix-for-775-deprecate-headers-that-require]
* Initial support for building on Mac OS X [https://bitbucket.org/osrf/gazebo/pull-request/660/osx-support-for-gazebo-19] [https://bitbucket.org/osrf/gazebo/pull-request/657/cmake-fixes-for-osx]
* Fixes for various issues [https://bitbucket.org/osrf/gazebo/pull-request/635/fix-for-issue-792/diff] [https://bitbucket.org/osrf/gazebo/pull-request/628/allow-scoped-and-non-scoped-joint-names-to/diff] [https://bitbucket.org/osrf/gazebo/pull-request/636/fix-build-dependency-in-message-generation/diff] [https://bitbucket.org/osrf/gazebo/pull-request/639/make-the-unversioned-setupsh-a-copy-of-the/diff] [https://bitbucket.org/osrf/gazebo/pull-request/650/added-missing-lib-to-player-client-library/diff] [https://bitbucket.org/osrf/gazebo/pull-request/656/install-gzmode_create-without-sh-suffix/diff]

### Gazebo 1.9.0 (2013-07-23)
* Use external package [sdformat](https://bitbucket.org/osrf/sdformat) for sdf parsing, refactor the `Element::GetValue*` function calls, and deprecate Gazebo's internal sdf parser [https://bitbucket.org/osrf/gazebo/pull-request/627]
* Improved ROS support ([[Tutorials#ROS_Integration |documentation here]]) [https://bitbucket.org/osrf/gazebo/pull-request/559]
* Added Sonar, Force-Torque, and Tactile Pressure sensors [https://bitbucket.org/osrf/gazebo/pull-request/557], [https://bitbucket.org/osrf/gazebo/pull-request/567]
* Add compile-time defaults for environment variables so that sourcing setup.sh is unnecessary in most cases [https://bitbucket.org/osrf/gazebo/pull-request/620]
* Enable user camera to follow objects in client window [https://bitbucket.org/osrf/gazebo/pull-request/603]
* Install protobuf message files for use in custom messages [https://bitbucket.org/osrf/gazebo/pull-request/614]
* Change default compilation flags to improve debugging [https://bitbucket.org/osrf/gazebo/pull-request/617]
* Change to supported relative include paths [https://bitbucket.org/osrf/gazebo/pull-request/594]
* Fix display of laser scans when sensor is rotated [https://bitbucket.org/osrf/gazebo/pull-request/599]

## Gazebo 1.8

### Gazebo 1.8.7 (2013-07-16)
* Fix bug in URDF parsing of Vector3 elements [https://bitbucket.org/osrf/gazebo/pull-request/613]
* Fix compilation errors with newest libraries [https://bitbucket.org/osrf/gazebo/pull-request/615]

### Gazebo 1.8.6 (2013-06-07)
* Fix inertia lumping in the URDF parser[https://bitbucket.org/osrf/gazebo/pull-request/554]
* Fix for ODEJoint CFM damping sign error [https://bitbucket.org/osrf/gazebo/pull-request/586]
* Fix transport memory growth[https://bitbucket.org/osrf/gazebo/pull-request/584]
* Reduce log file data in order to reduce buffer growth that results in out of memory kernel errors[https://bitbucket.org/osrf/gazebo/pull-request/587]

### Gazebo 1.8.5 (2013-06-04)
* Fix Gazebo build for machines without a valid display.[https://bitbucket.org/osrf/gazebo/commits/37f00422eea03365b839a632c1850431ee6a1d67]

### Gazebo 1.8.4 (2013-06-03)
* Fix UDRF to SDF converter so that URDF gazebo extensions are applied to all collisions in a link.[https://bitbucket.org/osrf/gazebo/pull-request/579]
* Prevent transport layer from locking when a gzclient connects to a gzserver over a connection with high latency.[https://bitbucket.org/osrf/gazebo/pull-request/572]
* Improve performance and fix uninitialized conditional jumps.[https://bitbucket.org/osrf/gazebo/pull-request/571]

### Gazebo 1.8.3 (2013-06-03)
* Fix for gzlog hanging when gzserver is not present or not responsive[https://bitbucket.org/osrf/gazebo/pull-request/577]
* Fix occasional segfault when generating log files[https://bitbucket.org/osrf/gazebo/pull-request/575]
* Performance improvement to ODE[https://bitbucket.org/osrf/gazebo/pull-request/556]
* Fix node initialization[https://bitbucket.org/osrf/gazebo/pull-request/570]
* Fix GPU laser Hz rate reduction when sensor moved away from world origin[https://bitbucket.org/osrf/gazebo/pull-request/566]
* Fix incorrect lighting in camera sensors when GPU laser is subscribe to[https://bitbucket.org/osrf/gazebo/pull-request/563]

### Gazebo 1.8.2 (2013-05-28)
* ODE performance improvements[https://bitbucket.org/osrf/gazebo/pull-request/535][https://bitbucket.org/osrf/gazebo/pull-request/537]
* Fixed tests[https://bitbucket.org/osrf/gazebo/pull-request/538][https://bitbucket.org/osrf/gazebo/pull-request/541][https://bitbucket.org/osrf/gazebo/pull-request/542]
* Fixed sinking vehicle bug[https://bitbucket.org/osrf/drcsim/issue/300] in pull-request[https://bitbucket.org/osrf/gazebo/pull-request/538]
* Fix GPU sensor throttling[https://bitbucket.org/osrf/gazebo/pull-request/536]
* Reduce string comparisons for better performance[https://bitbucket.org/osrf/gazebo/pull-request/546]
* Contact manager performance improvements[https://bitbucket.org/osrf/gazebo/pull-request/543]
* Transport performance improvements[https://bitbucket.org/osrf/gazebo/pull-request/548]
* Reduce friction noise[https://bitbucket.org/osrf/gazebo/pull-request/545]

### Gazebo 1.8.1 (2013-05-22)
* Please note that 1.8.1 contains a bug[https://bitbucket.org/osrf/drcsim/issue/300] that causes interpenetration between objects in resting contact to grow slowly.  Please update to 1.8.2 for the patch.
* Added warm starting[https://bitbucket.org/osrf/gazebo/pull-request/529]
* Reduced console output[https://bitbucket.org/osrf/gazebo/pull-request/533]
* Improved off screen rendering performance[https://bitbucket.org/osrf/gazebo/pull-request/530]
* Performance improvements [https://bitbucket.org/osrf/gazebo/pull-request/535] [https://bitbucket.org/osrf/gazebo/pull-request/537]

### Gazebo 1.8.0 (2013-05-17)
* Fixed slider axis [https://bitbucket.org/osrf/gazebo/pull-request/527]
* Fixed heightmap shadows [https://bitbucket.org/osrf/gazebo/pull-request/525]
* Fixed model and canonical link pose [https://bitbucket.org/osrf/gazebo/pull-request/519]
* Fixed OSX message header[https://bitbucket.org/osrf/gazebo/pull-request/524]
* Added zlib compression for logging [https://bitbucket.org/osrf/gazebo/pull-request/515]
* Allow clouds to be disabled in cameras [https://bitbucket.org/osrf/gazebo/pull-request/507]
* Camera rendering performance [https://bitbucket.org/osrf/gazebo/pull-request/528]


## Gazebo 1.7

### Gazebo 1.7.3 (2013-05-08)
* Fixed log cleanup (again) [https://bitbucket.org/osrf/gazebo/pull-request/511/fix-log-cleanup-logic]

### Gazebo 1.7.2 (2013-05-07)
* Fixed log cleanup [https://bitbucket.org/osrf/gazebo/pull-request/506/fix-gzlog-stop-command-line]
* Minor documentation fix [https://bitbucket.org/osrf/gazebo/pull-request/488/minor-documentation-fix]

### Gazebo 1.7.1 (2013-04-19)
* Fixed tests
* IMU sensor receives time stamped data from links
* Fix saving image frames [https://bitbucket.org/osrf/gazebo/pull-request/466/fix-saving-frames/diff]
* Wireframe rendering in GUI [https://bitbucket.org/osrf/gazebo/pull-request/414/allow-rendering-of-models-in-wireframe]
* Improved logging performance [https://bitbucket.org/osrf/gazebo/pull-request/457/improvements-to-gzlog-filter-and-logging]
* Viscous mud model [https://bitbucket.org/osrf/gazebo/pull-request/448/mud-plugin/diff]

## Gazebo 1.6

### Gazebo 1.6.3 (2013-04-15)
* Fixed a [critical SDF bug](https://bitbucket.org/osrf/gazebo/pull-request/451)
* Fixed a [laser offset bug](https://bitbucket.org/osrf/gazebo/pull-request/449)

### Gazebo 1.6.2 (2013-04-14)
* Fix for fdir1 physics property [https://bitbucket.org/osrf/gazebo/pull-request/429/fixes-to-treat-fdir1-better-1-rotate-into/diff]
* Fix for force torque sensor [https://bitbucket.org/osrf/gazebo/pull-request/447]
* SDF documentation fix [https://bitbucket.org/osrf/gazebo/issue/494/joint-axis-reference-frame-doesnt-match]

### Gazebo 1.6.1 (2013-04-05)
* Switch default build type to Release.

### Gazebo 1.6.0 (2013-04-05)
* Improvements to inertia in rubble pile
* Various Bullet integration advances.
* Noise models for ray, camera, and imu sensors.
* SDF 1.4, which accommodates more physics engine parameters and also some sensor noise models.
* Initial support for making movies from within Gazebo.
* Many performance improvements.
* Many bug fixes.
* Progress toward to building on OS X.

## Gazebo 1.5

### Gazebo 1.5.0 (2013-03-11)
* Partial integration of Bullet
  * Includes: cubes, spheres, cylinders, planes, meshes, revolute joints, ray sensors
* GUI Interface for log writing.
* Threaded sensors.
* Multi-camera sensor.

* Fixed the following issues:
 * [https://bitbucket.org/osrf/gazebo/issue/236 Issue #236]
 * [https://bitbucket.org/osrf/gazebo/issue/507 Issue #507]
 * [https://bitbucket.org/osrf/gazebo/issue/530 Issue #530]
 * [https://bitbucket.org/osrf/gazebo/issue/279 Issue #279]
 * [https://bitbucket.org/osrf/gazebo/issue/529 Issue #529]
 * [https://bitbucket.org/osrf/gazebo/issue/239 Issue #239]
 * [https://bitbucket.org/osrf/gazebo/issue/5 Issue #5]

## Gazebo 1.4

### Gazebo 1.4.0 (2013-02-01)
* New Features:
 * GUI elements to display messages from the server.
 * Multi-floor building editor and creator.
 * Improved sensor visualizations.
 * Improved mouse interactions

* Fixed the following issues:
 * [https://bitbucket.org/osrf/gazebo/issue/16 Issue #16]
 * [https://bitbucket.org/osrf/gazebo/issue/142 Issue #142]
 * [https://bitbucket.org/osrf/gazebo/issue/229 Issue #229]
 * [https://bitbucket.org/osrf/gazebo/issue/277 Issue #277]
 * [https://bitbucket.org/osrf/gazebo/issue/291 Issue #291]
 * [https://bitbucket.org/osrf/gazebo/issue/310 Issue #310]
 * [https://bitbucket.org/osrf/gazebo/issue/320 Issue #320]
 * [https://bitbucket.org/osrf/gazebo/issue/329 Issue #329]
 * [https://bitbucket.org/osrf/gazebo/issue/333 Issue #333]
 * [https://bitbucket.org/osrf/gazebo/issue/334 Issue #334]
 * [https://bitbucket.org/osrf/gazebo/issue/335 Issue #335]
 * [https://bitbucket.org/osrf/gazebo/issue/341 Issue #341]
 * [https://bitbucket.org/osrf/gazebo/issue/350 Issue #350]
 * [https://bitbucket.org/osrf/gazebo/issue/384 Issue #384]
 * [https://bitbucket.org/osrf/gazebo/issue/431 Issue #431]
 * [https://bitbucket.org/osrf/gazebo/issue/433 Issue #433]
 * [https://bitbucket.org/osrf/gazebo/issue/453 Issue #453]
 * [https://bitbucket.org/osrf/gazebo/issue/456 Issue #456]
 * [https://bitbucket.org/osrf/gazebo/issue/457 Issue #457]
 * [https://bitbucket.org/osrf/gazebo/issue/459 Issue #459]

## Gazebo 1.3

### Gazebo 1.3.1 (2012-12-14)
* Fixed the following issues:
 * [https://bitbucket.org/osrf/gazebo/issue/297 Issue #297]
* Other bugs fixed:
 * [https://bitbucket.org/osrf/gazebo/pull-request/164/ Fix light bounding box to disable properly when deselected]
 * [https://bitbucket.org/osrf/gazebo/pull-request/169/ Determine correct local IP address, to make remote clients work properly]
 * Various test fixes

### Gazebo 1.3.0 (2012-12-03)
* Fixed the following issues:
 * [https://bitbucket.org/osrf/gazebo/issue/233 Issue #233]
 * [https://bitbucket.org/osrf/gazebo/issue/238 Issue #238]
 * [https://bitbucket.org/osrf/gazebo/issue/2 Issue #2]
 * [https://bitbucket.org/osrf/gazebo/issue/95 Issue #95]
 * [https://bitbucket.org/osrf/gazebo/issue/97 Issue #97]
 * [https://bitbucket.org/osrf/gazebo/issue/90 Issue #90]
 * [https://bitbucket.org/osrf/gazebo/issue/253 Issue #253]
 * [https://bitbucket.org/osrf/gazebo/issue/163 Issue #163]
 * [https://bitbucket.org/osrf/gazebo/issue/91 Issue #91]
 * [https://bitbucket.org/osrf/gazebo/issue/245 Issue #245]
 * [https://bitbucket.org/osrf/gazebo/issue/242 Issue #242]
 * [https://bitbucket.org/osrf/gazebo/issue/156 Issue #156]
 * [https://bitbucket.org/osrf/gazebo/issue/78 Issue #78]
 * [https://bitbucket.org/osrf/gazebo/issue/36 Issue #36]
 * [https://bitbucket.org/osrf/gazebo/issue/104 Issue #104]
 * [https://bitbucket.org/osrf/gazebo/issue/249 Issue #249]
 * [https://bitbucket.org/osrf/gazebo/issue/244 Issue #244]
 * [https://bitbucket.org/osrf/gazebo/issue/36 Issue #36]

* New features:
 * Default camera view changed to look down at the origin from a height of 2 meters at location (5, -5, 2).
 * Record state data using the '-r' command line option, playback recorded state data using the '-p' command line option
 * Adjust placement of lights using the mouse.
 * Reduced the startup time.
 * Added visual reference for GUI mouse movements.
 * SDF version 1.3 released (changes from 1.2 listed below):
     - added `name` to `<camera name="cam_name"/>`
     - added `pose` to `<camera><pose>...</pose></camera>`
     - removed `filename` from `<mesh><filename>...</filename><mesh>`, use uri only.
     - recovered `provide_feedback` under `<joint>`, allowing calling `physics::Joint::GetForceTorque` in plugins.
     - added `imu` under `<sensor>`.

## Gazebo 1.2

### Gazebo 1.2.6 (2012-11-08)
* Fixed a transport issue with the GUI. Fixed saving the world via the GUI. Added more documentation. ([https://bitbucket.org/osrf/gazebo/pull-request/43/fixed-a-transport-issue-with-the-gui-fixed/diff pull request #43])
* Clean up mutex usage. ([https://bitbucket.org/osrf/gazebo/pull-request/54/fix-mutex-in-modellistwidget-using-boost/diff pull request #54])
* Fix OGRE path determination ([https://bitbucket.org/osrf/gazebo/pull-request/58/fix-ogre-paths-so-this-also-works-with/diff pull request #58], [https://bitbucket.org/osrf/gazebo/pull-request/68/fix-ogre-plugindir-determination/diff pull request #68])
* Fixed a couple of crashes and model selection/dragging problems ([https://bitbucket.org/osrf/gazebo/pull-request/59/fixed-a-couple-of-crashes-and-model/diff pull request #59])

### Gazebo 1.2.5 (2012-10-22)
* Step increment update while paused fixed ([https://bitbucket.org/osrf/gazebo/pull-request/45/fix-proper-world-stepinc-count-we-were/diff pull request #45])
* Actually call plugin destructors on shutdown ([https://bitbucket.org/osrf/gazebo/pull-request/51/fixed-a-bug-which-prevent-a-plugin/diff pull request #51])
* Don't crash on bad SDF input ([https://bitbucket.org/osrf/gazebo/pull-request/52/fixed-loading-of-bad-sdf-files/diff pull request #52])
* Fix cleanup of ray sensors on model deletion ([https://bitbucket.org/osrf/gazebo/pull-request/53/deleting-a-model-with-a-ray-sensor-did/diff pull request #53])
* Fix loading / deletion of improperly specified models ([https://bitbucket.org/osrf/gazebo/pull-request/56/catch-when-loading-bad-models-joint/diff pull request #56])

### Gazebo 1.2.4 (10-19-2012:08:00:52)
*  Style fixes ([https://bitbucket.org/osrf/gazebo/pull-request/30/style-fixes/diff pull request #30]).
*  Fix joint position control ([https://bitbucket.org/osrf/gazebo/pull-request/49/fixed-position-joint-control/diff pull request #49])

### Gazebo 1.2.3 (10-16-2012:18:39:54)
*  Disabled selection highlighting due to bug ([https://bitbucket.org/osrf/gazebo/pull-request/44/disabled-selection-highlighting-fixed/diff pull request #44]).
*  Fixed saving a world via the GUI.

### Gazebo 1.2.2 (10-16-2012:15:12:22)
*  Skip search for system install of libccd, use version inside gazebo ([https://bitbucket.org/osrf/gazebo/pull-request/39/skip-search-for-system-install-of-libccd/diff pull request #39]).
*  Fixed sensor initialization race condition ([https://bitbucket.org/osrf/gazebo/pull-request/42/fix-sensor-initializaiton-race-condition pull request #42]).

### Gazebo 1.2.1 (10-15-2012:21:32:55)
*  Properly removed projectors attached to deleted models ([https://bitbucket.org/osrf/gazebo/pull-request/37/remove-projectors-that-are-attached-to/diff pull request #37]).
*  Fix model plugin loading bug ([https://bitbucket.org/osrf/gazebo/pull-request/31/moving-bool-first-in-model-and-world pull request #31]).
*  Fix light insertion and visualization of models prior to insertion ([https://bitbucket.org/osrf/gazebo/pull-request/35/fixed-light-insertion-and-visualization-of/diff pull request #35]).
*  Fixed GUI manipulation of static objects ([https://bitbucket.org/osrf/gazebo/issue/63/moving-static-objects-does-not-move-the issue #63] [https://bitbucket.org/osrf/gazebo/pull-request/38/issue-63-bug-patch-moving-static-objects/diff pull request #38]).
*  Fixed GUI selection bug ([https://bitbucket.org/osrf/gazebo/pull-request/40/fixed-selection-of-multiple-objects-at/diff pull request #40])

### Gazebo 1.2.0 (10-04-2012:20:01:20)
*  Updated GUI: new style, improved mouse controls, and removal of non-functional items.
*  Model database: An online repository of models.
*  Numerous bug fixes
*  APT repository hosted at [http://osrfoundation.org OSRF]
*  Improved process control prevents zombie processes

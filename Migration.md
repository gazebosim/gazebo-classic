# Note on deprecations
A tick-tock release cycle allows easy migration to new software versions.
Obsolete Gazebo code is marked as deprecated for one major release.
Deprecated code produces compile-time warnings. These warning serve as
notification to users that their code should be upgraded. The next major
release will remove the deprecated code.

## Gazebo 11.14 to 11.15

### Modifications

The `gz world` tool now publishes a WorldControl message using gz-transport,
and the `physics::World` class and other subscribers now subscribe to both
`gz-transport` (ZeroMQ) and `gazebo_transport` (boost asio) topics.

## Gazebo 11.2 to 11.3

### Modifications

Updated the version of TinyOBJLoader from 1.0.0 to 2.0.0rc8.
See the changelog at https://github.com/osrf/gazebo/blob/gazebo11/deps/tinyobjloader/tiny_obj_loader.h

## Gazebo 11.1 to 11.3

### Modifications

The way Gazebo handles relative paths in SDF files has changed:

* Until `11.1.0`: relative paths could only be resolved against environment variables `GAZEBO_MODEL_PATH` / `GAZEBO_RESOURCE_PATH`.
* On `11.2.0`: relative paths loaded from SDF files can only be resolved against the SDF file - the previous use case was broken by mistake.
* From `11.3.0`: relative paths are first resolved against the SDF file, and if that fails, fallback to the environment variables.

## Gazebo 10.x to 11.0

### Build system

New versions in mandatory dependencies: `ign-transport8`, `ign-msgs5`, `ign-math6`, `sdformat9`.
New mandatory dependencies: `ign-fuel-tools4`, `ign-common3`, `ign-common3-graphics`, `ign-common3-profiler`.

### Additions

1. **gazebo/common/SdfFrameSemantics.hh**
    + `ignition::math::Pose3d` resolveSdfPose(const sdf::SemanticPose &, const std::string &)
    + `void` convertPosesToSdf16(const sdf::ElementPtr &)

1. **gazebo/physics/Base.hh**
    + public: `ignition::math::Pose3d` SDFPoseRelativeToParent() const
    + public: virtual `std::optional<sdf::SemanticPose>` SDFSemanticPose() const

1. **gazebo/physics/Collision.hh**
    + public: virtual `std::optional<sdf::SemanticPose>` SDFSemanticPose() const override

1. **gazebo/physics/Joint.hh**
    + public: const `sdf::Joint *` GetSDFDom() const
    + public: `ignition::math::Vector3d` ResolveAxisXyz(const unsigned int, const std::string &) const
    + public: virtual `std::optional<sdf::SemanticPose>` SDFSemanticPose() const override

1. **gazebo/physics/Light.hh**
    + public: virtual `std::optional<sdf::SemanticPose>` SDFSemanticPose() const override

1. **gazebo/physics/Link.hh**
      public: const `sdf::Link` *GetSDFDom() const;
    + public: virtual `std::optional<sdf::SemanticPose>` SDFSemanticPose() const override

1. **gazebo/physics/Model.hh**
    + public: const `sdf::Model *` GetSDFDom() const
    + public: virtual `std::optional<sdf::SemanticPose>` SDFSemanticPose() const override

1. **gazebo/physics/PhysicsEngine.hh**
    + public: template <typename T>
      static T any\_cast<T>(const boost::any &)

1. **gazebo/physics/PhysicsTypes.hh**
    + Defined function signature for API to update the poses of objects in a named scene
    + using UpdateScenePosesFunc =
      std::function<void(const std::string &, const msgs::PosesStamped &)>

1. **gazebo/rendering/RenderingIface.hh**
    + API for directly updating the poses of objects in a named scene
    + void `update_scene_poses`(const std::string &, const msgs::PosesStamped &)

1. **gazebo/rendering/Scene.hh**
    + API for directly updating the poses of objects in a scene
    + void UpdatePoses(const msgs::PosesStamped &)

1. **gazebo/physics/World.hh**
    + public: const `sdf::World` GetSDFDom() const

### Modifications

1. **gazebo/rendering/JointVisual.hh**
    + ***Deprecation:*** ArrowVisualPtr CreateAxis(const ignition::math::Vector3d &_axis, const bool _useParentFrame, const msgs::Joint::Type &_type);
    + ***Replacement:*** ArrowVisualPtr CreateAxis(const ignition::math::Vector3d &_axis, const std::string &_xyzExpressedIn, const msgs::Joint::Type &_type);
    + ***Deprecation:*** void UpdateAxis(ArrowVisualPtr _arrowVisual, const ignition::math::Vector3d &_axis, const bool _useParentFrame, const msgs::Joint::Type &_type);
    + ***Replacement:*** void UpdateAxis(ArrowVisualPtr _arrowVisual, const ignition::math::Vector3d &_axis, const std::string &_xyzExpressedIn, const msgs::Joint::Type &_type);

1. All instances of `ignition::math::Box` in the API are changed to `ignition::math::AxisAlignedBox`
   to match the changes in ignition-math6.

1. **gazebo/physics/JointController.hh**
   Use new optional fields in `ignition::msgs::JointCmd` and
   `ignition::msgs::PID` since the ign-msgs5 proto file uses `proto3`,
   which doesn't allow optional fields and breaks existing functionality.

1. **gazebo/physics/PresetManager.hh**
   The PresetManager stores data internally with a map of `boost::any` values
   and with the switch to sdformat9, the value may be stored as a `std::any`
   within a `boost::any`, making it more complex to cast to a specific type.
   This happens because the PresetManager stores the output of
   `sdf::Element::GetAny` as boost::any values in its parameterMap and
   calls `PhysicsEngine::SetParam` with these values.
   Prior to libsdformat9, the GetAny function returned `boost::any`, but it now
   returns `std::any`. The `gazebo::physics::PhysicsEngine::any_cast` helper
   is provided to first check if a `boost::any` value contains a std::any,
   and if so, perform a `std::any_cast<T>`.
   Otherwise, it returns `boost::any_cast<T>`.
   This `any_cast` helper should be used with `boost::any` values provided by
   `Preset::GetParam`, `PresetManager::GetProfileParam`, and
   `PresetManager::GetCurrentProfileParam`.

1. **gazebo/physics/PhysicsIface.hh**
    + A `std::function` argument is added to the API's for initializing worlds.
      This argument can be called to directly update the poses of
      objects in a rendering Scene. See **PhysicsTypes.hh** for the
      definition of `UpdateScenePosesFunc`.
    + ***Deprecation:*** void `physics::init_world`(WorldPtr)
    + ***Replacement:*** void `physics::init_world`(WorldPtr, UpdateScenePosesFunc)
    + ***Deprecation:*** void `physics::init_worlds`()
    + ***Replacement:*** void `physics::init_worlds`(UpdateScenePosesFunc)

1. **gazebo/physics/World.hh**
    + A `std::function` argument is added to the API's for initializing worlds.
      This argument can be called to directly update the poses of
      objects in a rendering Scene. See **PhysicsTypes.hh** for the
      definition of `UpdateScenePosesFunc`.
    + ***Deprecation:*** void World::Init()
    + ***Replacement:*** void World::Init(UpdateScenePosesFunc)

1. **gazebo/rendering/MarkerManager.cc**
    The `/marker` ignition transport service allows specifying the `id` field
    of a marker to be created. If the `id` field is not specified, a random,
    valid `id` is generated as a convenience for the user.
    Due to the upgrade to `ign-msgs5`, which uses `proto3` syntax, an `id`
    of `0` is indistinguishable from an unset `id`.
    As such, an `id` of `0` will now trigger a random `id` to be generated,
    and non-zero `id` values should be used instead.

### Deletions

1. **gazebo/physics/Joint.hh**
    + ***Removed:*** protected: bool axisParentModelFrame[]
    + ***Replacement:*** protected: std::string axisExpressedIn[]

## Gazebo 9.x to 10.x

### Additions
1. **gazebo/physics/dart/DARTLink.hh**
    + public: void AddSlaveBodyNode(dart::dynamics::BodyNode *_dtBodyNode);
    + public: bool RemoveSlaveBodyNode(dart::dynamics::BodyNode *_dtBodyNode);
1. **gazebo/physics/dart/DARTJoint.hh**
    + public: virtual void SetName(const std::string &_name);
1. **gazebo/physics/dart/DARTPhysics.hh**
    + public: std::string GetSolverType() const;
    + public: void SetSolverType(const std::string &_type);

### Modifications

1. **gazebo/physics/Model.hh**
    + Made `CreateJoint` virtual
    + Made `RemoveJoint` virtual
1. WindPlugin now requires setting `<force_approximation_scaling_factor>` to
   enable mass based force approximation. Set to 1.0 for original behavior.
1. **gazebo/transport/TransportIface.hh**
    + ***Removed:*** boost::shared_ptr<msgs::Response> request(const std::string &_worldName, const std::string &_request, const std::string &_data = "");
    + ***Replacement:*** boost::shared_ptr<msgs::Response> request(const std::string &_worldName, const std::string &_request, const std::string &_data = "", const common::Time &_timeout = -1);
    + ***Note:*** Added extra argument `_timeout`
1. **gazebo/gui/qt_test.h**
    + ***Removed:*** The whole header file. Note that it also won't be included into `gazebo/gui/gui.hh`.
    + ***Replacement:*** Include `<QtTest/QtTest>` instead.

## Gazebo 9.15 to 9.16

### Modifications

Updated the version of TinyOBJLoader from 1.0.0 to 2.0.0rc8.
See the changelog at https://github.com/osrf/gazebo/blob/gazebo9/deps/tinyobjloader/tiny_obj_loader.h

## Gazebo 8.4 to 9.x

### Models with duplicate names will not be inserted

1. Prior to gazebo 8.4, multiple models with the same name could be inserted
   into a world and simulated, though they were not fully functional.
   Now models with duplicate names are not allowed to be inserted.

## Gazebo 8.x to 9.x

### Build system

New versions in mandatory dependencies: `ign-transport4`, `ign-msgs1`, `ign-math4`, `sdformat6`.
New optional dependencies: `ign-fuel-tools`,`ign-common1`

### -g command line argument to load plugins in gzclient

1. During the gazebo 8.x series the `-g` was used to load System plugins in the
   client side instead of GUI plugins. In gazebo 9.x the `-g` loads GUI
   plugins. The `--gui-client-plugin` argument introduced in gazebo 8.2 load GUI
   plugins and will remain the exactly the same.

### Modifications

1. ***Modified:*** Many constructors with 1 argument were marked as explicit.
1. **gazebo/gui/JointControlWidget.hh**
    + ***Removed:*** gazebo::transport::Publisher for topic(s) `~/.../joint_cmd`
    + ***Replacement:*** ignition::transport::Publisher for topic(s) `/.../joint_cmd`
1. **gazebo/physics/**
    + ***Modified:*** Many constructors with 1 argument in physics classes were marked as explicit.
1. **gazebo/physics/Link.hh**
    + ***Deprecation:*** void SetLinearAccel(const ignition::math::Vector3d &_accel);
    + ***Replacement:***  None. Doesn't do anything, acceleration should be achieved by setting force.
    + ***Deprecation:***  void SetAngularAccel(const ignition::math::Vector3d &_accel);
    + ***Replacement:***  None. Doesn't do anything, acceleration should be achieved by setting force.
1. **gazebo/physics/Model.hh**
    + ***Deprecation:*** void SetLinearAccel(const ignition::math::Vector3d &_vel);
    + ***Replacement:*** None. Calls now deprecated SetLinearAccel() on all links.
    + ***Deprecation:*** void SetAngularAccel(const ignition::math::Vector3d &_vel);
    + ***Replacement:*** None. Calls now deprecated SetAngularAccel() on all links.
1. **gazebo/sensors/CameraSensor.cc**
    + ***Modified:*** Ignition transport topic now uses ignition::msgs::Image instead of ignition::msgs::ImageStamped
1. **gazebo/sensors/WideAngleCameraSensor.cc**
    + ***Modified:*** Ignition transport topic now uses ignition::msgs::Image instead of ignition::msgs::ImageStamped
1. **gazebo/gui/ConfigWidget.hh**
    + ColorValueChanged signal now uses ignition::math::Color instead of gazebo::common::Color
1. **gazebo/common/Material.hh**
    + Changed `protected: Color ambient;` to `protected: ignition::math::Color ambient;`
    + Changed `protected: Color diffuse;` to `protected: ignition::math::Color diffuse;`
    + Changed `protected: Color specular;` to `protected: ignition::math::Color specular;`
    + Changed `protected: Color emissive;` to `protected: ignition::math::Color emissive;`
1. `BUILD_TYPE_*` macros renamed to `GAZEBO_BUILD_TYPE_*`

### Deprecations

1. **gazebo/physics/JointController.hh**
    + ***Deprecation:*** private: void OnJointCmd(ConstJointCmdPtr &_msg);
    + ***Replacement:*** private: void OnJointCommand(const ignition::msgs::JointCmd &_msg);
1. **gazebo/physics/Link.hh**
    + ***Deprecation:*** void SetLinearAccel(const ignition::math::Vector3d &_accel);
    + ***Replacement:***  None. Doesn't do anything, acceleration should be achieved by setting force.
    + ***Deprecation:***  void SetAngularAccel(const ignition::math::Vector3d &_accel);
    + ***Replacement:***  None. Doesn't do anything, acceleration should be achieved by setting force.
1. **gazebo/physics/Model.hh**
    + ***Deprecation:*** void SetLinearAccel(const ignition::math::Vector3d &_vel);
    + ***Replacement:*** None. Calls now deprecated SetLinearAccel() on all links.
    + ***Deprecation:*** void SetAngularAccel(const ignition::math::Vector3d &_vel);
    + ***Replacement:*** None. Calls now deprecated SetAngularAccel() on all links.
1. **gazebo/rendering/GpuLaser.hh**
    + ***Deprecation:*** const float\* LaserData() const
    + ***Replacement:*** Call GpuLaser::DataIter LaserDataBegin() const
        iterate until reaching GpuLaser::DataIter LaserDataEnd() const
1. **gazebo/rendering/Conversions.hh**
    + ***Deprecation:*** Ogre::ColourValue Convert(const common::Color &_clr)
    + ***Replacement:*** Ogre::ColourValue Convert(const ignition::math::Color &_clr)
    + common::Color Convert(const Ogre::ColourValue &_clr) now returns ignition::math::Color
1. **gazebo/rendering/Material.hh**
    + ***Deprecation:*** bool GetMaterialAsColor(const std::string &_materialName, common::Color &_ambient, common::Color &_diffuse, common::Color &_specular, common::Color &_emissive)
    + ***Replacement:*** bool MaterialAsColor(const std::string &_materialName, ignition::math::Color &_ambient, ignition::math::Color &_diffuse, ignition::math::Color &_specular, ignition::math::Color &_emissive)
1. **gazebo/rendering/Visual.hh**
    + ***Deprecation:*** void SetAmbient(const common::Color &_color, const bool _cascade = true)
    + ***Replacement:*** void SetAmbient(const ignition::math::Color &_color, const bool _cascade = true)
    + ***Deprecation:*** void SetDiffuse(const common::Color &_color, const bool _cascade = true)
    + ***Replacement:*** void SetDiffuse(const ignition::math::Color &_color, const bool _cascade = true)
    + ***Deprecation:*** void SetSpecular(const common::Color &_color, const bool _cascade = true)
    + ***Replacement:*** void SetSpecular(const ignition::math::Color &_color, const bool _cascade = true)
    + ***Deprecation:*** void SetEmissive(const common::Color &_color, const bool _cascade = true)
    + ***Replacement:*** void SetEmissive(const ignition::math::Color &_color, const bool _cascade = true)
    + ***Deprecation:*** common::Color GetAmbient() const
    + ***Replacement:*** ignition::math::Color Ambient() const
    + ***Deprecation:*** common::Color GetDiffuse() const
    + ***Replacement:*** ignition::math::Color Diffuse()
    + ***Deprecation:*** common::Color GetSpecular() const
    + ***Replacement:*** ignition::math::Color Specular() const
    + ***Deprecation:*** common::Color GetEmissive() const
    + ***Replacement:*** ignition::math::Color Emissive()
    + ***Deprecation:*** void SetRibbonTrail(bool _value, const common::Color &_initialColor, const common::Color &_changeColor)
    + ***Replacement:*** void SetRibbonTrail(bool _value, const ignition::math::Color &_initialColor, const ignition::math::Color &_changeColor)
1. **gazebo/rendering/LaserVisual.hh**
    + virtual void SetEmissive(const common::Color &_color, const bool _cascade = true) now accepts ignition::math::Color
1. **gazebo/rendering/Scene.hh**
    + ***Deprecation:*** void SetAmbientColor(const common::Color &_color)
    + ***Replacement:*** void SetAmbientColor(const ignition::math::Color &_color)
    + ***Deprecation:*** void SetBackgroundColor(const common::Color &_color)
    + ***Replacement:*** void SetBackgroundColor(const ignition::math::Color &_color)
    + ***Deprecation:*** void CreateGrid(const uint32_t _cellCount, const float _cellLength, const float _lineWidth, const common::Color &_color)
    + ***Replacement:*** void CreateGrid(const uint32_t _cellCount, const float _cellLength, const ignition::math::Color &_color)
    + ***Deprecation:*** void SetFog(const std::string &_type, const common::Color &_color, const double _density, const double _start, const double _end)
    + ***Replacement:*** void SetFog(const std::string &_type, const ignition::math::Color &_color, const double _density, const double _start, const double _end)
    + common::Color AmbientColor() const now returns ignition::math::Color
    + common::Color BackgroundColor() const now returns ignition::math::Color
    + ***Deprecation:*** LightPtr GetLight(const uint32_t _index) const;
    + ***Replacement:*** LightPtr LightByIndex(const uint32_t _index) const;
    + ***Deprecation:*** LightPtr GetLight(const std::string &_name) const;
    + ***Replacement:*** LightPtr LightByName(const std::string &_name) const;
1. **gazebo/rendering/Camera.hh**
    + ***Deprecation:*** virtual bool SetBackgroundColor(const common::Color &_color)
    + ***Replacement:*** virtual bool SetBackgroundColor(const ignition::math::Color &_color)
1. **gazebo/rendering/WideAngleCamera.hh**
    + bool SetBackgroundColor(const common::Color &_color) now accepts ignition::math::Color
1. **gazebo/rendering/Grid.hh**
    + ***Deprecation:*** Grid(Scene *_scene, const uint32_t _cellCount, const float _cellLength, const float _lineWidth, const common::Color &_color)
    + ***Replacement:*** Grid(Scene *_scene, const uint32_t _cellCount, const float _cellLength, const ignition::math::Color &_color)
    + ***Deprecation:*** void SetColor(const common::Color &_color)
    + ***Replacement:*** void SetColor(const ignition::math::Color &_color)
    + ***Deprecation:*** void SetLineWidth(const float _width)
    + ***Replacement:*** None, grid lines are always 1px wide.
    + ***Deprecation:*** float LineWidth() const
    + ***Replacement:*** None, grid lines are always 1px wide.
    + common::Color Color() const now returns ignition::math::Color
1. **gazebo/rendering/Light.hh**
    + ***Deprecation:*** void SetDiffuseColor(const common::Color &_color)
    + ***Replacement:*** void SetDiffuseColor(const ignition::math::Color &_color)
    + ***Deprecation:*** void SetSpecularColor(const common::Color &_color)
    + ***Replacement:*** void SetSpecularColor(const ignition::math::Color &_color)
    + common::Color DiffuseColor() const now returns ignition::math::Color DiffuseColor() const
    + common::Color SpecularColor() const now returns ignition::math::Color SpecularColor() const
1. **gazebo/rendering/DynamicLines.hh**
    + ***Deprecation:*** void AddPoint(const ignition::math::Vector3d &_pt, const common::Color &_color)
    + ***Replacement:*** void AddPoint(const ignition::math::Vector3d &_pt, const ignition::math::Color &_color = ignition::math::Color::White)
    + ***Deprecation:*** void AddPoint(double _x, double _y, double _z, const common::Color &_color)
    + ***Replacement:*** void AddPoint(const double _x, const double _y, const double _z, const ignition::math::Color &_color = ignition::math::Color::White)
    + ***Deprecation:*** void SetColor(unsigned int _index, const common::Color &_color)
    + ***Replacement:*** void SetColor(const unsigned int _index, const ignition::math::Color &_color)
1. **gazebo/rendering/MovableText.hh**
    + ***Deprecation:*** void Load(const std::string &_name, const std::string &_text, const std::string &_fontName, float _charHeight, const common::Color &_color)
    + ***Replacement:*** void Load(const std::string &_name, const std::string &_text, const std::string &_fontName = "Arial", float _charHeight = 1.0, const ignition::math::Color &_color = ignition::math::Color::White)
    + ***Deprecation:*** const std::string &GetFont() const
    + ***Replacement:*** const std::string &FontName() const
    + ***Deprecation:*** const std::string &GetText() const
    + ***Replacement:*** const std::string &Text() const
    + ***Deprecation:*** void SetColor(const common::Color &_color)
    + ***Replacement:*** void SetColor(const ignition::math::Color &_color)
    + ***Deprecation:*** const common::Color GetColor() const
    + ***Replacement:*** const ignition::math::Color &Color() const
    + ***Deprecation:*** float GetCharHeight() const
    + ***Replacement:*** float CharHeight() const
    + ***Deprecation:*** float GetSpaceWidth() const
    + ***Replacement:*** float SpaceWidth() const
    + ***Deprecation:*** float GetBaseline() const
    + ***Replacement:*** float Baseline() const
    + ***Deprecation:*** bool GetShowOnTop() const
    + ***Replacement:*** bool ShowOnTop() const
    + ***Deprecation:*** void _setupGeometry()
    + ***Replacement:*** void SetupGeometry()
    + ***Deprecation:*** void _updateColors()
    + ***Replacement:*** void UpdateColors()
1. **gazebo/gui/building/BuildingModelManip.hh**
    + BuildingModelManip::Color() now returns ignition::math::Color()
1. **gazebo/msgs/msgs.hh**
    + ***Deprecation:*** msgs::Any ConvertAny(const common::Color &_c)
    + ***Replacement:*** msgs::Any ConvertAny(const ignition::math::Color &_c)
    + ***Deprecation:*** msgs::Color Convert(const common::Color &_c)
    + ***Replacement:*** msgs::Color Convert(const ignition::math::Color &_c)
    + ***Deprecation:*** void Set(msgs::Color *_c, const common::Color &_v)
    + ***Replacement:*** void Set(msgs::Color *_c, const ignition::math::Color &_v)
    + Convert(const msgs::Color &_c) now returns ignition::math::Color()
1. **gazebo/gui/Conversions.hh**
    + ***Deprecation:*** QColor Convert(const common::Color &_clr)
    + ***Replacement:*** QColor Convert(const ignition::math::Color &_clr)
    + common::Color Convert(const QColor &_clr) now returns ignition::math::Color
1. **gazebo/gui/ConfigWidget.hh**
    + ***Deprecation:*** bool SetColorWidgetValue(const std::string &_name, const common::Color &_value)
    + ***Replacement:*** bool SetColorWidgetValue(const std::string &_name, const ignition::math::Color &_value)
    + common::Color ColorWidgetValue(const std::string &_name) const now returns ignition::math::Color
1. **gazebo/common/Material.hh**
    + ***Deprecation:*** explicit Material(const Color &_clr)
    + ***Replacement:*** explicit Material(const ignition::math::Color &_clr)
    + ***Deprecation:*** void SetAmbient(const Color &_clr)
    + ***Replacement:*** void SetAmbient(const ignition::math::Color &_clr)
    + ***Deprecation:*** void SetDiffuse(const Color &_clr)
    + ***Replacement:*** void SetDiffuse(const ignition::math::Color &_clr)
    + ***Deprecation:*** void SetSpecular(const Color &_clr)
    + ***Replacement:*** void SetSpecular(const ignition::math::Color &_clr)
    + ***Deprecation:*** void SetEmissive(const Color &_clr)
    + ***Replacement:*** void SetEmissive(const ignition::math::Color &_clr)
    + Color GetAmbient() const now returns ignition::math::Color
    + Color GetDiffuse() const now returns ignition::math::Color
    + Color GetSpecular() const now returns ignition::math::Color
    + Color GetEmissive() const now returns ignition::math::Color
1. **gazebo/common/Image.hh**
    + ***Deprecation:*** Color GetPixel(unsigned int _x, unsigned int _y) const
    + ***Replacement:*** ignition::math::Color Pixel(unsigned int _x, unsigned int _y) const
    + ***Deprecation:*** Color GetAvgColor()
    + ***Replacement:*** ignition::math::Color AvgColor()
    + ***Deprecation:*** Color GetMaxColor() const
    + ***Replacement:*** ignition::math::Color MaxColor() const
1. **gazebo/test/ServerFixture.hh**
    + ***Deprecation:*** SpawnLight function that accepts common::Color
    + ***Replacement:*** SpawnLight function that accepts ignition::math::Color
1. **gazebo/common/Color.hh**
    + gazebo::common::Color is deprecated, use ignition::math::Color instead.

## Gazebo 7.X to 8.X

### Build system

1. ***Disable tests compilation by default:*** tests compilation has been
   excluded from the make all (default make) target.  cmake will generate make
   targets for all the tests in the code, individual test compilation can be
   trigger by using: make <binary_test_name> (i.e make UNIT_gz_TEST).
   For compiling the whole test suite see the new 'make tests' target.

1. ***New 'tests' target for make***: a new 'tests' target has been implemented.
   it will compile all the tests present in the code by calling 'make tests'.

1. ***Deprecate ENABLE_TESTS_COMPILATION parameter:***  the previous cmake
   parameter to control tests make target generation has been deprecated. Tests
   compilation is disabled by default.

### Additions

1. **gazebo/common/Material.hh**
    + changed default lighting value to true

1. **gazebo/common/Event.hh**
    + public: bool Connection::Id() const;
    + public: bool Event::Signaled() const;
    + public: void Event::SetSignaled(const bool);

### Modifications

1. **gazebo/gui/**
    + Dropped support for Qt4 and migrated the gui library to use Qt5.

1. **gazebo/physics/RayShape.hh**
    + Changed `math::Vector3 relativeStartPos` to `ignition::math::Vector3d relativeStartPos`
    + Changed `math::Vector3 relativeEndPos` to `ignition::math::Vector3d relativeEndPos`
    + Changed `math::Vector3 globalStartPos` to `ignition::math::Vector3d globalStartPos`
    + Changed `math::Vector3 globalEndPos` to `ignition::math::Vector3d globalEndPos`

1. **gazebo/physics/MultiRayShape.hh**
    + Changed `protected: math::Pose offset;` to `protected: ignition::math::Pose3d offset;`

1. **gazebo/physics/Contact.hh**
    + Changed `math::Vector3 positions[MAX_CONTACT_JOINTS]` to `ignition::math::Vector3d positions[MAX_CONTACT_JOINTS]`
    + Changed `math::Vector3 normals[MAX_CONTACT_JOINTS]` to `ignition::math::Vector3d normals[MAX_CONTACT_JOINTS]`

1. **gazebo/physics/Entity.hh**
    + `gazebo::math::Pose worldPose` replaced with `ignition::math::Pose3d worldPose`
    + `gazebo::math::Pose animationStartPose` replaced with `ignition::math::Pose3d animationStartPose`
    + `gazebo::math::Pose dirtyPose` replaced with `ignition::math::Pose3d dirtyPose`
    + `gazebo::math::Pose initialRelativePose` replaced with `ignition::math::Pose3d initialRelativePose`

1. **gazebo/physics/Model.hh**
    + Changed `std::vector<math::Pose> attachedModelsOffset` to `std::vector<ignition::math::Pose3d> attachedModelsOffset`.

1. **gazebo/physics/Joint.hh**
    + `gazebo::math::Vector3 anchorPos` replaced with `ignition::math::Vector3d anchorPos`
    + `gazebo::math::Pose anchorPose` replaced with `ignition::math::Pose3d anchorPose`
    + `gazebo::math::Pose parentAnchorPose` replaced with `ignition::math::Pose3d parentAnchorPose`
    + `gazebo::math::Angle lowerLimit` replaced with `double lowerLimit`
    + `gazebo::math::Angle upperLimit` replaced with `double upperLimit`
    + `gazebo::math::Angle staticAngle` replaced with `double staticAngle`

1. **gazebo/test/ServerFixture.hh**
    + ***Deprecation:*** all public methods using gazebo::math
    + ***Replacement:*** same signatures for methods just replacing gazebo::math by ignition::math

1. **physics/SurfaceParams.hh**
    + Changed the type of `FrictionPyramid::direction1` from
    `gazebo::math::Vector3` to `ignition::math::Vector3d`.

1. **gazebo/physics/Population.hh**
    + `gazebo::math::Vector3 size` replaced with `ignition::math::Vector3d size`
    + `gazebo::math::Vector3 step` replaced with `ignition::math::Vector3d step`
    + `gazebo::math::Pose pose` replaced with `ignition::math::Pose3d pose`

1. **plugins/events/Region.hh**
    + ***Deprecation:*** public: bool Contains(const math::Vector3 &_p) const
    + ***Replacement:*** public: bool Contains(const ignition::math::Vector3d &_p) const
    + changed type from `std::vector<math::Box> boxes` to `std::vector<ignition::math::Box> boxes`

1. **plugins/events/EventSource.hh**
    + changed type from `typedef boost::shared_ptr<EventSource> EventSourcePtr` to `typedef std::shared_ptr<EventSource> EventSourcePtr`

1. **plugins/BuoyancyPlugin.hh**
    + VolumeProperties: changed type from `public: math::Vector3 cov` to `public ignition::math::Vector3d cov`

1. **plugins/ArrangePlugin.hh**
    + Object: changed type from `public: math::Pose pose` to `public: ignition::math::Pose3d pose`
    + changed type from `typedef boost::shared_ptr<Object> ObjectPtr` to `std::shared_ptr<Object> ObjectPtr`
    + changed type from `typedef std::map<std::string, math::Pose> Pose_M` to `typedef std::map<std::string, ignition::math::Pose3d> Pose_M`

1. **plugins/LiftDragPlugin.hh**
    + changed type from `protected: math::Vector3 cp` to `protected: ignition::math::Vector3d cp`
    + changed type from `protected: math::Vector3 forward` to `protected: ignition::math::Vector3d forward`
    + changed type from `protected: math::Vector3 upward` to `protected: ignition::math::Vector3d upward`
    + changed type from `protected: math::Vector3 velSmooth` to `protected: ignition::math::Vector3d velSmooth`

1. **gazebo/physics/dart/**
    + Updated to support version 5 of DART physics engine.

1. **gazebo/rendering/Road2d.hh**
    + Modified to inherit from Visual class.

1. **gazebo/common/Event.hh**
    + Connection(Event*, int) constructor changed to
      Connection(Event*, const int)
    + EventTPrivate no longer inherits from EventPrivate
    + EventT::Connect(const boost::function<T> &) changed to
      EventT::Connect(const std::function<T> &)

1. **gazebo/sensors/DepthCameraSensor.hh**
    + Modified to inherit from CameraSensor class.

1. **gazebo/gui/model/ModelEditorEvents.hh**
    + ***Removed:*** ConnectSetSelectedLink
    + ***Replacement:*** ConnectSetSelectedEntity
    + ***Removed:*** DisconnectSetSelectedLink
    + ***Replacement:*** DisconnectSetSelectedEntity
    + ***Removed:*** setSelectedLink
    + ***Replacement:*** setSelectedEntity
    + ***Removed:*** event::EventT<void (std::string)> requestModelPluginRemoval;
    + ***Replacement:*** event::EventT<void (std::string, bool)> requestModelPluginRemoval;
    + ***Removed:*** event::EventT<void (std::string, std::string, std::string)> requestModelPluginInsertion;
    + ***Replacement:*** event::EventT<void (std::string, std::string, std::string, bool)> requestModelPluginInsertion;

1. **gazebo/gui/GuiEvents.hh**
    + ***Removed:*** event::EventT<void (const std::string &, const gazebo::math::Vector3 &)> Events::scaleEntity
    + ***Replacement:*** event::EventT<void (const std::string &, const ignition::math::Vector3d &)> Events::scaleEntity

1. **gazebo/common/CommonTypes.hh**
    + ***Removed:*** GAZEBO_DEPRECATED
1. **gazebo/util/system.hh**
    + ***Replacement:*** GAZEBO_DEPRECATED

1. **gazebo/common/CommonTypes.hh**
    + ***Removed:*** GAZEBO_FORCEINLINE
1. **gazebo/util/system.hh**
    + ***Replacement:*** GAZEBO_FORCEINLINE

1. **gazebo/rendering/OculusCamera.hh**
    + ***Removed:*** public: virtual bool MoveToPosition(const math::Pose &_pose, double _time)

1. **gazebo/rendering/UserCamera.hh**
    + ***Removed:*** public: virtual bool MoveToPosition(const math::Pose &_pose, double _time)

### Deprecations

1. **plugins/ContainPlugin.hh**
    + ***Deprecation:*** Gazebo transport publisher on <namespace>/contain
    + ***Replacement:*** Ignition transport publisher on <namespace>/contain
    + ***Deprecation:*** Gazebo transport subscriber on <namespace>/enable
    + ***Replacement:*** Ignition transport service on <namespace>/enable

1. **gazebo/physics/RayShape.hh**
    + ***Deprecation:*** void SetPoints(const math::Vector3 &_posStart, const math::Vector3 &_posEnd)
    + ***Replacement:*** void SetPoints(const ignition::math::Vector3d &_posStart, const ignition::math::Vector3d &_posEnd)
    + ***Deprecation:*** virtual void GetRelativePoints(math::Vector3 &_posA, math::Vector3 &_posB)
    + ***Replacement:*** virtual void RelativePoints(ignition::math::Vector3d &_posA, ignition::math::Vector3d &_posB)
    + ***Deprecation:*** virtual void GetGlobalPoints(math::Vector3 &_posA, math::Vector3 &_posB)
    + ***Replacement:*** virtual void GlobalPoints(ignition::math::Vector3d &_posA, ignition::math::Vector3d &_posB)

1. **gazebo/physics/MeshShape.hh**
    + ***Deprecation:*** math::Vector3 GetSize() const
    + ***Replacement:*** ignition::math::Vector3d Size() const

1. **gazebo/physics/MapShape.hh**
    + ***Deprecation:*** math::Vector3 GetScale() const
    + ***Replacement:*** ignition::math::Vector3d Scale() const

1. **gazebo/physics/BoxShape.hh**
    + ***Deprecation:*** void SetSize(const math::Vector3 &_size)
    + ***Replacement:*** void SetSize(const ignition::math::Vector3d &_size)
    + ***Deprecation:*** math::Vector3 GetSize() const
    + ***Replacement:*** ignition::math::Vector3d Size() const

1. **gazebo/rendering/MovableText.hh**
    + ***Deprecation:*** math::Box GetAABB()
    + ***Replacement:*** ignition::math::Box AABB()

1. **gazebo/physics/Road.hh**
    + ***Deprecation:*** const std::vector<math::Vector3> &GetPoints() const
    + ***Replacement:*** const std::vector<ignition::math::Vector3d> &Points() const

1. **gazebo/physics/Collision.hh**
    + ***Deprecation:*** inline virtual const math::Pose GetWorldPose() const
    + ***Replacement:*** inline virtual const ignition::math::Pose3d &WorldPose() const
    + ***Deprecation:*** virtual math::Box GetBoundingBox() const
    + ***Replacement:*** virtual ignition::math::Box BoundingBox() const
    + ***Deprecation:*** void SetScale(const math::Vector3 &_scale)
    + ***Replacement:*** void SetScale(const ignition::math::Vector3d &_scale)

1. **gazebo/physics/Model.hh**
    + ***Deprecation:*** virtual math::Box GetBoundingBox() const
    + ***Replacement:*** virtual ignition::math::Box BoundingBox() const
    + ***Deprecation:*** void SetLinearVel(const math::Vector3 &_vel)
    + ***Replacement:*** void SetLinearVel(const ignition::math::Vector3d &_vel)
    + ***Deprecation:*** void SetAngularVel(const math::Vector3 &_vel)
    + ***Replacement:*** void SetAngularVel(const ignition::math::Vector3d &_vel)
    + ***Deprecation:*** void SetLinearAccel(const math::Vector3 &_vel)
    + ***Replacement:*** void SetLinearAccel(const ignition::math::Vector3d &_vel)

    + ***Deprecation:*** void SetAngularAccel(const math::Vector3 &_vel)
    + ***Replacement:*** void SetAngularAccel(const ignition::math::Vector3d &_vel)
    + ***Deprecation:*** void AttachStaticModel(ModelPtr &_model, math::Pose _offset)
    + ***Replacement:*** void AttachStaticModel(ModelPtr &_model, ignition::math::Pose3d _offset)

    + ***Deprecation:*** void SetLinkWorldPose(const math::Pose &_pose, std::string _linkName)
    + ***Replacement:*** void SetLinkWorldPose(const ignition::math::Pose3d &_pose, std::string _linkName)

    + ***Deprecation:*** void SetLinkWorldPose(const math::Pose &_pose, const LinkPtr &_link)
    + ***Replacement:*** void SetLinkWorldPose(const ignition::math::Pose3d &_pose, const LinkPtr &_link)

1. **gazebo/physics/Joint.hh**
    + ***Deprecation:*** math::Pose GetInitialAnchorPose() const
    + ***Replacement:*** ignition::math::Pose3d InitialAnchorPose() const
    + ***Deprecation:*** math::Pose GetWorldPose() const
    + ***Replacement:*** ignition::math::Pose3d WorldPose() const
    + ***Deprecation:*** math::Pose GetAnchorErrorPose() const
    + ***Replacement:*** ignition::math::Pose3d AnchorErrorPose() const
    + ***Deprecation:*** math::Quaternion GetAxisFrame(unsigned int _index) const
    + ***Replacement:*** ignition::math::Quaterniond AxisFrame(const unsigned int _index) const
    + ***Deprecation:*** math::Quaternion GetAxisFrameOffset(unsigned int _index) const
    + ***Replacement:*** ignition::math::Quaterniond AxisFrameOffset(const unsigned int _index) const
    + ***Deprecation:*** math::Pose ComputeChildLinkPose(unsigned int _index, double _position)
    + ***Replacement:*** ignition::math::Pose3d ChildLinkPose(const unsigned int _index, const double _position)
    + ***Deprecation:*** virtual unsigned int GetAngleCount() const = 0
    + ***Replacement:*** virtual unsigned int DOF() const = 0
    + ***Deprecation:*** math::Angle GetAngle(unsigned int _index) const
    + ***Replacement:*** virtual double Position(const unsigned int _index = 0) const final
    + ***Deprecation:*** virtual math::Angle GetAngleImpl(unsigned int _index) const = 0
    + ***Replacement:*** virtual double PositionImpl(const unsigned int _index = 0) const = 0
    + ***Deprecation:*** bool SetHighStop(unsigned int _index, const math::Angle &_angle)
    + ***Replacement:*** virtual void SetUpperLimit(const unsigned int _index, const double _limit)
    + ***Deprecation:*** void SetUpperLimit(unsigned int _index, math::Angle _limit)
    + ***Replacement:*** virtual void SetUpperLimit(const unsigned int _index, const double _limit)
    + ***Deprecation:*** bool SetLowStop(unsigned int _index, const math::Angle &_angle)
    + ***Replacement:*** virtual oid SetLowerLimit(const unsigned int _index, const double _limit)
    + ***Deprecation:*** void SetLowerLimit(unsigned int _index, math::Angle _limit)
    + ***Replacement:*** virtual oid SetLowerLimit(const unsigned int _index, const double _limit)
    + ***Deprecation:*** virtual math::Angle GetHighStop(unsigned int _index) = 0
    + ***Replacement:*** virtual double UpperLimit(const unsigned int _index = 0) const
    + ***Deprecation:*** math::Angle GetUpperLimit(unsigned int _index) const
    + ***Replacement:*** virtual double UpperLimit(const unsigned int _index = 0) const
    + ***Deprecation:*** virtual math::Angle GetLowStop(unsigned int _index) = 0
    + ***Replacement:*** virtual double LowerLimit(const unsigned int _index = 0) const
    + ***Deprecation:*** math::Angle GetLowerLimit(unsigned int _index) const
    + ***Replacement:*** virtual double LowerLimit(const unsigned int _index = 0) const
    + ***Deprecation:*** void Load(LinkPtr _parent, LinkPtr _child, const math::Pose &_pose)
    + ***Replacement:*** void Load(LinkPtr _parent, LinkPtr _child, const ignition::math::Pose3d &_pose)
    + ***Deprecation:*** void SetAxis(unsigned int _index, const math::Vector3 &_axis) = 0
    + ***Replacement:*** void SetAxis(const unsigned int _index, const ignition::math::Vector3d &_axis) = 0
    + ***Deprecation:*** math::Vector3 GetLocalAxis(unsigned int _index) const
    + ***Replacement:*** ignition::math::Vector3d LocalAxis(const unsigned int _index) const
    + ***Deprecation:*** math::Vector3 GetGlobalAxis(unsigned int _index)
    + ***Replacement:*** ignition::math::Vector3d GlobalAxis(unsigned int _index) const
    + ***Deprecation:*** void SetAnchor(unsigned int _index, const math::Vector3 &_anchor) = 0
    + ***Replacement:*** void SetAnchor(const unsigned int _index, const ignition::math::Vector3d &_anchor) = 0
    + ***Deprecation:*** math::Vector3 GetAnchor(unsigned int _index) const = 0
    + ***Replacement:*** ignition::math::Vector3d Anchor(const unsigned int _index) const = 0
    + ***Deprecation:*** math::Vector3 GetLinkForce(unsigned int _index) const = 0
    + ***Replacement:*** ignition::math::Vector3d LinkForce(const unsigned int _index) const = 0
    + ***Deprecation:*** math::Vector3 GetLinkTorque(unsigned int _index) const = 0
    + ***Replacement:*** ignition::math::Vector3d LinkTorque(const unsigned int _index) const = 0
    + ***Deprecation:*** double GetInertiaRatio(const math::Vector3 &_axis) const
    + ***Replacement:*** double InertiaRatio(const ignition::math::Vector3d &_axis) const

1. **gazebo/physics/Shape.hh**
    + ***Deprecation:*** void SetScale(const math::Vector3 &_scale)
    + ***Replacement:*** void SetScale(const ignition::math::Vector3d &_scale)
    + ***Deprecation:*** math::Vector3 GetScale() const
    + ***Replacement:*** ignition::math::Vector3d Scale() const

1. **gazebo/physics/HeightmapShape.hh**
    + ***Deprecation:*** math::Vector2i GetVertexCount() const
    + ***Replacement:*** ignition::math::Vector2i VertexCount() const
    + ***Deprecation:*** math::Vector3 GetSize() const
    + ***Replacement:*** ignition::math::Vector3d Size() const
    + ***Deprecation:*** math::Vector3 GetPos() const
    + ***Replacement:*** ignition::math::Vector3d Pos() const

1. **gazebo/physics/Entity.hh**
    + ***Deprecation:*** const math::Pose GetDirtyPose() const
    + ***Replacement:*** const ignition::math::Pose3d &DirtyPose() const
    + ***Deprecation:*** inline virtual const math::Pose GetWorldPose() const
    + ***Replacement:*** inline virtual const ignition::math::Pose3d &WorldPose() const
    + ***Deprecation:*** void SetInitialRelativePose(const math::Pose &_pose)
    + ***Replacement:*** void SetInitialRelativePose(const ignition::math::Pose3d &_pose)
    + ***Deprecation:*** math::Pose GetInitialRelativePose() const
    + ***Replacement:*** ignition::math::Pose3d InitialRelativePose() const
    + ***Deprecation:*** virtual math::Box GetBoundingBox() const
    + ***Replacement:*** virtual ignition::math::Box BoundingBox() const
    + ***Deprecation:*** void SetRelativePose(const math::Pose &_pose, bool _notify = true, bool _publish = true)
    + ***Replacement:*** void SetRelativePose(const ignition::math::Pose3d &_pose, const bool _notify = true, const bool _publish = true)
    + ***Deprecation:*** void SetWorldPose(const math::Pose &_pose, bool _notify = true, bool _publish = true)
    + ***Replacement:*** void SetWorldPose(const ignition::math::Pose3d &_pose, const bool _notify = true, const bool _publish = true)
    + ***Deprecation:*** virtual math::Vector3 GetRelativeLinearVel() const
    + ***Replacement:*** virtual ignition::math::Vector3d RelativeLinearVel() const
    + ***Deprecation:*** virtual math::Vector3 GetWorldLinearVel() const
    + ***Replacement:*** virtual ignition::math::Vector3d WorldLinearVel() const
    + ***Deprecation:*** virtual math::Vector3 GetRelativeAngularVel() const
    + ***Replacement:*** virtual ignition::math::Vector3d RelativeAngularVel() const
    + ***Deprecation:*** virtual math::Vector3 GetWorldAngularVel() const
    + ***Replacement:***  virtual ignition::math::Vector3d WorldAngularVel() const
    + ***Deprecation:*** virtual math::Vector3 GetRelativeLinearAccel() const
    + ***Replacement:***  virtual ignition::math::Vector3d RelativeLinearAccel() const
    + ***Deprecation:*** virtual math::Vector3 GetWorldLinearAccel() const
    + ***Replacement:***  virtual ignition::math::Vector3d WorldLinearAccel() const
    + ***Deprecation:*** virtual math::Vector3 GetRelativeAngularAccel() const
    + ***Replacement:*** virtual ignition::math::Vector3d RelativeAngularAccel() const
    + ***Deprecation:*** virtual math::Vector3 GetWorldAngularAccel() const
    + ***Replacement:*** virtual ignition::math::Vector3d WorldAngularAccel() const
    + ***Deprecation:*** math::Box GetCollisionBoundingBox() const
    + ***Replacement:*** ignition::math::Box CollisionBoundingBox() const
    + ***Deprecation:*** math::Pose GetRelativePose() const
    + ***Replacement:*** ignition::math::Pose3d RelativePose() const
    + ***Deprecation:*** void SetWorldTwist(const math::Vector3 &_linear, const math::Vector3 &_angular, bool _updateChildren = true)
    + ***Replacement:*** void SetWorldTwist(const ignition::math::Vector3d &_linear, const ignition::math::Vector3d &_angular, const bool _updateChildren = true)

1. **gazebo/physics/PlaneShape.hh**
    + ***Deprecation:*** void SetSize(const math::Vector2d &_size)
    + ***Replacement:*** void SetSize(const ignition::math::Vector2d &_size)
    + ***Deprecation:*** math::Vector2d GetSize() const
    + ***Replacement:*** ignition::math::Vector2d Size() const
    + ***Deprecation:*** void SetNormal(const math::Vector3 &_norm)
    + ***Replacement:*** void SetNormal(const ignition::math::Vector3d &_norm)
    + ***Deprecation:*** math::Vector3 GetNormal() const
    + ***Replacement:*** ignition::math::Vector3d Normal() const
    + ***Deprecation:*** void SetAltitude(const math::Vector3 &_pos)
    + ***Replacement:*** void SetAltitude(const ignition::math::Vector3d &_pos)

1. **gazebo/physics/CollisionState.hh**
    + ***Deprecation:*** const math::Pose &GetPose() const
    + ***Replacement:*** const ignition::math::Pose3d &Pose() const

1. **gazebo/physics/JointState.hh**
    + ***Deprecation:*** math::Angle GetAngle(unsigned int _axis) const
    + ***Replacement:*** double Position(const unsigned int _axis = 0) const
    + ***Deprecation:*** const std::vector<math::Angle> GetAngles() const
    + ***Replacement:*** const std::vector<double> &Positions() const

1. **gazebo/physics/LinkState.hh**
    + ***Deprecation:*** const math::Pose &GetPose() const
    + ***Replacement:*** const ignition::math::Pose3d &Pose() const
    + ***Deprecation:*** const math::Pose GetVelocity() const
    + ***Replacement:*** const ignition::math::Pose3d &Velocity() const
    + ***Deprecation:*** const math::Pose GetAcceleration() const
    + ***Replacement:*** const ignition::math::Pose3d &Acceleration() const
    + ***Deprecation:*** const math::Pose GetWrench() const
    + ***Replacement:*** const ignition::math::Pose3d &Wrench() const

1. **gazebo/physics/ModelState.hh**
    + ***Deprecation:*** const math::Pose &GetPose() const
    + ***Replacement:*** const ignition::math::Pose3d &Pose() const

1. **gazebo/rendering/Distortion.hh**
    + ***Deprecation:*** double GetK1() const
    + ***Replacement:*** double K1() const
    + ***Deprecation:*** double GetK2() const
    + ***Replacement:*** double K2() const
    + ***Deprecation:*** double GetK3() const
    + ***Replacement:*** double K3() const
    + ***Deprecation:*** double GetP1() const
    + ***Replacement:*** double P1() const
    + ***Deprecation:*** double GetP2() const
    + ***Replacement:*** double P2() const
    + ***Deprecation:*** math::Vector2d GetCenter() const
    + ***Replacement:*** ignition::math::Vector2d Center() const
    + ***Deprecation:*** static math::Vector2d Distort(const math::Vector2d &_in, const math::Vector2d &_center, double _k1, double _k2, double _k3, double _p1, double _p2)
    + ***Replacement:*** static ignition::math::Vector2d Distort( const ignition::math::Vector2d &_in, const ignition::math::Vector2d &_center, double _k1, double _k2, double _k3, double _p1, double _p2)

1. **gazebo/rendering/COMVisual.hh**
    + ***Deprecation:*** math::Pose GetInertiaPose() const
    + ***Replacement:*** ignition::math::Pose3d InertiaPose() const

1. **gazebo/rendering/JointVisual.hh**
    + ***Deprecation:*** void Load(ConstJointPtr &_msg, const math::Pose &_worldPose)
    + ***Replacement:*** void Load(ConstJointPtr &_msg, const ignition::math::Pose3d &_worldPose)
    + ***Deprecation:*** ArrowVisualPtr CreateAxis(const math::Vector3 &_axis, bool _useParentFrame, msgs::Joint::Type _type)
    + ***Replacement:*** ArrowVisualPtr CreateAxis(const ignition::math::Vector3d &_axis, const bool _useParentFrame, const msgs::Joint::Type _type)
    + ***Deprecation:*** void UpdateAxis(ArrowVisualPtr _arrowVisual, const math::Vector3 &_axis, bool _useParentFrame, msgs::Joint::Type _type)
    + ***Replacement:*** void UpdateAxis(ArrowVisualPtr _arrowVisual, const ignition::math::Vector3d &_axis, const bool _useParentFrame, const msgs::Joint::Type _type)

1. **gazebo/rendering/OrbitViewController.hh**
    + ***Deprecation:*** virtual void Init(const math::Vector3 &_focalPoint, const double _yaw = 0, const double _pitch = 0)
    + ***Replacement:*** virtual void Init(const ignition::math::Vector3d &_focalPoint, const double _yaw = 0, const double _pitch = 0)
    + ***Deprecation:*** void SetFocalPoint(const math::Vector3 &_fp)
    + ***Replacement:*** void SetFocalPoint(const ignition::math::Vector3d &_fp)
    + ***Deprecation:*** math::Vector3 GetFocalPoint() const
    + ***Replacement:*** ignition::math::Vector3d FocalPoint() const
    + ***Deprecation:*** void TranslateLocal(const math::Vector3 &_vec)
    + ***Replacement:*** void TranslateLocal(const ignition::math::Vector3d &_vec)
    + ***Deprecation:*** void TranslateGlobal(const math::Vector3 &_vec)
    + ***Replacement:*** void TranslateGlobal(const ignition::math::Vector3d &_vec)

1. **gazebo/rendering/OrthoViewController.hh**
    + ***Deprecation:*** virtual void Init(const math::Vector3 &_focalPoint, const double _yaw = 0, const double _pitch = 0)
    + ***Replacement:*** virtual void Init(const ignition::math::Vector3d &_focalPoint, const double _yaw = 0, const double _pitch = 0)

1. **gazebo/rendering/Projector.hh**
    + ***Deprecation:*** void Load(const std::string &_name, const math::Pose &_pose = math::Pose(0, 0, 0, 0, 0, 0), const std::string &_textureName = "", double _nearClip = 0.25, double _farClip = 15.0, double _fov = IGN_PI * 0.25)
    + ***Replacement:*** void Load(const std::string &_name, const ignition::math::Pose3d &_pose = ignition::math::Pose3d::Zero, const std::string &_textureName = "", const double _nearClip = 0.25, const double _farClip = 15.0, const double _fov = IGN_PI * 0.25);
    + ***Deprecation:*** void SetPose(const math::Pose &_pose)
    + ***Replacement:*** void SetPose(const ignition::math::Pose3d &_pose)

1. **gazebo/rendering/UserCamera.hh**
    + ***Deprecation:*** void SetViewController(const std::string &_type, const math::Vector3 &_pos)
    + ***Replacement:*** void SetViewController(const std::string &_type, const ignition::math::Vector3d &_pos)
    + ***Deprecation:*** void SetFocalPoint(const math::Vector3 &_pt)
    + ***Replacement:*** void SetFocalPoint(const ignition::math::Vector3d &_pt)
    + ***Deprecation:*** VisualPtr GetVisual(const math::Vector2i &_mousePos) const
    + ***Replacement:*** VisualPtr Visual(const ignition::math::Vector2i &_mousePos) const
    + ***Deprecation:*** VisualPtr GetVisual(const math::Vector2i &_mousePos, std::string &_mod)
    + ***Replacement:*** VisualPtr Visual(const ignition::math::Vector2i &_mousePos, std::string &_mod) const
    + ***Deprecation:*** void SetDefaultPose(const math::Pose &_pose)
    + ***Replacement:*** void SetInitialPose(const ignition::math::Pose3d &_pose)
    + ***Deprecation:*** math::Pose3 DefaultPose() const
    + ***Replacement:*** ignition::math::Pose3d InitialPose() const

1. **gazebo/rendering/ViewController.hh**
    + ***Deprecation:*** virtual void Init(const math::Vector3 &_focalPoint, const double _yaw = 0, const double _pitch = 0)
    + ***Replacement:*** virtual void Init(const ignition::math::Vector3d &_focalPoint, const double _yaw = 0, const double _pitch = 0)

1. **gazebo/physics/Gripper.hh**
    + ***Deprecation:*** std::string GetName() const
    + ***Replacement:*** std::string Name() const

1. **gazebo/physics/World.hh**
    + ***Deprecation:*** bool GetRunning() const
    + ***Replacement:*** bool Running() const;
    + ***Deprecation:*** std::string GetName() const
    + ***Replacement:*** std::string Name() const
    + ***Deprecation:*** PhysicsEnginePtr GetPhysicsEngine() const
    + ***Replacement:*** PhysicsEnginePtr Physics() const
    + ***Deprecation:*** PresetManagerPtr GetPresetManager() const
    + ***Replacement:*** PresetManagerPtr PresetMgr() const
    + ***Deprecation:*** common::SphericalCoordinatesPtr GetSphericalCoordinates() const
    + ***Replacement:*** common::SphericalCoordinatesPtr SphericalCoords() const
    + ***Deprecation:*** unsigned int GetModelCount() const
    + ***Replacement:*** unsigned int ModelCount() const
    + ***Deprecation:*** ModelPtr GetModel(unsigned int _index) const
    + ***Replacement:*** ModelPtr ModelByIndex(const unsigned int _index) const
    + ***Deprecation:*** Model_V GetModels() const
    + ***Replacement:*** Model_V Models() const
    + ***Deprecation:*** common::Time GetSimTime() const
    + ***Replacement:*** common::Time SimTime() const
    + ***Deprecation:*** common::Time GetPauseTime() const
    + ***Replacement:*** common::Time PauseTime() const
    + ***Deprecation:*** common::Time GetStartTime() const
    + ***Replacement:*** common::Time StartTime() const
    + ***Deprecation:*** common::Time GetRealTime() const
    + ***Replacement:*** common::Time RealTime() const
    + ***Deprecation:*** BasePtr GetByName(const std::string &_name)
    + ***Replacement:*** BasePtr BaseByName(const std::string &_name) const
    + ***Deprecation:*** ModelPtr GetModel(const std::string &_name)
    + ***Replacement:*** ModelPtr ModelByName(const std::string &_name) const
    + ***Deprecation:*** LightPtr Light(const std::string &_name) const
    + ***Replacement:*** LightPtr LightByName(const std::string &_name) const
    + ***Deprecation:*** EntityPtr GetEntity(const std::string &_name)
    + ***Replacement:*** EntityPtr EntityByName(const std::string &_name) const
    + ***Deprecation:*** ModelPtr GetModelBelowPoint(const math::Vector3 &_pt)
    + ***Replacement:*** ModelPtr ModelBelowPoint(const ignition::math::Vector3d &_pt) const
    + ***Deprecation:*** EntityPtr GetEntityBelowPoint(const math::Vector3 &_pt)
    + ***Replacement:*** EntityPtr EntityBelowPoint(const ignition::math::Vector3d &_pt) const
    + ***Deprecation:*** std::mutex &GetSetWorldPoseMutex() const
    + ***Replacement:*** std::mutex &WorldPoseMutex() const
    + ***Deprecation:*** bool GetEnablePhysicsEngine()
    + ***Replacement:*** bool PhysicsEnabled() const
    + ***Deprecation:*** void EnablePhysicsEngine(const bool _enable)
    + ***Replacement:*** void SetPhysicsEnabled(const bool _enable)
    + ***Deprecation:*** uint32_t GetIterations() const
    + ***Replacement:*** uint32_t Iterations() const
    + ***Deprecation:*** msgs::Scene GetSceneMsg() const
    + ***Replacement:*** msgs::Scene SceneMsg() const

1. **gazebo/rendering/Visual.hh**
    + ***Deprecation:*** public: void SetScale(const math::Vector3 &_scale)
    + ***Replacement:*** public: void SetScale(const ignition::math::Vector3d &_scale)
    + ***Deprecation:*** public: void SetPosition(const math::Vector3 &_pos)
    + ***Replacement:*** public: void SetPosition(const ignition::math::Vector3d &_pos)
    + ***Deprecation:*** std::string GetName() const
    + ***Replacement:*** std::string Name() const
    + ***Deprecation:*** math::Vector3 GetScale()
    + ***Replacement:*** ignition::math::Vector3d Scale() const
    + ***Deprecation:*** void SetRotation(const math::Quaternion &_rot)
    + ***Replacement:*** void SetRotation(const ignition::math::Quaterniond &_rot)
    + ***Deprecation:*** void SetPose(const math::Pose &_pose)
    + ***Replacement:*** void SetPose(const ignition::math::Pose3d &_pose)
    + ***Deprecation:*** math::Vector3 GetPosition() const
    + ***Replacement:*** ignition::math::Vector3d Position() const
    + ***Deprecation:*** math::Quaternion GetRotation() const
    + ***Replacement:*** ignition::math::Quaterniond Rotation() const
    + ***Deprecation:*** math::Pose GetPose() const
    + ***Replacement:*** ignition::math::Pose3d Pose()
    + ***Deprecation:*** math::Pose GetWorldPose() const
    + ***Replacement:*** ignition::math::Pose3d WorldPose() const
    + ***Deprecation:*** void SetWorldPosition(const math::Vector3 &_pos)
    + ***Replacement:*** void SetWorldPosition(const ignition::math::Vector3d &_pos)
    + ***Deprecation:*** void SetWorldRotation(const math::Quaternion &_pos)
    + ***Replacement:*** void SetWorldRotation(const ignition::math::Quaterniond &_pos)
    + ***Deprecation:*** math::Box GetBoundingBox() const
    + ***Replacement:*** ignition::math::Box BoundingBox() const
    + ***Deprecation:*** void MoveToPosition(const math::Pose &_pose, double _time)
    + ***Replacement:*** void MoveToPosition(const ignition::math::Pose3d &_pose, const double _time)
    + ***Deprecation:*** void MoveToPositions(const std::vector<math::Pose> &_pts, double _time, std::function<void()> _onComplete = nullptr)
    + ***Replacement:*** void MoveToPositions(const std::vector<ignition::math::Pose3d> &_pts, const double _time, std::function<void()> _onComplete = nullptr)
    + ***Deprecation:*** void SetWorldPose(const math::Pose &_pose)
    + ***Replacement:*** void SetWorldPose(const ignition::math::Pose3d &_pose)

1. **gazebo/rendering/Camera.hh**
    + ***Deprecation:*** public: virtual void SetWorldPose(const math::Pose &_pose)
    + ***Replacement:*** public: virtual void SetWorldPose(const ignition::math::Pose3d &_pose)

1. **gazebo/rendering/WireBox.hh**
    + ***Deprecation:*** public: explicit WireBox(VisualPtr _parent, const math::Box &_box)
    + ***Replacement:*** public: explicit WireBox(VisualPtr _parent, const ignition::math::Box &_box);
    + ***Deprecation:*** public: void Init(const math::Box &_box)
    + ***Replacement:*** public: void Init(const ignition::math::Box &_box)
    + ***Deprecation:*** public: bool GetVisible() const
    + ***Replacement:*** public: bool Visible() const
    + ***Deprecation:*** public: math::Box GetBox() const
    + ***Replacement:*** public: ignition::math::Box Box()

1. **gazebo/gui/ModelManipulator.hh**
    + ***Deprecation:*** public: void RotateEntity(rendering::VisualPtr &_vis,
                                                   const math::Vector3 &_axis,
                                                   bool _local = false);
    + ***Replacement:*** public: void RotateEntity(rendering::VisualPtr &_vis,
                                                   const ignition::math::Vector3d &_axis,
                                                   const bool _local = false)
    + ***Deprecation:*** public: void TranslateEntity(rendering::VisualPtr &_vis,
                                                      const math::Vector3 &_axis,
                                                      bool _local = false)
    + ***Replacement:*** public: void TranslateEntity(rendering::VisualPtr &_vis,
                                                      const ignition::math::Vector3d &_axis,
                                                      const bool _local = false);
    + ***Deprecation:*** public: void ScaleEntity(rendering::VisualPtr &_vis,
                                                  const math::Vector3 &_axis,
                                                  bool _local = false)
    + ***Replacement:*** public: void ScaleEntity(rendering::VisualPtr &_vis,
                                                  const ignition::math::Vector3d &_axis,
                                                  const bool _local = false)
    + ***Deprecation:*** public: static math::Vector3 SnapPoint(const math::Vector3 &_point,
                                                                double _interval = 1.0,
                                                                double _sensitivity = 0.4);
    + ***Replacement:*** public: static ignition::math::Vector3d SnapPoinit(const ignition::math::Vector3d &_point,
                                                                            const double _interval = 1.0,
                                                                            const double _sensitivity = 0.4)
    + ***Deprecation:*** public: static math::Vector3 GetMousePositionOnPlane(rendering::CameraPtr _camera,
                                                                              const common::MouseEvent &_event)
    + ***Replacement:*** public: static ignition::math::Vector3d MousePositionOnPlane(rendering::CameraPtr _camera,
                                                                                      const common::MouseEvent &_event)
    + ***Deprecation:*** public: static math::Vector3 GetMouseMoveDistance(rendering::CameraPtr _camera,
                                                                           const math::Vector2i &_start,
                                                                           const math::Vector2i &_end,
                                                                           const math::Pose &_pose,
                                                                           const math::Vector3 &_axis,
                                                                           bool _local)
    + ***Replacement:*** public: static ignition::math::Vector3d MouseMoveDistance(rendering::CameraPtr _camera,
                                                                                   const ignition::math::Vector2i &_start,
                                                                                   const ignition::math::Vector2i &_end,
                                                                                   const ignition::math::Pose3d &_pose,
                                                                                   const ignition::math::Vector3d &_axis,
                                                                                   const bool _local)

1. **gazebo/gui/ModelSnap.hh**
    + ***Deprecation:*** public: void Snap(const std::vector<math::Vector3> &_triangleSrc,
                                           const std::vector<math::Vector3> &_triangleDest,
                                           rendering::VisualPtr _visualSrc)
    + ***Replacement:*** public: void Snap(const ignition::math::Triangle3d &_triangleSrc,
                                           const ignition::math::Triangle3d &_triangleDest,
                                           rendering::VisualPtr _visualSrc);
    + ***Deprecation:*** public: void GetSnapTransform(const std::vector<math::Vector3> &_triangleSrc,
                                                       const std::vector<math::Vector3> &_triangleDest,
                                                       const math::Pose &_poseSrc, math::Vector3 &_trans,
                                                       math::Quaternion &_rot)
    + ***Replacement:*** public: void SnapTransform(const ignition::math::Triangle3d &_triangleSrc,
                                                    const ignition::math::Triangle3d &_triangleDest,
                                                    const ignition::math::Pose3d &_poseSrc,
                                                    ignition::math::Vector3d &_trans,
                                                    ignition::math::Quaterniond &_rot)

1. **gazebo/rendering/RayQuery.hh**
    + ***Deprecation:*** public: bool SelectMeshTriangle(int _x, int _y,
                                                         VisualPtr _visual,
                                                         math::Vector3 &_intersect,
                                                         std::vector<math::Vector3> &_vertices)
    + ***Replacement:*** public: bool SelectMeshTriangle(const int _x, const int _y,
                                                         const VisualPtr _visual,
                                                         ignition::math::Vector3d &_intersect,
                                                         ignition::math::Triangle3d &_triangle)

1. **gazebo/rendering/Road2d.hh**
    + ***Deprecation:*** public: void Load(VisualPtr);
    + ***Replacement:*** public: void Load(msgs::Road);

1. **gazebo/common/Event.hh**
    + ***Deprecation:*** public: void Event::Disconnect(ConnectionPtr);
    + ***Deprecation:*** public: void EventT::Disconnect(ConnectionPtr);
    + ***Replacement:*** Delete the Connection object, perhaps by calling
    reset() on its smart pointer.
    + ***Deprecation:*** public: bool Event::GetSignaled() const;
    + ***Replacement:*** public: bool Event::Signaled() const;
    + ***Deprecation:*** public: bool Connection::GetId() const;
    + ***Replacement:*** public: bool Connection::Id() const;

1. **gazebo/common/Events.hh**
    + ***Deprecation:*** public: void Events::Disconnect.*(ConnectionPtr);
    + ***Replacement:*** Delete the Connection object, perhaps by calling
    reset() on its smart pointer.

1. **gazebo/gui/GuiEvents.hh**
    + ***Deprecation:*** public: void Events::Disconnect.*(ConnectionPtr);
    + ***Replacement:*** Delete the Connection object, perhaps by calling
    reset() on its smart pointer.

1. **gazebo/gui/building/BuildingEditorEvents.hh**
    + ***Deprecation:*** public: void Events::Disconnect.*(ConnectionPtr);
    + ***Replacement:*** Delete the Connection object, perhaps by calling
    reset() on its smart pointer.

1. **gazebo/gui/model/ModelEditorEvents.hh**
    + ***Deprecation:*** public: void Events::Disconnect.*(ConnectionPtr);
    + ***Replacement:*** Delete the Connection object, perhaps by calling
    reset() on its smart pointer.

1. **gazebo/physics/Inertial.hh**
    + ***Deprecation:*** public: math::Matrix3 GetMOI() const
    + ***Replacement:*** public: ignition::math::Matrix3d MOI() const
    + ***Deprecation:*** public: math::Matrix3 GetMOI(const math::Pose) const
    + ***Replacement:*** public: ignition::math::Matrix3d MOI(const ignition::math::Pose3d) const
    + ***Deprecation:*** public: void SetMOI(const math::Matrix3)
    + ***Replacement:*** public: void SetMOI(const ignition::math::Matrix3d)

    + ***Deprecation:*** double GetMass() const
    + ***Replacement:*** double Mass() const
    + ***Deprecation:*** void SetCoG(const math::Vector3 &_center)
    + ***Replacement:*** void SetCoG(const ignition::math::Vector3d &_center)
    + ***Deprecation:*** void SetCoG(const math::Pose &_c)
    + ***Replacement:*** void SetCoG(const ignition::math::Pose3d &_c)
    + ***Deprecation:*** math::Vector3 GetCoG() const
    + ***Replacement:*** const ignition::math::Vector3d &CoG() const
    + ***Deprecation:*** const math::Pose GetPose() const
    + ***Replacement:*** const ignition::math::Pose3d Pose() const
    + ***Deprecation:*** math::Vector3 GetPrincipalMoments() const
    + ***Replacement:*** ignition::math::Vector3d PrincipalMoments() const
    + ***Deprecation:*** math::Vector3 GetProductsofInertia() const
    + ***Replacement:*** ignition::math::Vector3d ProductsOfInertia() const
    + ***Deprecation:*** double GetIXX() const
    + ***Replacement:*** double IXX() const
    + ***Deprecation:*** double GetIYY() const
    + ***Replacement:*** double IYY() const
    + ***Deprecation:*** double GetIZZ() const
    + ***Replacement:*** double IZZ() const
    + ***Deprecation:*** double GetIXY() const
    + ***Replacement:*** double IXY() const
    + ***Deprecation:*** double GetIXZ() const
    + ***Replacement:*** double IXZ() const
    + ***Deprecation:*** double GetIYZ() const
    + ***Replacement:*** double IYZ() const
    + ***Deprecation:*** void Rotate(const math::Quaternion &_rot)
    + ***Replacement:*** void Rotate(const ignition::math::Quaterniond &_rot)
    + ***Deprecation:*** Inertial GetInertial(const math::Pose &_frameOffset) const
    + ***Replacement:*** Inertial operator()(const ignition::math::Pose3d &_frameOffset) const

1. **gazebo/physics/Joint.hh**
    + ***Deprecation:*** public: void Joint::DisconnectJointUpdate(ConnectionPtr);
    + ***Replacement:*** Delete the Connection object, perhaps by calling
    reset() on its smart pointer.

1. **gazebo/physics/Link.hh**
    + ***Deprecation:*** public: math::Matrix3 GetWorldInertiaMatrix() const
    + ***Replacement:*** public: ignition::math::Matrix3d WorldInertiaMatrix() const
    + ***Deprecation:*** public: void Link::DisconnectEnabled(ConnectionPtr);
    + ***Replacement:*** Delete the Connection object, perhaps by calling
    reset() on its smart pointer.
    + ***Deprecation:*** virtual math::Box GetBoundingBox() const
    + ***Replacement:*** virtual ignition::math::Box BoundingBox() const
    + ***Deprecation:*** void SetScale(const math::Vector3 &_scale)
    + ***Replacement:*** void SetScale(const ignition::math::Vector3d &_scale)
    + ***Deprecation:*** virtual void SetLinearVel(const math::Vector3 &_vel)
    + ***Replacement:*** virtual void SetLinearVel(const ignition::math::Vector3d &_vel)
    + ***Deprecation:*** virtual void SetAngularVel(const math::Vector3 &_vel)
    + ***Replacement:*** virtual void SetAngularVel(const ignition::math::Vector3d &_vel)
    + ***Deprecation:*** void SetLinearAccel(const math::Vector3 &_accel)
    + ***Replacement:*** void SetLinearAccel(const ignition::math::Vector3d &_accel)
    + ***Deprecation:*** void SetAngularAccel(const math::Vector3 &_accel)
    + ***Replacement:*** void SetAngularAccel(const ignition::math::Vector3d &_accel)
    + ***Deprecation:*** virtual void SetForce(const math::Vector3 &_force)
    + ***Replacement:*** virtual void SetForce(const ignition::math::Vector3d &_force)
    + ***Deprecation:*** virtual void SetTorque(const math::Vector3 &_torque)
    + ***Replacement:***virtual void SetTorque(const ignition::math::Vector3d &_torque)
    + ***Deprecation:*** virtual void AddForce(const math::Vector3 &_force)
    + ***Replacement:*** virtual void AddForce(const ignition::math::Vector3d &_force)
    + ***Deprecation:*** virtual void AddRelativeForce(const math::Vector3 &_force)
    + ***Replacement:*** virtual void AddRelativeForce(const ignition::math::Vector3d &_force)
    + ***Deprecation:*** virtual void AddForceAtWorldPosition(const math::Vector3 &_force, const math::Vector3 &_pos)
    + ***Replacement:*** virtual void AddForceAtWorldPosition(const ignition::math::Vector3d &_force, const ignition::math::Vector3d &_pos)
    + ***Deprecation:*** virtual void AddForceAtRelativePosition( const math::Vector3 &_force, const math::Vector3 &_relPos)
    + ***Replacement:*** virtual void AddForceAtRelativePosition(const ignition::math::Vector3d &_force, const ignition::math::Vector3d &_relPos)
    + ***Deprecation:*** virtual void AddLinkForce(const math::Vector3 &_force, const math::Vector3 &_offset = math::Vector3::Zero)
    + ***Replacement:*** virtual void AddLinkForce(const ignition::math::Vector3d &_force, const ignition::math::Vector3d &_offset = ignition::math::Vector3d::Zero)
    + ***Deprecation:*** virtual void AddTorque(const math::Vector3 &_torque)
    + ***Replacement:*** virtual void AddTorque(const ignition::math::Vector3d &_torque)
    + ***Deprecation:*** virtual void AddRelativeTorque(const math::Vector3 &_torque)
    + ***Replacement:*** virtual void AddRelativeTorque(const ignition::math::Vector3d &_torque)
    + ***Deprecation:*** math::Pose GetWorldCoGPose() const
    + ***Replacement:*** ignition::math::Pose3d WorldCoGPose() const
    + ***Deprecation:*** virtual math::Vector3 GetWorldLinearVel() const
    + ***Replacement:*** virtual ignition::math::Vector3d WorldLinearVel() const
    + ***Deprecation:*** virtual math::Vector3 GetWorldLinearVel(const math::Vector3 &_offset) const
    + ***Replacement:*** virtual ignition::math::Vector3d WorldLinearVel(const ignition::math::Vector3d &_offset) const
    + ***Deprecation:*** virtual math::Vector3 GetWorldLinearVel(const math::Vector3 &_offset, const math::Quaternion &_q) const
    + ***Replacement:***  virtual ignition::math::Vector3d WorldLinearVel(const ignition::math::Vector3d &_offset, const ignition::math::Quaterniond &_q) const
    + ***Deprecation:*** virtual math::Vector3 GetWorldCoGLinearVel() const
    + ***Replacement:*** virtual ignition::math::Vector3d WorldCoGLinearVel() const
    + ***Deprecation:*** math::Vector3 GetRelativeLinearVel() const
    + ***Replacement:*** ignition::math::Vector3d RelativeLinearVel() const
    + ***Deprecation:*** math::Vector3 GetRelativeAngularVel() const
    + ***Replacement:*** ignition::math::Vector3d RelativeAngularVel() const
    + ***Deprecation:*** math::Vector3 GetRelativeLinearAccel() const
    + ***Replacement:*** ignition::math::Vector3d RelativeLinearAccel() const
    + ***Deprecation:*** math::Vector3 GetWorldLinearAccel() const
    + ***Replacement:*** ignition::math::Vector3d WorldLinearAccel() const
    + ***Deprecation:*** math::Vector3 GetRelativeAngularAccel() const
    + ***Replacement:*** ignition::math::Vector3d RelativeAngularAccel() const
    + ***Deprecation:*** math::Vector3 GetWorldAngularMomentum() const
    + ***Replacement:*** ignition::math::Vector3d WorldAngularMomentum() const
    + ***Deprecation:*** math::Vector3 GetWorldAngularAccel() const
    + ***Replacement:*** ignition::math::Vector3d WorldAngularAccel() const
    + ***Deprecation:*** math::Vector3 GetRelativeForce()
    + ***Replacement:*** ignition::math::Vector3d RelativeForce() const
    + ***Deprecation:*** virtual math::Vector3 GetWorldForce() const
    + ***Replacement:*** virtual ignition::math::Vector3d WorldForce() const
    + ***Deprecation:*** math::Vector3 GetRelativeTorque() const
    + ***Replacement:*** ignition::math::Vector3d RelativeTorque() const
    + ***Deprecation:*** virtual math::Vector3 GetWorldTorque() const
    + ***Replacement:*** virtual ignition::math::Vector3d WorldTorque() const
    + ***Deprecation:*** math::Pose GetWorldInertialPose() const
    + ***Replacement:*** ignition::math::Pose3d WorldInertialPose() const
    + ***Deprecation:*** void AttachStaticModel(ModelPtr &_model, const math::Pose &_offset)
    + ***Replacement:*** void AttachStaticModel(ModelPtr &_model, const ignition::math::Pose3d &_offset)

1. **gazebo/physics/MultiRayShape.hh**
    + ***Deprecation:*** public: void MultiRayShape::DisconnectNewLaserScans(ConnectionPtr);
    + ***Replacement:*** Delete the Connection object, perhaps by calling
    reset() on its smart pointer.
    + ***Deprecation:*** math::Angle GetMinAngle() const
    + ***Replacement:*** ignition::math::Angle MinAngle() const
    + ***Deprecation:*** math::Angle GetMaxAngle() const
    + ***Replacement:*** ignition::math::Angle MaxAngle() const
    + ***Deprecation:*** math::Angle GetVerticalMinAngle() const
    + ***Replacement:*** ignition::math::Angle VerticalMinAngle() const
    + ***Deprecation:*** math::Angle GetVerticalMaxAngle() const
    + ***Replacement:*** ignition::math::Angle VerticalMaxAngle() const
    + ***Deprecation:*** void AddRay(const math::Vector3 &_start, const math::Vector3 &_end)
    + ***Replacement:*** void AddRay(const ignition::math::Vector3d &_start, const ignition::math::Vector3d &_end)

1. **gazebo/physics/PhysicsEngine.hh**
    + ***Deprecation:*** virtual void SetGravity(const gazebo::math::Vector3 &_gravity) = 0
    + ***Replacement:*** virtual void SetGravity(const ignition::math::Vector3d &_gravity) = 0

1. **gazebo/physics/PhysicsEngine.hh**
    + ***Deprecation:*** public: math::Vector3 GetGravity const
1. **gazebo/physics/World.hh**
    + ***Replacement:*** public: ignition::math::Vector3 Gravity const

1. **gazebo/physics/PhysicsEngine.hh**
    + ***Deprecation:*** public: ignition::math::Vector3d MagneticField const
1. **gazebo/physics/World.hh**
    + ***Replacement:*** public: ignition::math::Vector3d MagneticField const

1. **gazebo/rendering/Conversions.hh**
    + ***Deprecation:*** public: static Ogre::Quaternion Convert(const math::Quaternion &)
    + ***Replacement:*** public: static Ogre::Quaternion Convert(const ignition::math::Quaterniond &)
    + ***Deprecation:*** public: static Ogre::Vector3 Convert(const math::Vector3 &)
    + ***Replacement:*** public: static Ogre::Vector3 Convert(const ignition::math::Vector3d &)
    + ***Deprecation:*** public: static math::Quaternion Convert(const Ogre::Quaternion &)
    + ***Replacement:*** public: static ignition::math::Quaterniond ConvertIgn(const Ogre::Quaternion &)
    + ***Deprecation:*** public: static math::Vector3 Convert(const Ogre::Vector3 &)
    + ***Replacement:*** public: static ignition::math::Vector3d ConvertIgn(const Ogre::Vector3 &)

1. **gazebo/physics/simbody/SimbodyPhysics.hh**
    + ***Deprecation:*** static SimTK::Quaternion QuadToQuad(const math::Quaternion &_q)
    + ***Replacement:*** static SimTK::Quaternion QuadToQuad(const ignition::math::Quaterniond &_q)
    + ***Deprecation:*** static math::Quaternion QuadToQuad(const SimTK::Quaternion &_q)
    + ***Replacement:*** static ignition::math::Quaterniond QuadToQuadIgn(const SimTK::Quaternion &_q)
    + ***Deprecation:*** static SimTK::Vec3 Vector3ToVec3(const math::Vector3 &_v)
    + ***Replacement:*** static SimTK::Vec3 Vector3ToVec3(const ignition::math::Vector3d &_v)
    + ***Deprecation:*** static math::Vector3 Vec3ToVector3(const SimTK::Vec3 &_v) GAZEBO_DEPRECATED(8.0)
    + ***Replacement:*** static ignition::math::Vector3d Vec3ToVector3Ign(const SimTK::Vec3 &_v)
    + ***Deprecation:*** static SimTK::Transform Pose2Transform(const math::Pose &_pose)
    + ***Replacement:*** static SimTK::Transform Pose2Transform(const ignition::math::Pose3d &_pose)
    + ***Deprecation:*** static math::Pose Transform2Pose(const SimTK::Transform &_xAB)
    + ***Replacement:*** static ignition::math::Pose3d Transform2PoseIgn(const SimTK::Transform &_xAB)

1. **gazebo/physics/dart/DARTTypes.hh**
    + ***Deprecation:*** static Eigen::Vector3d ConvVec3(const math::Vector3 &_vec3)
    + ***Replacement:*** static Eigen::Vector3d ConvVec3(const ignition::math::Vector3d &_vec3)
    + ***Deprecation:*** static math::Vector3 ConvVec3(const Eigen::Vector3d &_vec3)
    + ***Replacement:*** static ignition::math::Vector3d ConvVec3Ign(const Eigen::Vector3d &_vec3)
    + ***Deprecation:*** static Eigen::Quaterniond ConvQuat(const math::Quaternion &_quat)
    + ***Replacement:*** static Eigen::Quaterniond ConvQuat(const ignition::math::Quaterniond &_quat)
    + ***Deprecation:*** static math::Quaternion ConvQuat(const Eigen::Quaterniond &_quat)
    + ***Replacement:*** static ignition::math::Quaterniond ConvQuatIgn(const Eigen::Quaterniond &_quat)
    + ***Deprecation:*** static Eigen::Isometry3d ConvPose(const math::Pose &_pose)
    + ***Replacement:*** static Eigen::Isometry3d ConvPose(const ignition::math::Pose3d &_pose)
    + ***Deprecation:*** static math::Pose ConvPose(const Eigen::Isometry3d &_T)
    + ***Replacement:*** static ignition::math::Pose3d ConvPoseIgn(const Eigen::Isometry3d &_T)

1. **gazebo/physics/dart/DARTCollision.hh**
    + ***Deprecation:*** public: dart::dynamics::Shape *GetDARTCollisionShape() const
    + ***Replacement:*** public: dart::dynamics::ShapePtr DARTCollisionShape() const
    + ***Deprecation:*** public: void SetDARTCollisionShape(dart::dynamics::Shape*,...)
    + ***Replacement:*** public: void SetDARTCollisionShape(dart::dynamics::ShapePtr,...)

1. **gazebo/physics/dart/DARTCylinderShape.hh**
    + ***Deprecation:*** public: DARTCylinderShape(CollisionPtr)
    + ***Replacement:*** public: DARTCylinderShape(DARTCollisionPtr)

1. **gazebo/physics/dart/DARTHeightmapShape.hh**
    + ***Deprecation:*** public: DARTHeightmapShape(CollisionPtr)
    + ***Replacement:*** public: DARTHeightmapShape(DARTCollisionPtr)
    + ***Deprecation:*** public: dart::simulation::World *GetDARTWorld() const
    + ***Replacement:*** public: dart::simulation::WorldPtr DARTWorld() const

1. **gazebo/physics/dart/DARTMesh.hh**
    + ***Deprecation:*** void Init(const common::SubMesh *_subMesh, DARTCollisionPtr _collision, const math::Vector3 &_scale)
    + ***Replacement:*** void Init(const common::SubMesh *_subMesh, DARTCollisionPtr _collision, const ignition::math::Vector3d &_scale)
    + ***Deprecation:*** void Init(const common::Mesh *_mesh, DARTCollisionPtr _collision, const math::Vector3 &_scale)
    + ***Replacement:*** void Init(const common::Mesh *_mesh, DARTCollisionPtr _collision, const ignition::math::Vector3d &_scale)

1. **gazebo/physics/dart/DARTMeshShape.hh**
    + ***Deprecation:*** public: DARTMeshShape(CollisionPtr)
    + ***Replacement:*** public: DARTMeshShape(DARTCollisionPtr)

1. **gazebo/physics/dart/DARTModel.hh**
    + ***Deprecation:*** public: dart::dynamics::Skeleton *GetDARTSkeleton() const
    + ***Replacement:*** public: dart::dynamics::SkeletonPtr DARTSkeleton() const
    + ***Deprecation:*** public: dart::simulation::World *GetDARTWorld() const
    + ***Replacement:*** public: dart::simulation::WorldPtr DARTWorld() const

1. **gazebo/physics/dart/DARTMultiRayShape.hh**
    + ***Deprecation:*** public: DARTMultiRayShape(CollisionPtr)
    + ***Replacement:*** public: DARTMultiRayShape(DARTCollisionPtr)

1. **gazebo/physics/dart/DARTPhysics.hh**
    + ***Deprecation:*** public: dart::simulation::World *GetDARTWorld() const
    + ***Replacement:*** public: dart::simulation::WorldPtr DARTWorld() const

1. **gazebo/physics/dart/DARTPlaneShape.hh**
    + ***Deprecation:*** public: DARTPlaneShape(CollisionPtr)
    + ***Replacement:*** public: DARTPlaneShape(DARTCollisionPtr)

1. **gazebo/physics/ode/ODEMesh.hh**
    + ***Deprecation:*** void Init(const common::SubMesh *_subMesh, ODECollisionPtr _collision, const math::Vector3 &_scale)
    + ***Replacement:*** void Init(const common::SubMesh *_subMesh, ODECollisionPtr _collision, const ignition::math::Vector3d &_scale)
    + ***Deprecation:*** void Init(const common::Mesh *_mesh, ODECollisionPtr _collision, const math::Vector3 &_scale)
    + ***Replacement:*** void Init(const common::Mesh *_mesh, ODECollisionPtr _collision, const ignition::math::Vector3d &_scale)

1. **gazebo/physics/bullet/BulletTypes.hh**
    + ***Deprecation:*** static math::Vector4 ConvertVector4(const btVector4 &_bt)
    + ***Replacement:*** static ignition::math::Vector4d ConvertVector4dIgn(const btVector4 &_bt)
    + ***Deprecation:*** static btVector4 ConvertVector4(const math::Vector4 &_vec)
    + ***Replacement:*** static btVector4 ConvertVector4dIgn(const ignition::math::Vector4d &_vec)
    + ***Deprecation:*** static math::Vector3 ConvertVector3(const btVector3 &_bt)
    + ***Replacement:*** static ignition::math::Vector3d ConvertVector3Ign
    + ***Deprecation:*** static btVector3 ConvertVector3(const math::Vector3 &_vec)
    + ***Replacement:*** static btVector3 ConvertVector3(const ignition::math::Vector3d &_vec)
    + ***Deprecation:*** static math::Pose ConvertPose(const btTransform &_bt)
    + ***Replacement:*** static ignition::math::Pose3d ConvertPoseIgn(const btTransform &_bt)
    + ***Deprecation:*** static math::Pose ConvertPose(const btTransform &_bt)
    + ***Replacement:*** static btTransform ConvertPose(const math::Pose &_pose)

1. **gazebo/physics/bullet/BulletBallJoint.hh**
    + ***Deprecation:*** void Init(const common::SubMesh *_subMesh, BulletCollisionPtr _collision, const math::Vector3 &_scale)
    + ***Replacement:*** void Init(const common::SubMesh *_subMesh, BulletCollisionPtr _collision, const ignition::math::Vector3d &_scale)
    + ***Deprecation:*** void Init(const common::Mesh *_mesh, BulletCollisionPtr _collision, const math::Vector3 &_scale)
    + ***Replacement:*** void Init(const common::Mesh *_mesh, BulletCollisionPtr _collision, const math::Vector3 &_scale)

1. **gazebo/physics/bullet/BulletBallJoint.hh**
    + ***Deprecation:*** virtual math::Vector3 GetAxis(unsigned int _index) const
    + ***Replacement:*** Use Joint::LocalAxis or Joint::GlobalAxis

1. **gazebo/physics/bullet/BulletHinge2Joint.hh**
    + ***Deprecation:*** virtual math::Vector3 GetAxis(unsigned int _index) const
    + ***Replacement:*** Use Joint::LocalAxis or Joint::GlobalAxis

1. **gazebo/physics/simbody/SimbodyBallJoint.hh**
    + ***Deprecation:*** virtual math::Vector3 GetAxis(unsigned int _index) const
    + ***Replacement:*** Use Joint::LocalAxis or Joint::GlobalAxis

1. **gazebo/physics/simbody/SimbodyHinge2Joint.hh**
    + ***Deprecation:*** virtual math::Vector3 GetAxis(unsigned int _index) const
    + ***Replacement:*** Use Joint::LocalAxis or Joint::GlobalAxis

1. **gazebo/physics/simbody/SimbodyUniversalJoint.hh**
    + ***Deprecation:*** virtual math::Vector3 GetAxis(unsigned int _index) const
    + ***Replacement:*** Use Joint::LocalAxis or Joint::GlobalAxis

1. **gazebo/physics/simbody/SimbodyLink.hh**
    + ***Deprecation:*** void SetDirtyPose(const math::Pose &_pose)
    + ***Replacement:*** void SetDirtyPose(const ignition::math::Pose3d &_pose)

1. **gazebo/physics/simbody/SimbodyMesh.hh**
    + ***Deprecation:*** void Init(const common::SubMesh *_subMesh, SimbodyCollisionPtr _collision, const math::Vector3 &_scale)
    + ***Replacement:*** void Init(const common::SubMesh *_subMesh, SimbodyCollisionPtr _collision, const ignition::math::Vector3d &_scale)
    + ***Deprecation:*** void Init(const common::Mesh *_mesh, SimbodyCollisionPtr _collision, const math::Vector3 &_scale)
    + ***Replacement:*** void Init(const common::Mesh *_mesh, SimbodyCollisionPtr _collision, const ignition::math::Vector3d &_scale)

1. **gazebo/rendering/Grid.hh**
    + ***Deprecation:*** public: Ogre::SceneNode *GetSceneNode()
    + ***Replacement:*** public: Ogre::SceneNode *SceneNode() const
    + ***Deprecation:*** public: common::Color GetColor() const
    + ***Replacement:*** public: common::Color Color() const
    + ***Deprecation:*** public: uint32_t GetCellCount() const
    + ***Replacement:*** public: uint32_t CellCount() const
    + ***Deprecation:*** public: float GetCellLength() const
    + ***Replacement:*** public: float CellLength() const
    + ***Deprecation:*** public: float GetLineWidth() const
    + ***Replacement:*** public: float LineWidth() const
    + ***Deprecation:*** public: uint32_t GetHeight() const
    + ***Replacement:*** public: uint32_t Height() const

1. **gazebo/rendering/Camera.hh**
    + ***Deprecation:*** public: void Camera::DisconnectNewImageFrame(ConnectionPtr);
    + ***Replacement:*** Delete the Connection object, perhaps by calling
    reset() on its smart pointer.

1. **gazebo/rendering/DepthCamera.hh**
    + ***Deprecation:*** public: void DepthCamera::DisconnectNewDepthFrame(ConnectionPtr);
    + ***Deprecation:*** public: void DepthCamera::DisconnectNewRGBPointCloud(ConnectionPtr);
    + ***Replacement:*** Delete the Connection object, perhaps by calling
    reset() on its smart pointer.

1. **gazebo/rendering/GpuLaser.hh**
    + ***Deprecation:*** public: void GpuLaser::DisconnectNewLaserFrame(ConnectionPtr);
    + ***Replacement:*** Delete the Connection object, perhaps by calling
    reset() on its smart pointer.

1. **gazebo/sensors/ForceTorqueSensor.hh**
    + ***Deprecation:*** public: void ForceTorqueSensor::DisconnectUpdate(ConnectionPtr);
    + ***Replacement:*** Delete the Connection object, perhaps by calling
    reset() on its smart pointer.

1. **gazebo/sensors/GpuRaySensor.hh**
    + ***Deprecation:*** public: void GpuRaySensor::DisconnectNewLaserFrame(ConnectionPtr);
    + ***Replacement:*** Delete the Connection object, perhaps by calling
    reset() on its smart pointer.

1. **gazebo/sensors/Sensor.hh**
    + ***Deprecation:*** public: void Sensor::DisconnectUpdated(ConnectionPtr);
    + ***Replacement:*** Delete the Connection object, perhaps by calling
    reset() on its smart pointer.

1. **gazebo/sensors/SonarSensor.hh**
    + ***Deprecation:*** public: void SonarSensor::DisconnectUpdate(ConnectionPtr);
    + ***Replacement:*** Delete the Connection object, perhaps by calling
    reset() on its smart pointer.

1. **gazebo/transport/Connection.hh**
    + ***Deprecation:*** public: void Connection::DisconnectShutdown(ConnectionPtr);
    + ***Replacement:*** Delete the Connection object, perhaps by calling
    reset() on its smart pointer.

1. **plugins/events/EventSource.hh**
    + ***Deprecation:*** public: void SimEventConnector::DisconnectSpawnModel(ConnectionPtr);
    + ***Replacement:*** Delete the Connection object, perhaps by calling
    reset() on its smart pointer.

1. **gazebo/math/Filter.hh**
    + ***Deprecation:*** public:   gazebo::math::BiQuad
    + ***Replacement:*** public: ignition::math::BiQuad
    + ***Deprecation:*** public:   gazebo::math::BiQuadVector3
    + ***Replacement:*** public: ignition::math::BiQuadVector3
    + ***Deprecation:*** public:   gazebo::math::Filter
    + ***Replacement:*** public: ignition::math::Filter
    + ***Deprecation:*** public:   gazebo::math::OnePole
    + ***Replacement:*** public: ignition::math::OnePole
    + ***Deprecation:*** public:   gazebo::math::OnePoleQuaternion
    + ***Replacement:*** public: ignition::math::OnePoleQuaternion
    + ***Deprecation:*** public:   gazebo::math::OnePoleVector3
    + ***Replacement:*** public: ignition::math::OnePoleVector3

1. **gazebo/math/Helpers.hh**
    + ***Deprecation:*** GZ_DBL_INF
    + ***Replacement:*** ignition::math::INF_D
    + ***Deprecation:*** GZ_DBL_MIN
    + ***Replacement:*** ignition::math::MIN_D
    + ***Deprecation:*** GZ_DBL_MAX
    + ***Replacement:*** ignition::math::MAX_D
    + ***Deprecation:*** GZ_FLT_MIN
    + ***Replacement:*** ignition::math::MIN_F
    + ***Deprecation:*** GZ_FLT_MAX
    + ***Replacement:*** ignition::math::MAX_F
    + ***Deprecation:*** GZ_INT32_MIN
    + ***Replacement:*** ignition::math::MIN_I32
    + ***Deprecation:*** GZ_INT32_MAX
    + ***Replacement:*** ignition::math::MAX_I32
    + ***Deprecation:*** GZ_UINT32_MIN
    + ***Replacement:*** ignition::math::MIN_UI32
    + ***Deprecation:*** GZ_UINT32_MAX
    + ***Replacement:*** ignition::math::MAX_UI32
    + ***Deprecation:*** static const double gazebo::math::NAN_D
    + ***Replacement:*** static const double ignition::math::NAN_D
    + ***Deprecation:*** static const int gazebo::math::NAN_I
    + ***Replacement:*** static const int ignition::math::NAN_I
    + ***Deprecation:*** public: T   gazebo::math::clamp(T, T, T)
    + ***Replacement:*** public: T ignition::math::clamp(T, T, T)
    + ***Deprecation:*** public: bool   gazebo::math::equal(T, T, T)
    + ***Replacement:*** public: bool ignition::math::equal(T, T, T)
    + ***Deprecation:*** public: double   gazebo::math::fixnan(double)
    + ***Replacement:*** public: double ignition::math::fixnan(double)
    + ***Deprecation:*** public: float   gazebo::math::fixnan(float)
    + ***Replacement:*** public: float ignition::math::fixnan(float)
    + ***Deprecation:*** public: bool   gazebo::math::isnan(double)
    + ***Replacement:*** public: bool ignition::math::isnan(double)
    + ***Deprecation:*** public: bool   gazebo::math::isnan(float)
    + ***Replacement:*** public: bool ignition::math::isnan(float)
    + ***Deprecation:*** public: bool   gazebo::math::isPowerOfTwo(unsigned int)
    + ***Replacement:*** public: bool ignition::math::isPowerOfTwo(unsigned int)
    + ***Deprecation:*** public: T   gazebo::math::max(const std::vector<T> &)
    + ***Replacement:*** public: T ignition::math::max(const std::vector<T> &)
    + ***Deprecation:*** public: T   gazebo::math::mean(const std::vector<T> &)
    + ***Replacement:*** public: T ignition::math::mean(const std::vector<T> &)
    + ***Deprecation:*** public: T   gazebo::math::min(const std::vector<T> &)
    + ***Replacement:*** public: T ignition::math::min(const std::vector<T> &)
    + ***Deprecation:*** public: double   gazebo::math::parseFloat(const std::string &)
    + ***Replacement:*** public: double ignition::math::parseFloat(const std::string &)
    + ***Deprecation:*** public: int   gazebo::math::parseInt(const std::string &)
    + ***Replacement:*** public: int ignition::math::parseInt(const std::string &)
    + ***Deprecation:*** public: T   gazebo::math::precision(const T &, const unsigned int &)
    + ***Replacement:*** public: T ignition::math::precision(const T &, const unsigned int &)
    + ***Deprecation:*** public: unsigned int   gazebo::math::roundUpPowerOfTwo(unsigned int)
    + ***Replacement:*** public: unsigned int ignition::math::roundUpPowerOfTwo(unsigned int)
    + ***Deprecation:*** public: T   gazebo::math::variance(const std::vector<T> &)
    + ***Replacement:*** public: T ignition::math::variance(const std::vector<T> &)

1. **gazebo/math/Box.hh**
    + ***Deprecation:*** public:   gazebo::math::Box
    + ***Replacement:*** public: ignition::math::Box

1. **gazebo/math/Kmeans.hh**
    + ***Deprecation:*** public:   gazebo::math::Kmeans
    + ***Replacement:*** public: ignition::math::Kmeans

1. **gazebo/math/Matrix3.hh**
    + ***Deprecation:*** public:   gazebo::math::Matrix3
    + ***Replacement:*** public: ignition::math::Matrix3

1. **gazebo/math/Matrix4.hh**
    + ***Deprecation:*** public:   gazebo::math::Matrix4
    + ***Replacement:*** public: ignition::math::Matrix4

1. **gazebo/math/Quaternion.hh**
    + ***Deprecation:*** public:   gazebo::math::Quaternion
    + ***Replacement:*** public: ignition::math::Quaterniond

1. **gazebo/math/Plane.hh**
    + ***Deprecation:*** public:   gazebo::math::Plane
    + ***Replacement:*** public: ignition::math::Plane

1. **gazebo/math/Angle.hh**
    + ***Deprecation:*** public:   gazebo::math::Angle
    + ***Replacement:*** public: ignition::math::Angle
    + ***Deprecation:*** GZ_DTOR
    + ***Replacement:*** IGN_DTOR
    + ***Deprecation:*** GZ_RTOD
    + ***Replacement:*** IGN_RTOD
    + ***Deprecation:*** GZ_NORMALIZE
    + ***Replacement:*** IGN_NORMALIZE

1. **gazebo/math/Rand.hh**
    + ***Deprecation:*** public: static double   gazebo::math::GetDblNormal(double, double)
    + ***Replacement:*** public: static double ignition::math::DblNormal(double, double)
    + ***Deprecation:*** public: static int   gazebo::math::GetIntNormal(int, int)
    + ***Replacement:*** public: static int ignition::math::IntNormal(int, int)
    + ***Deprecation:*** public: static double   gazebo::math::GetDblUniform(double, double)
    + ***Replacement:*** public: static double ignition::math::DblUniform(double, double)
    + ***Deprecation:*** public: static int   gazebo::math::GetIntUniform(int, int)
    + ***Replacement:*** public: static int ignition::math::IntUniform(int, int)
    + ***Deprecation:*** public: static       uint32_t gazebo::math::GetSeed()
    + ***Replacement:*** public: static unsigned int ignition::math::Seed()
    + ***Deprecation:*** public: static   void gazebo::math::SetSeed(uint32_t)
    + ***Replacement:*** public: static void ignition::math::Seed(unsigned int)

1. **gazebo/math/RotationSpline.hh**
    + ***Deprecation:*** public:   gazebo::math::RotationSpline
    + ***Replacement:*** public: ignition::math::RotationSpline

1. **gazebo/math/SignalStats.hh**
    + ***Deprecation:*** public:   gazebo::math::SignalStatistic
    + ***Replacement:*** public: ignition::math::SignalStatistic
    + ***Deprecation:*** public:   gazebo::math::SignalStats
    + ***Replacement:*** public: ignition::math::SignalStats

1. **gazebo/math/Spline.hh**
    + ***Deprecation:*** public:   gazebo::math::Spline
    + ***Replacement:*** public: ignition::math::Spline

1. **gazebo/math/Vector2d.hh**
    + ***Deprecation:*** public:   gazebo::math::Vector2d
    + ***Replacement:*** public: ignition::math::Vector2d

1. **gazebo/math/Vector2i.hh**
    + ***Deprecation:*** public:   gazebo::math::Vector2i
    + ***Replacement:*** public: ignition::math::Vector2i

1. **gazebo/math/Vector3Stats.hh**
    + ***Deprecation:*** public:   gazebo::math::Vector3Stats
    + ***Replacement:*** public: ignition::math::Vector3Stats

1. **gazebo/math/Vector4.hh**
    + ***Deprecation:*** public:   gazebo::math::Vector4
    + ***Replacement:*** public: ignition::math::Vector4d

### Deletions

1. **gazebo/common/Event.hh**
    + ConnectionPrivate class
    + Connection() constructor
    + EventPrivate class
    + Event(EventPrivate&) constructor

1. **gazebo/gui/EntityMaker.hh**
    + EntityMakerPrivate class
    + Entity(EntityMakerPrivate&) constructor
    + EntityMakerPrivate *dataPtr

1. **gazebo/physics/Link.hh**
    + std::vector<std::string> cgVisuals

## Gazebo 7.10.0 to 7.X

### Modifications

1. Shadows ambient factor has been reduced - they will now appear darker than before. Also increased shadow texture resolution and reduced effect of jagged shadow edges.
   Please see [BitBucket pull request 2805](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2805)
   for more details.

## Gazebo 7.9.0 to 7.X

### Modifications

1. **gazebo/physics/ode/ODEPhysics.cc**
   `ODEPhysics::Collide` combines surface slip parameters with a sum
   instead of `std::min`.
   Please see [BitBucket pull request 2717](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2717)
   for more details.

## Gazebo 7.8.0 to 7.X

### Modifications

1. **gz log**
   Gazebo log files no longer store velocity data and have reduced floating point precision.
   See [BitBucket pull request 2715](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2715/add-log-record-filter-options)
   for further details.

## Gazebo 7.3.1 to 7.X

### Deprecations

1. **gazebo/sensors/ImuSensor.hh**
    + ***Deprecation:** public: void SetWorldToReferencePose(const ignition::math::Pose3d &)
    + ***Replacement:** public: void SetWorldToReferenceOrientation(const ignition::math::Quaterniond &)

## Gazebo 7.1.0 to 7.X

### Additions

1. **gazebo/physics/ode/ODEJoint.hh**
    + public: virtual void Fini();

1. **gazebo/physics/bullet/BulletJoint.hh**
    + public: virtual void Fini();

### Deprecations

1. **gazebo::common::VisualPlugin**
    The custom inner xml inside visual plugins used to be wrapped in an extra
    <sdf> tag. Now the inner xml should be accessed directly from the plugin's
    sdf. For example, for the following plugin:

          <visual ...>
            <plugin ...>
              <param>true</param>
            </plugin>
          </visual>

     <param> should be accessed with:

          auto param = _sdf->GetElement("param");

     The old behaviour is still supported on Gazebo7, that is:

          auto param = _sdf->GetElement("sdf")->GetElement("param");

     but this behaviour will be removed on Gazebo8.

    + [BitBucket pull request #2394](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2394)

## Gazebo 6.X to 7.X

### Additions

1. **gazebo/physics/Model.hh**
    + public: gazebo::physics::JointPtr CreateJoint(
        const std::string &_name, const std::string &_type,
        physics::LinkPtr _parent, physics::LinkPtr _child);
    + public: bool RemoveJoint(const std::string &_name);
    + public: boost::shared_ptr<Model> shared_from_this();

1. **gazebo/physics/SurfaceParams.hh**
    + public: double PoissonsRatio() const;
    + public: void SetPoissonsRatio(double _ratio);
    + public: double ElasticModulus() const;
    + public: void SetElasticModulus(double _modulus);

### Modifications

1. **gazebo/sensor/SensorTypes.hh**
    + All `typedef`'s of `shared_ptr`'s to Sensor's are changed
      from `boost::shared_ptr` to `std::shared_ptr`.
      Any downstream code that does a pointer cast
      (such as `dynamic_pointer_cast` or `static_pointer_cast`)
      will need to switch from `boost::*_pointer_cast` to `std::*_pointer_cast`.
    + [BitBucket pull request #2079](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2079)

1. **gazebo/sensors/Sensor.hh**
    + ***Removed:*** public: template<typename T> event::ConnectionPtr ConnectUpdated(T _subscriber);
    + ***Replacement:*** public: event::ConnectionPtr ConnectUpdated(std::function<void()> _subscriber);

1. **gazebo/rendering/GpuLaser.hh**
    + ***Removed:*** public: void SetCameraCount(double _cameraCount);
    + ***Replacement:*** public: void SetCameraCount(const unsigned int _cameraCount);
    + ***Removed:*** public: template<typename T> event::ConnectionPtr ConnectNewLaserFrame(T _subscriber);
    + ***Replacement:*** public: event::ConnectionPtr ConnectNewLaserFrame(std::function<void (const float *_frame, unsigned int _width, unsigned int _height, unsigned int _depth, const std::string &_format)> _subscriber);

1. **gazebo/rendering/DepthCamera.hh**
    + ***Removed:*** public: template<typename T> event::ConnectionPtr ConnectNewDepthFrame(T _subscriber)
    + ***Replacement:*** public: event::ConnectionPtr ConnectNewDepthFrame(std::function<void (const float *, unsigned int, unsigned int, unsigned int, const std::string &)>  _subscriber);
    + ***Removed:*** public: template<typename T> event::ConnectionPtr ConnectNewRGBPointCloud(T _subscriber)
    + ***Replacement:*** public: event::ConnectionPtr ConnectNewRGBPointCloud(std::function<void (const float *, unsigned int, unsigned int, unsigned int, const std::string &)>  _subscriber);

1. **gazebo/physics/Actor.hh**
    + Type change of `protected: math::Vector3 lastPos;` to `protected: ignition::math::Vector3d lastPos;`

1. **gazebo/physics/ContactManager.hh**
    + Remove contact filters with names that contain `::`.
      The `CreateFilter`, `HasFilter`, and `RemoveFilter` functions
      now convert `::` strings to `/` in the filter name before acting.
      These were not being deleted properly in previous versions.

1. **gazebo/rendering/RenderTypes.hh**
    + typedefs for Visual and its derived classes have been changed from boost to std pointers.
    + [BitBucket pull request #1924](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1924)

1. **gazebo/gui/model/ModelEditorEvents.hh**
    + ***Removed:*** public: static event::EventT<void (bool, bool, const math::Pose &, const std::string &)> modelPropertiesChanged
    + ***Replacement:*** public: static event::EventT<void (bool, bool)> modelPropertiesChanged
    + ***Note:*** Removed last two arguments, model pose and name, from the function

1. **gazebo/rendering/Camera.hh**
    + ***Removed:*** public: void SetClipDist();
    + ***Replacement:*** public: virtual void SetClipDist();
    + ***Removed:*** public: template<typename T> event::ConnectionPtr ConnectNewImageFrame(T _subscriber);
    + ***Replacement:*** public: event::ConnectionPtr ConnectNewImageFrame(std::function<void (const unsigned char *, unsigned int, unsigned int, unsigned int, const std::string &)> _subscriber);

1. **gazebo/msgs/logical_camera_sensors.proto**
    + The `near` and `far` members have been replaced with `near_clip` and `far_clip`
    + [BitBucket pull request #1942](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1942)

1. **Light topic**
    + ***Removed:*** ~/light
    + ***Replacement:*** ~/factory/light - for spawning new lights
    + ***Replacement:*** ~/light/modify - for modifying existing lights
    * [BitBucket pull request #1920](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1920)

1. **gazebo/rendering/Visual.hh**
    + ***Removed:*** public: void SetVisible(bool _visible, bool _cascade = true);
    + ***Replacement:*** public: virtual void SetVisible(bool _visible, bool _cascade = true);

1. **gazebo/rendering/OribitViewController.hh**
    + ***Removed:*** public: OrbitViewController(UserCameraPtr _camera);
    + ***Replacement:*** public: OrbitViewController(UserCameraPtr _camera, const std::string &_name = "OrbitViewController");

1. **gazebo/test/ServerFixture.hh**
    + ***Removed:*** protected: void RunServer(const std::string &_worldFilename);
    + ***Removed:*** protected: void RunServer(const std::string &_worldFilename,
      bool _paused, const std::string &_physics, const std::vector<std::string> &
      _systemPlugins = {});
    + ***Replacement:*** void ServerFixture::RunServer(const std::vector<:string>
      &_args)
    * [BitBucket pull request #1874](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1874)

1. **gazebo/gui/building/BuildingMaker.hh**
    * Doesn't inherit from gui::EntityMaker anymore
    * [BitBucket pull request #1828](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1828)

1. **gazebo/gui/EntityMaker.hh**
    + ***Removed:*** EntityMaker();
    + ***Replacement:*** EntityMaker(EntityMakerPrivate &_dataPtr);
    + ***Removed:*** public: virtual void Start(const rendering::UserCameraPtr _camera) = 0;
    + ***Replacement:*** public: virtual void Start();
    + ***Removed:*** public: virtual void Stop() = 0;
    + ***Replacement:*** public: virtual void Stop();

1. **gazebo/gui/ModelAlign.hh**
    + ***Removed:*** public: void AlignVisuals(std::vector<rendering::VisualPtr> _visuals, const std::string &_axis, const std::string &_config, const std::string &_target, bool _publish = true);
    + ***Replacement:*** public: void AlignVisuals(std::vector<rendering::VisualPtr> _visuals, const std::string &_axis, const std::string &_config, const std::string &_target, bool _publish = true, const bool _inverted = false);

1. **gazebo/gui/GuiEvents.hh**
    + ***Removed:*** public: static event::EventT<void (std::string, std::string, std::string, bool)> alignMode; std::string, std::string, bool)> alignMode;
    + ***Replacement:*** public: static event::EventT<void (std::string, std::string, std::string, bool)> alignMode; std::string, std::string, bool, bool)> alignMode;

### Deprecations

1. **gazebo/util/OpenAL.hh**
    + ***Deprecation:*** public: bool GetOnContact() const;
    + ***Replacement:*** public: bool OnContact() const;
    + ***Deprecation:*** public: std::vector<std::string> GetCollisionNames() const;
    + ***Replacement:*** public: std::vector<std::string> CollisionNames() const;

1. **gazebo/util/LogRecord.hh**
    + ***Deprecation:*** public: bool GetPaused() const;
    + ***Replacement:*** public: bool Paused() const;
    + ***Deprecation:*** public: bool GetRunning() const;
    + ***Replacement:*** public: bool Running() const;
    + ***Deprecation:*** public: const std::string &GetEncoding() const;
    + ***Replacement:*** public: const std::string &Encoding() const;
    + ***Deprecation:*** public: std::string GetFilename(const std::string &_name = "") const;
    + ***Replacement:*** public: std::string Filename(const std::string &_name = "") const;
    + ***Deprecation:*** public: unsigned int GetFileSize(const std::string &_name = "") const
    + ***Replacement:*** public: unsigned int FileSize(const std::string &_name = "") const;
    + ***Deprecation:*** public: std::string GetBasePath() const;
    + ***Replacement:*** public: std::string BasePath() const;
    + ***Deprecation:*** public: common::Time GetRunTime() const;
    + ***Replacement:*** public: common::Time RunTime() const;
    + ***Deprecation:*** public: bool GetFirstUpdate() const;
    + ***Replacement:*** public: bool FirstUpdate() const;
    + ***Deprecation:*** public: unsigned int GetBufferSize() const;
    + ***Replacement:*** public: unsigned int BufferSize() const;

1. **gazebo/rendering/Scene.hh**
    + ***Deprecation:*** public: Ogre::SceneManager *GetManager() const;
    + ***Replacement:*** public: Ogre::SceneManager *OgreSceneManager() const;
    + ***Deprecation:*** public: std::string GetName() const;
    + ***Replacement:*** public: std::string Name() const;
    + ***Deprecation:*** public: common::Color GetAmbientColor() const;
    + ***Replacement:*** public: common::Color AmbientColor() const;
    + ***Deprecation:*** public: common::Color GetBackgroundColor();
    + ***Replacement:*** public: common::Color BackgroundColor() const;
    + ***Deprecation:*** public: uint32_t GetGridCount();
    + ***Replacement:*** public: uint32_t GridCount() const;
    + ***Deprecation:*** public: uint32_t GetOculusCameraCount() const;
    + ***Replacement:*** public: uint32_t OculusCameraCount() const;
    + ***Deprecation:*** public: uint32_t GetCameraCount() const;
    + ***Replacement:*** public: uint32_t CameraCount() const;
    + ***Deprecation:*** public: uint32_t GetUserCameraCount() const;
    + ***Replacement:*** public: uint32_t UserCameraCount() const;
    + ***Deprecation:*** public: uint32_t GetLightCount() const;
    + ***Replacement:*** public: uint32_t LightCount() const;
    + ***Deprecation:*** public: VisualPtr GetVisualAt(CameraPtr _camera, const math::Vector2i &_mousePos, std::string &_mod)
    + ***Replacement:*** public: VisualPtr VisualAt(CameraPtr _camera, const ignition::math::Vector2i &_mousePos, std::string &_mod);
    + ***Deprecation:*** public: VisualPtr GetVisualAt(CameraPtr _camera, const math::Vector2i &_mousePos)
    + ***Replacement:*** public: VisualPtr VisualAt(CameraPtr _camera, const ignition::math::Vector2i &_mousePos);
    + ***Deprecation:*** public: VisualPtr GetVisualBelow(const std::string &_visualName);
    + ***Replacement:*** public: VisualPtr VisualBelow(const std::string &_visualName);
    + ***Deprecation:*** public: void GetVisualsBelowPoint(const math::Vector3 &_pt, std::vector<VisualPtr> &_visuals);
    + ***Replacement:*** public: void VisualsBelowPoint(const ignition::math::Vector3d &_pt, std::vector<VisualPtr> &_visuals);
    + ***Deprecation:*** public: double GetHeightBelowPoint(const math::Vector3 &_pt);
    + ***Replacement:*** public: double HeightBelowPoint(const ignition::math::Vector3d &_pt);
    + ***Deprecation:*** public: bool GetFirstContact(CameraPtr _camera, const math::Vector2i &_mousePos, math::Vector3 &_position)
    + ***Replacement:*** public: bool FirstContact(CameraPtr _camera, const ignition::math::Vector2i &_mousePos, ignition::math::Vector3d &_position);
    + ***Deprecation:*** public: void DrawLine(const math::Vector3 &_start, const math::Vector3 &_end, const std::string &_name)
    + ***Replacement:*** public: void DrawLine(const ignition::math::Vector3d &_start, const ignition::math::Vector3d &_end, const std::string &_name);
    + ***Deprecation:*** public: uint32_t GetId() const;
    + ***Replacement:*** public: uint32_t Id() const;
    + ***Deprecation:*** public: std::string GetIdString() const;
    + ***Replacement:*** public: std::string IdString() const;
    + ***Deprecation:*** public: bool GetShadowsEnabled() const;
    + ***Replacement:*** public: bool ShadowsEnabled() const;
    + ***Deprecation:*** public: VisualPtr GetWorldVisual() const;
    + ***Replacement:*** public: VisualPtr WorldVisual() const;
    + ***Deprecation:*** public: VisualPtr GetSelectedVisual() const;
    + ***Replacement:*** public: VisualPtr SelectedVisual() const;
    + ***Deprecation:*** public: bool GetShowClouds() const;
    + ***Replacement:*** public: bool ShowClouds() const;
    + ***Deprecation:*** public: bool GetInitialized() const;
    + ***Replacement:*** public: bool Initialized() const;
    + ***Deprecation:*** public: common::Time GetSimTime() const;
    + ***Replacement:*** public: common::Time SimTime() const;
    + ***Deprecation:*** public: uint32_t GetVisualCount() const
    + ***Replacement:*** public: uint32_t VisualCount() const;

1. **gazebo/rendering/DepthCamera.hh**
    + ***Deprecation:*** public: virtual const float *GetDepthData();
    + ***Replacement:*** public: virtual const float *DepthData() const;

1. **gazebo/rendering/Heightmap.hh**
    + ***Deprecation:*** public: double GetHeight(double _x, double _y, double _z = 1000);
    + ***Replacement:*** public: double Height(const double _x, const double _y, const double _z = 1000) const;
    + ***Deprecation:*** public: bool Flatten(CameraPtr _camera, math::Vector2i _mousePos, double _outsideRadius, double _insideRadius, double _weight = 0.1)
    + ***Replacement:*** public: bool Flatten(CameraPtr _camera, const ignition::math::Vector2i &_mousePos, const double _outsideRadius, const double _insideRadius, const double _weight = 0.1);
    + ***Deprecation:*** public: bool Smooth(CameraPtr _camera, math::Vector2i _mousePos, double _outsideRadius, double _insideRadius, double _weight = 0.1);
    + ***Replacement:*** public: bool Smooth(CameraPtr _camera, const ignition::math::Vector2i &_mousePos, const double _outsideRadius, const double _insideRadius, const double _weight = 0.1);
    + ***Deprecation:*** public: bool Raise(CameraPtr _camera, math::Vector2i _mousePos,
 double _outsideRadius, double _insideRadius, double _weight = 0.1)
    + ***Replacement:*** public: bool Raise(CameraPtr _camera, const ignition::math::Vector2i &_mousePos, const double _outsideRadius, const double _insideRadius, const double _weight = 0.1)
    + ***Deprecation:*** public: bool Lower(CameraPtr _camera, math::Vector2i _mousePos, double _outsideRadius, double _insideRadius, double _weight = 0.1)
    + ***Replacement:*** public: bool Lower(CameraPtr _camera, const ignition::math::Vector2i &_mousePos, const double _outsideRadius, const double _insideRadius, const double _weight = 0.1)
    + ***Deprecation:*** public: double GetAvgHeight(Ogre::Vector3 _pos, double _brushSize);
    + ***Replacement:*** public: double AvgHeight(const ignition::math::Vector3d &_pos, const double _brushSize) const
    + ***Deprecation:*** public: Ogre::TerrainGroup *GetOgreTerrain() const;
    + ***Replacement:*** public: Ogre::TerrainGroup *OgreTerrain() const;
    + ***Deprecation:*** public: common::Image GetImage() const;
    + ***Replacement:*** public: public: common::Image Image() const;
    + ***Deprecation:*** public: Ogre::TerrainGroup::RayResult GetMouseHit(CameraPtr _camera, math::Vector2i _mousePos);
    + ***Replacement:*** public: Ogre::TerrainGroup::RayResult MouseHit(CameraPtr _camera, const ignition::math::Vector2i &_mousePos) const;
    + ***Deprecation:*** public: unsigned int GetTerrainSubdivisionCount() const;
    + ***Replacement:*** public: unsigned int TerrainSubdivisionCount() const;

1. **gazebo/rendering/RenderEngine.hh**
    + ***Deprecation:*** public: unsigned int GetSceneCount() const;
    + ***Replacement:*** public: unsigned int SceneCount() const;
    + ***Deprecation:*** public: Ogre::OverlaySystem *GetOverlaySystem() const;
    + ***Replacement:*** public: Ogre::OverlaySystem *OverlaySystem() const;

1. **gazebo/rendering/GpuLaser.hh**
    + ***Deprecation:*** public: const float *GetLaserData();
    + ***Replacement:*** public: const float *LaserData() const;
    + ***Deprecation:*** public: double GetHorzHalfAngle() const;
    + ***Replacement:*** public: double HorzHalfAngle() const;
    + ***Deprecation:*** public: double GetVertHalfAngle() const;
    + ***Replacement:*** public: double VertHalfAngle() const;
    + ***Deprecation:*** public: double GetHorzFOV() const;
    + ***Replacement:*** public: double HorzFOV() const;
    + ***Deprecation:*** public: double GetCosHorzFOV() const;
    + ***Replacement:*** public: double CosHorzFOV() const;
    + ***Deprecation:*** public: double GetVertFOV() const;
    + ***Replacement:*** public: double VertFOV() const;
    + ***Deprecation:*** public: double GetCosVertFOV() const;
    + ***Replacement:*** public: double CosVertFOV() const;
    + ***Deprecation:*** public: double GetNearClip() const;
    + ***Replacement:*** public: double NearClip() const;
    + ***Deprecation:*** public: double GetFarClip() const;
    + ***Replacement:*** public: double FarClip() const;
    + ***Deprecation:*** public: double CameraCount() const;
    + ***Replacement:*** public: unsigned int CameraCount() const;
    + ***Deprecation:*** public: double GetRayCountRatio() const;
    + ***Replacement:*** public: double RayCountRatio() const;

1. **gazebo/rendering/DynamicLines.hh**
    + ***Deprecation:*** public: void AddPoint(const math::Vector3 &_pt,const common::Color &_color = common::Color::White)
    + ***Replacement:*** public: void AddPoint(const ignition::math::Vector3d &_pt,const common::Color &_color = common::Color::White);
    + ***Deprecation:*** public: void SetPoint(unsigned int _index, const math::Vector3 &_value)
    + ***Replacement:*** public: void SetPoint(unsigned int _index,const ignition::math::Vector3d &_value);
    + ***Deprecation:*** public: math::Vector3 GetPoint(unsigned int _index) const
    + ***Replacement:*** public: ignition::math::Vector3d Point(const unsigned int _index) const;

1. **gazebo/rendering/WindowManager.hh**
    + ***Deprecation:*** public: uint32_t GetAvgFPS(uint32_t _id);
    + ***Replacement:*** public: uint32_t AvgFPS(const uint32_t _id) const;
    + ***Deprecation:*** public: uint32_t GetTriangleCount(uint32_t _id);
    + ***Replacement:*** public: uint32_t TriangleCount(const uint32_t _id) const;
    + ***Deprecation:*** public: Ogre::RenderWindow *GetWindow(uint32_t _id);
    + ***Replacement:*** public: Ogre::RenderWindow *Window(const uint32_t _id) const;

1. **gazebo/rendering/Light.hh**
    + ***Deprecation:*** public: std::string GetName() const;
    + ***Replacement:*** public: std::string Name() const;
    + ***Deprecation:*** public: std::string GetType() const;
    + ***Replacement:*** public: std::string Type() const;
    + ***Deprecation:*** public: public: void SetPosition(const math::Vector3 &_p);
    + ***Replacement:*** public: void SetPosition(const ignition::math::Vector3d &_p);
    + ***Deprecation:*** public: math::Vector3 GetPosition();
    + ***Replacement:*** public: ignition::math::Vector3d Position() const;
    + ***Deprecation:*** public: void SetRotation(const math::Quaternion &_q);
    + ***Replacement:*** public: void SetRotation(const ignition::math::Quaterniond &_q);
    + ***Deprecation:*** public: math::Quaternion GetRotation() const;
    + ***Replacement:*** public: ignition::math::Quaterniond Rotation() const;
    + ***Deprecation:*** public: bool GetVisible() const;
    + ***Replacement:*** public: bool Visible() const;
    + ***Deprecation:*** public: common::Color GetDiffuseColor() const;
    + ***Replacement:*** public: common::Color DiffuseColor() const;
    + ***Deprecation:*** public: common::Color GetSpecularColor() const;
    + ***Replacement:*** public: common::Color SpecularColor() const;
    + ***Deprecation:*** public: void SetDirection(const math::Vector3 &_dir);
    + ***Replacement:*** public: void SetDirection(const ignition::math::Vector3d &_dir);
    + ***Deprecation:*** public: math::Vector3 GetDirection() const;
    + ***Replacement:*** public: ignition::math::Vector3d Direction() const;

1. **gazebo/util/Diagnostics.hh**
    + ***Deprecation:*** public: int GetTimerCount() const;
    + ***Replacement:*** public: int TimerCount() const;
    + ***Deprecation:*** public: common::Time GetTime(int _index) const;
    + ***Replacement:*** public: common::Time Time(const int _index) const;
    + ***Deprecation:*** public: common::Time GetTime(const std::string &_label) const;
    + ***Replacement:*** public: common::Time Time(const std::string &_label) const;
    + ***Deprecation:*** public: std::string GetLabel(int _index) const;
    + ***Replacement:*** public: std::string Label(const int _index) const;
    + ***Deprecation:*** public: boost::filesystem::path GetLogPath() const
    + ***Replacement:*** public: boost::filesystem::path LogPath() const;

1. **gazebo/sensors/CameraSensor.hh**
    + ***Deprecation:** public: virtual std::string GetTopic() const;
    + ***Replacement:** public: virtual std::string Topic() const;
    + ***Deprecation:** public: rendering::CameraPtr GetCamera() const
    + ***Replacement:** public: rendering::CameraPtr Camera() const;
    + ***Deprecation:** public: unsigned int GetImageWidth() const;
    + ***Replacement:** public: unsigned int ImageWidth() const;
    + ***Deprecation:** public: unsigned int GetImageHeight() const;
    + ***Replacement:** public: unsigned int ImageHeight() const;
    + ***Deprecation:** public: const unsigned char *GetImageData();
    + ***Replacement:** const unsigned char *ImageData() const;

1. **gazebo/util/LogPlay.hh**
    + ***Deprecation:*** public: std::string GetLogVersion() const;
    + ***Replacement:*** public: std::string LogVersion() const;
    + ***Deprecation:*** public: std::string GetGazeboVersion() const;
    + ***Replacement:*** public: std::string GazeboVersion() const;
    + ***Deprecation:*** public: uint32_t GetRandSeed() const
    + ***Replacement:*** public: uint32_t RandSeed() const;
    + ***Deprecation:*** public: common::Time GetLogStartTime() const;
    + ***Replacement:*** public: common::Time LogStartTime() const;
    + ***Deprecation:*** public: common::Time GetLogEndTime() const;
    + ***Replacement:*** public: common::Time LogEndTime() const;
    + ***Deprecation:*** public: std::string GetFilename() const;
    + ***Replacement:*** public: std::string Filename() const;
    + ***Deprecation:*** public: std::string GetFullPathFilename() const;
    + ***Replacement:*** public: std::string FullPathFilename() const;
    + ***Deprecation:*** public: uintmax_t GetFileSize() const
    + ***Replacement:*** public: uintmax_t FileSize() const;
    + ***Deprecation:*** public: unsigned int GetChunkCount() const;
    + ***Replacement:*** public: unsigned int ChunkCount() const;
    + ***Deprecation:*** public: bool GetChunk(unsigned int _index, std::string &_data);
    + ***Replacement:*** public: bool Chunk(const unsigned int _index, std::string &_data) const;
    + ***Deprecation:*** public: std::string GetEncoding() const
    + ***Replacement:*** public: std::string Encoding() const;
    + ***Deprecation:*** public: std::string GetHeader() const
    + ***Replacement:*** public: std::string Header() const;
    + ***Deprecation:*** public: uint64_t GetInitialIterations() const
    + ***Replacement:*** public: uint64_t InitialIterations() const;

1. **gazebo/sensors/ContactSensor.hh**
    + ***Deprecation:** public: msgs::Contacts GetContacts() const;
    + ***Replacement:** public: msgs::Contacts Contacts() const;
    + ***Deprecation:** public: std::map<std::string, physics::Contact> GetContacts(const std::string &_collisionName);
    + ***Replacement:** public: std::map<std::string, physics::Contact> Contacts(const std::string &_collisionName) const;

1. **gazebo/sensors/DepthCameraSensor.hh**
    + ***Deprecation:** public: rendering::DepthCameraPtr GetDepthCamera() const
    + ***Replacement:** public: rendering::DepthCameraPtr DepthCamera() const;

1. **gazebo/sensors/ForceTorqueSensor.hh**
    + ***Deprecation:** public: virtual std::string GetTopic() const
    + ***Replacement:** public: virtual std::string Topic() const;
    + ***Deprecation:** public: physics::JointPtr GetJoint() const;
    + ***Replacement:** public: physics::JointPtr Joint() const;

1. **gazebo/sensors/GpsSensor.hh**
    + ***Deprecation:** public: double GetAltitude();
    + ***Replacement:** public: double Altitude() const;

1. **gazebo/sensors/GpuRaySensor.hh**
    + ***Deprecation:** public: virtual std::string GetTopic() const;
    + ***Replacement:** public: virtual std::string Topic() const;
    + ***Deprecation:** public: rendering::GpuLaserPtr GetLaserCamera() const
    + ***Replacement:** public: rendering::GpuLaserPtr LaserCamera() const;
    + ***Deprecation:** public: double GetAngleResolution() const
    + ***Replacement:** public: double AngleResolution() const;
    + ***Deprecation:** public: double GetRangeMin() const
    + ***Replacement:** public: double RangeMin() const;
    + ***Deprecation:** public: double GetRangeMax() const
    + ***Replacement:** public: double RangeMax() const;
    + ***Deprecation:** public: double GetRangeResolution() const
    + ***Replacement:** public: double RangeResolution() const;
    + ***Deprecation:** public: int GetRayCount() const
    + ***Replacement:** public: int RayCount() const;
    + ***Deprecation:** public: int GetRangeCount() const
    + ***Replacement:** public: int RangeCount() const;
    + ***Deprecation:** public: int GetVerticalRayCount() const
    + ***Replacement:** public: int VerticalRayCount()
    + ***Deprecation:** public: int GetVerticalRangeCount() const;
    + ***Replacement:** public: int VerticalRangeCount() const;
    + ***Deprecation:** public: double GetVerticalAngleResolution() const
    + ***Replacement:** public: double VerticalAngleResolution() const;
    + ***Deprecation:** public: double GetRange(int _index)
    + ***Replacement:** public: double Range(const int _index) const;
    + ***Deprecation:** public: void GetRanges(std::vector<double> &_ranges)
    + ***Replacement:** public: void Ranges(std::vector<double> &_ranges) const
    + ***Deprecation:** public: double GetRetro(int _index) const
    + ***Replacement:** public: double Retro(const int _index) const;
    + ***Deprecation:** public: int GetFiducial(int _index) const
    + ***Replacement:** public: int Fiducial(const unsigned int _index) const;
    + ***Deprecation:** public: unsigned int GetCameraCount() const
    + ***Replacement:** public: unsigned int CameraCount() const;
    + ***Deprecation:** public: double GetRayCountRatio()
    + ***Replacement:** public: double RayCountRatio() const;
    + ***Deprecation:** public: double GetRangeCountRatio() const
    + ***Replacement:** public: double RangeCountRatio() const;
    + ***Deprecation:** public: double GetHorzFOV() const
    + ***Replacement:** public: double HorzFOV() const;
    + ***Deprecation:** public: double GetCosHorzFOV() const
    + ***Replacement:** public: double CosHorzFOV() const;
    + ***Deprecation:** public: double GetVertFOV() const
    + ***Replacement:** public: double VertFOV() const;
    + ***Deprecation:** public: double GetCosVertFOV() const
    + ***Replacement:** public: double CosVertFOV() const;
    + ***Deprecation:** public: double GetHorzHalfAngle() const
    + ***Replacement:** public: double HorzHalfAngle() const;
    + ***Deprecation:** public: double GetVertHalfAngle() const
    + ***Replacement:** public: double VertHalfAngle() const;

1. **gazebo/sensors/ImuSensor.hh**
    + ***Deprecation:** public: msgs::IMU GetImuMessage() const
    + ***Replacement:** public: msgs::IMU ImuMessage() const;

1. **gazebo/sensors/MultiCameraSensor.hh**
    + ***Deprecation:** public: virtual std::string GetTopic() const;
    + ***Replacement:** public: virtual std::string Topic() const;
    + ***Deprecation:** public: unsigned int GetCameraCount() const
    + ***Replacement:** public: unsigned int CameraCount() const;
    + ***Deprecation:** public: rendering::CameraPtr GetCamera(unsigned int _index) const
    + ***Replacement:** public: rendering::CameraPtr Camera(const unsigned int _index) const;
    + ***Deprecation:** public: unsigned int GetImageWidth(unsigned int _index) const
    + ***Replacement:** public: unsigned int ImageWidth(const unsigned int _index) const;
    + ***Deprecation:** public: unsigned int GetImageHeight(unsigned int _index) const
    + ***Replacement:** public: unsigned int ImageHeight(const unsigned int _index) const;
    + ***Deprecation:** public: const unsigned char *GetImageData(unsigned int _index)
    + ***Replacement:** public: const unsigned char *ImageData(const unsigned int _index);

1. **gazebo/sensors/RaySensor.hh**
    + ***Deprecation:** public: virtual std::string GetTopic() const;
    + ***Replacement:** public: virtual std::string Topic() const;
    + ***Deprecation:** public: double GetAngleResolution() const
    + ***Replacement:** public: double AngleResolution() const;
    + ***Deprecation:** public: double GetRangeMin() const
    + ***Replacement:** public: double RangeMin() const;
    + ***Deprecation:** public: double GetRangeMax() const
    + ***Replacement:** public: double RangeMax() const;
    + ***Deprecation:** public: double GetRangeResolution() const
    + ***Replacement:** public: double RangeResolution() const;
    + ***Deprecation:** public: int GetRayCount() const
    + ***Replacement:** public: int RayCount() const;
    + ***Deprecation:** public: int GetRangeCount() const
    + ***Replacement:** public: int RangeCount() const;
    + ***Deprecation:** public: int GetVerticalRayCount() const
    + ***Replacement:** public: int VerticalRayCount() const
    + ***Deprecation:** public: int GetVerticalRangeCount() const
    + ***Replacement:** public: int VerticalRangeCount() const;
    + ***Deprecation:** public: double GetVerticalAngleResolution() const
    + ***Replacement:** public: double VerticalAngleResolution() const;
    + ***Deprecation:** public: double GetRange(unsigned int _index)
    + ***Replacement:** public: double Range(const unsigned int _index) const;
    + ***Deprecation:** public: void GetRanges(std::vector<double> &_ranges)
    + ***Replacement:** public: void Ranges(std::vector<double> &_ranges) const;
    + ***Deprecation:** public: double GetRetro(unsigned int _index)
    + ***Replacement:** public: double Retro(const unsigned int _index) const;
    + ***Deprecation:** public: int GetFiducial(unsigned int _index)
    + ***Replacement:** public: int Fiducial(const unsigned int _index) const;
    + ***Deprecation:** public: physics::MultiRayShapePtr GetLaserShape()
    + ***Replacement:** public: physics::MultiRayShapePtr LaserShape() const;

1. **gazebo/sensors/Sensor.hh**
    + ***Deprecation:** public: std::string GetParentName() const
    + ***Replacement:** public: std::string ParentName() const;
    + ***Deprecation:** public: double GetUpdateRate()
    + ***Replacement:** public: double UpdateRate() const;
    + ***Deprecation:** public: std::string GetName() const
    + ***Replacement:** public: std::string Name() const;
    + ***Deprecation:** public: std::string GetScopedName() const
    + ***Replacement:** public: std::string ScopedName() const;
    + ***Deprecation:** public: std::string GetType() const
    + ***Replacement:** public: std::string Type() const;
    + ***Deprecation:** public: common::Time GetLastUpdateTime()
    + ***Replacement:** public: common::Time LastUpdateTime() const;
    + ***Deprecation:** public: common::Time GetLastMeasurementTime()
    + ***Replacement:** public: common::Time LastMeasurementTime() const;
    + ***Deprecation:** public: bool GetVisualize() const
    + ***Replacement:** public: bool Visualize() const;
    + ***Deprecation:** public: virtual std::string GetTopic() const
    + ***Replacement:** public: virtual std::string Topic() const;
    + ***Deprecation:** public: std::string GetWorldName() const
    + ***Replacement:** public: std::string WorldName() const;
    + ***Deprecation:** public: SensorCategory GetCategory() const
    + ***Replacement:** public: SensorCategory Category() const;
    + ***Deprecation:** public: uint32_t GetId() const
    + ***Replacement:** public: uint32_t Id() const;
    + ***Deprecation:** public: uint32_t GetParentId() const
    + ***Replacement:** public: uint32_t ParentId() const;
    + ***Deprecation:** public: NoisePtr GetNoise(const SensorNoiseType _type) const
    + ***Replacement:** public: NoisePtr Noise(const SensorNoiseType _type) const;

1. **gazebo/sensors/SonarSensor.hh**
    + ***Deprecation:** public: virtual std::string GetTopic() const;
    + ***Replacement:** public: virtual std::string Topic() const;
    + ***Deprecation:** public: double GetRangeMin() const
    + ***Replacement:** public: double RangeMin() const;
    + ***Deprecation:** public: double GetRangeMax() const
    + ***Replacement:** public: double RangeMax() const;
    + ***Deprecation:** public: double GetRadius() const
    + ***Replacement:** public: double Radius() const;
    + ***Deprecation:** public: double GetRange()
    + ***Replacement:** public: double Range();

1. **gazebo/sensors/WirelessReceiver.hh**
    + ***Deprecation:** public: double GetMinFreqFiltered() const
    + ***Replacement:** public: double MinFreqFiltered() const;
    + ***Deprecation:** public: double GetMaxFreqFiltered() const
    + ***Replacement:** public: double MaxFreqFiltered() const;
    + ***Deprecation:** public: double GetSensitivity() const
    + ***Replacement:** public: double Sensitivity() const;

1. **gazebo/sensors/WirelessTransceiver.hh**
    + ***Deprecation:** public: virtual std::string GetTopic() const;
    + ***Replacement:** public: virtual std::string Topic() const;
    + ***Deprecation:** public: double GetGain() const
    + ***Replacement:** public: double Gain() const;
    + ***Deprecation:** public: double GetPower() const
    + ***Replacement:** public: double Power() const;

1. **gazebo/sensors/WirelessTransmitter.hh**
    + ***Deprecation:** public: std::string GetESSID() const
    + ***Replacement:** public: std::string ESSID() const;
    + ***Deprecation:** public: double GetFreq() const
    + ***Replacement:** public: double Freq() const;

1. **gazebo/rendering/ApplyWrenchVisual.hh**
    + ***Deprecation:*** public: void SetCoM(const math::Vector3 &_comVector)
    + ***Replacement:*** public: void SetCoM(const ignition::math::Vector3d &_comVector);
    + ***Deprecation:*** public: void SetForcePos(const math::Vector3 &_forcePosVector)
    + ***Replacement:*** public: void SetForcePos(const ignition::math::Vector3d &_forcePosVector);
    + ***Deprecation:*** public: void SetForce(const math::Vector3 &_forceVector,const bool _rotatedByMouse);
    + ***Replacement:*** public: void SetForce(const ignition::math::Vector3d &_forceVector, const bool _rotatedByMouse);
    + ***Deprecation:*** public: void SetTorque(const math::Vector3 &_torqueVector,const bool _rotatedByMouse);
    + ***Replacement:*** public: void SetTorque(const ignition::math::Vector3d &_torqueVector, const bool _rotatedByMouse);

1. **gazebo/rendering/AxisVisual.hh**
    + ***Deprecation:*** public: void ScaleXAxis(const math::Vector3 &_scale)
    + ***Replacement:*** public: void ScaleXAxis(const ignition::math::Vector3d &_scale);
    + ***Deprecation:*** public: void ScaleYAxis(const math::Vector3 &_scale)
    + ***Replacement:*** public: void ScaleYAxis(const ignition::math::Vector3d &_scale);
    + ***Deprecation:*** public: void ScaleZAxis(const math::Vector3 &_scale)
    + ***Replacement:*** public: void ScaleZAxis(const ignition::math::Vector3d &_scale);

1. **gazebo/gui/CloneWindow.hh**
    + ***Deprecation:*** int GetPort()
    + ***Replacement:*** int Port() const

1. **gazebo/gui/ConfigWidget.hh**
    + ***Deprecation:*** public: google::protobuf::Message *GetMsg()
    + ***Replacement:*** public: google::protobuf::Message *Msg()
    + ***Deprecation:*** public: std::string GetHumanReadableKey(const std::string &_key)
    + ***Replacement:*** public: std::string HumanReadableKey(const std::string &_key) const
    + ***Deprecation:*** public: std::string GetUnitFromKey(const std::string &_key, const std::string &_jointType = "")
    + ***Replacement:*** public: std::string UnitFromKey(const std::string &_key, const std::string &_jointType = "") const
    + ***Deprecation:*** public: void GetRangeFromKey(const std::string &_key, double &_min, double &_max)
    + ***Replacement:*** public: void RangeFromKey(const std::string &_key, double &_min, double &_max) const
    + ***Deprecation:*** public: bool GetWidgetVisible(const std::string &_name)
    + ***Replacement:*** public: bool WidgetVisible(const std::string &_name) const
    + ***Deprecation:*** public: bool GetWidgetReadOnly(const std::string &_name) const
    + ***Replacement:*** public: bool WidgetReadOnly(const std::string &_name) const
    + ***Deprecation:*** public: bool SetVector3WidgetValue(const std::string &_name, const math::Vector3 &_value)
    + ***Replacement:*** public: bool SetVector3dWidgetValue(const std::string &_name, const ignition::math::Vector3d &_value)
    + ***Deprecation:*** public: bool SetPoseWidgetValue(const std::string &_name, const math::Pose &_value)
    + ***Replacement:*** public: bool SetPoseWidgetValue(const std::string &_name, const ignition::math::Pose3d &_value)
    + ***Deprecation:*** public: bool SetGeometryWidgetValue(const std::string &_name, const std::string &_value, const math::Vector3 &_dimensions, const std::string &_uri = "")
    + ***Replacement:*** public: bool SetGeometryWidgetValue(const std::string &_name, const std::string &_value, const ignition::math::Vector3d &_dimensions, const std::string &_uri = "")
    + ***Deprecation:*** public: int GetIntWidgetValue(const std::string &_name) const
    + ***Replacement:*** public: int IntWidgetValue(const std::string &_name) const
    + ***Deprecation:*** public: unsigned int GetUIntWidgetValue(const std::string &_name) const
    + ***Replacement:*** public: unsigned int UIntWidgetValue(const std::string &_name) const
    + ***Deprecation:*** public: double GetDoubleWidgetValue(const std::string &_name) const
    + ***Replacement:*** public: double DoubleWidgetValue(const std::string &_name) const
    + ***Deprecation:*** public: bool GetBoolWidgetValue(const std::string &_name) const
    + ***Replacement:*** public: bool BoolWidgetValue(const std::string &_name) const
    + ***Deprecation:*** public: std::string GetStringWidgetValue(const std::string &_name) const
    + ***Replacement:*** public: std::string StringWidgetValue(const std::string &_name) const
    + ***Deprecation:*** public: math::Vector3 GetVector3WidgetValue(const std::string &_name)
    + ***Replacement:*** public: ignition::math::Vector3d Vector3dWidgetValue(const std::string &_name) const
    + ***Deprecation:*** public: common::Color GetColorWidgetValue(const std::string &_name) const
    + ***Replacement:*** public: common::Color ColorWidgetValue(const std::string &_name) const
    + ***Deprecation:*** public: math::Pose GetPoseWidgetValue(const std::string &_name) const
    + ***Replacement:*** public: ignition::math::Pose3d PoseWidgetValue(const std::string &_name) const
    + ***Deprecation:*** public: std::string GetGeometryWidgetValue(const std::string &_name, math::Vector3 &_dimensions, std::string &_uri) const
    + ***Replacement:*** public: std::string GeometryWidgetValue(const std::string &_name, ignition::math::Vector3d &_dimensions, std::string &_uri) const
    + ***Deprecation:*** public: std::string GetEnumWidgetValue(const std::string &_name) const
    + ***Replacement:*** public: std::string EnumWidgetValue(const std::string &_name) const

1. **gazebo/gui/GLWidget.hh**
    + ***Deprecation:*** rendering::UserCameraPtr GetCamera() const
    + ***Replacement:*** rendering::UserCameraPtr Camera() const
    + ***Deprecation:*** rendering::ScenePtr GetScene() const
    + ***Replacement:*** rendering::ScenePtr Scene() const

1. **gazebo/gui/KeyEventHandler.hh**
    + ***Deprecation:*** bool GetAutoRepeat() const
    + ***Replacement:*** bool AutoRepeat() const

1. **gazebo/gui/MainWindow.hh**
    + ***Deprecation:*** gui::RenderWidget *GetRenderWidget() const
    + ***Replacement:*** gui::RenderWidget *RenderWidget() const
    + ***Deprecation:*** gui::Editor *GetEditor(const std::string &_name) const
    + ***Replacement:*** gui::Editor *Editor(const std::string &_name) const

1. **gazebo/rendering/Camera.hh**
    + ***Deprecation:*** public: DistortionPtr GetDistortion() const;
    + ***Replacement:*** public: DistortionPtr LensDistortion() const;
    + ***Deprecation:*** public: double GetRenderRate() const;
    + ***Replacement:*** public: double RenderRate() const;
    + ***Deprecation:*** public: bool GetInitialized() const;
    + ***Replacement:*** public: bool Initialized() const;
    + ***Deprecation:*** public: unsigned int GetWindowId() const;
    + ***Replacement:*** public: unsigned int WindowId() const;
    + ***Deprecation:*** public: math::Vector3 GetWorldPosition() const
    + ***Replacement:*** public: ignition::math::Vector3d WorldPosition() const;
    + ***Deprecation:*** public: math::Quaternion GetWorldRotation() const
    + ***Replacement:*** public: ignition::math::Quaterniond WorldRotation() const;
    + ***Deprecation:*** public: virtual void SetWorldPose(const math::Pose &_pose)
    + ***Replacement:*** public: virtual void SetWorldPose(const ignition::math::Pose3d &_pose);
    + ***Deprecation:***  public: math::Pose GetWorldPose() const
    + ***Replacement:*** public: ignition::math::Pose3d WorldPose() const;
    + ***Deprecation:*** public: void SetWorldPosition(const math::Vector3 &_pos);
    + ***Replacement:*** public: void SetWorldPosition(const ignition::math::Vector3d &_pos);
    + ***Deprecation:*** public: void SetWorldRotation(const math::Quaternion &_quat);
    + ***Replacement:*** public: void SetWorldRotation(const ignition::math::Quaterniond &_quat);
    + ***Deprecation:*** public: void Translate(const math::Vector3 &_direction)
    + ***Replacement:*** public: void Translate(const ignition::math::Vector3d &_direction);
    + ***Deprecation:***  public: void Roll(const math::Angle &_angle, Ogre::Node::TransformSpace _relativeTo =Ogre::Node::TS_LOCAL);
    + ***Replacement:*** public: void Roll(const ignition::math::Angle &_angle, ReferenceFrame _relativeTo = RF_LOCAL);
    + ***Deprecation:***  public: void Pitch(const math::Angle &_angle, Ogre::Node::TransformSpace _relativeTo =Ogre::Node::TS_LOCAL);
    + ***Replacement:*** public: void Pitch(const ignition::math::Angle &_angle, ReferenceFrame _relativeTo = RF_LOCAL);
    + ***Deprecation:***  public: void Yaw(const math::Angle &_angle, Ogre::Node::TransformSpace _relativeTo =Ogre::Node::TS_WORLD);
    + ***Replacement:*** public: void Yaw(const ignition::math::Angle &_angle, ReferenceFrame _relativeTo = RF_WORLD);
    + ***Deprecation:*** public: void SetHFOV(math::Angle _angle);
    + ***Replacement:*** public: void SetHFOV(const ignition::math::Angle &_angle);
    + ***Deprecation:*** public: math::Angle GetHFOV() const
    + ***Replacement:*** public: ignition::math::Angle HFOV() const;
    + ***Deprecation:*** public: math::Angle GetVFOV() const;
    + ***Replacement:*** public: ignition::math::Angle VFOV() const;
    + ***Deprecation:*** public: virtual unsigned int GetImageWidth() const;
    + ***Replacement:*** public: virtual unsigned int ImageWidth() const;
    + ***Deprecation:*** public: unsigned int GetTextureWidth() const;
    + ***Replacement:*** public: unsigned int TextureWidth() const;
    + ***Deprecation:*** public: virtual unsigned int GetImageHeight() const;
    + ***Replacement:*** public: virtual unsigned int ImageHeight() const;
    + ***Deprecation:*** public: unsigned int GetImageDepth() const;
    + ***Replacement:*** public: unsigned int ImageDepth() const;
    + ***Deprecation:*** public: std::string GetImageFormat() const;
    + ***Replacement:*** public: std::string ImageFormat() const;
    + ***Deprecation:*** public: unsigned int GetTextureHeight() const;
    + ***Replacement:*** public: unsigned int TextureHeight() const;
    + ***Deprecation:*** public: size_t GetImageByteSize() const;
    + ***Replacement:*** public: size_t ImageByteSize() const;
    + ***Deprecation:*** public: static size_t GetImageByteSize(unsigned int _width, unsigned int _height, const std::string &_format);
    + ***Replacement:*** static size_t ImageByteSize(const unsigned int _width, const unsigned int _height, const std::string &_format);
    + ***Deprecation:*** public: double GetZValue(int _x, int _y);
    + ***Replacement:*** public: double ZValue(const int _x, const int _y);
    + ***Deprecation:*** public: double GetNearClip();
    + ***Replacement:*** public: double NearClip() const;
    + ***Deprecation:*** public: double GetFarClip();
    + ***Replacement:*** public: double FarClip() const;
    + ***Deprecation:*** public: bool GetCaptureData() const;
    + ***Replacement:*** public: bool CaptureData() const;
    + ***Deprecation:*** public: Ogre::Camera *GetOgreCamera() const;
    + ***Replacement:*** public: Ogre::Camera *OgreCamera() const;
    + ***Deprecation:*** public: Ogre::Viewport *GetViewport() const;
    + ***Replacement:*** public: Ogre::Viewport *OgreViewport() const;
    + ***Deprecation:*** public: unsigned int GetViewportWidth() const;
    + ***Replacement:*** public: unsigned int ViewportWidth() const;
    + ***Deprecation:*** public: unsigned int GetViewportHeight() const;
    + ***Replacement:*** public: unsigned int ViewportHeight() const;
    + ***Deprecation:*** public: math::Vector3 GetUp();
    + ***Replacement:*** public: ignition::math::Vector3d Up() const;
    + ***Deprecation:*** public: math::Vector3 GetRight();
    + ***Replacement:*** public: ignition::math::Vector3d Right() const;
    + ***Deprecation:*** public: virtual float GetAvgFPS() const;
    + ***Replacement:*** public: virtual float AvgFPS() const;
    + ***Deprecation:*** public: virtual unsigned int GetTriangleCount() const;
    + ***Replacement:*** public: virtual unsigned int TriangleCount() const;
    + ***Deprecation:*** public: float GetAspectRatio() const;
    + ***Replacement:*** public: float AspectRatio() const;
    + ***Deprecation:*** public: Ogre::SceneNode *GetSceneNode() const;
    + ***Replacement:*** public: Ogre::SceneNode *SceneNode() const;
    + ***Deprecation:*** public: virtual const unsigned char *GetImageData(unsigned int i = 0);
    + ***Replacement:*** public: virtual const unsigned char *ImageData(unsigned int i = 0) const;
    + ***Deprecation:*** public: std::string GetName() const;
    + ***Replacement:*** public: std::string Name() const;
    + ***Deprecation:*** public: std::string GetScopedName() const;
    + ***Replacement:*** public: std::string ScopedName() const;
    + ***Deprecation:*** public: void GetCameraToViewportRay(int _screenx, int _screeny,math::Vector3 &_origin, math::Vector3 &_dir);
    + ***Replacement:*** public: void CameraToViewportRay(const int _screenx, const int _screeny,ignition::math::Vector3d &_origin,ignition::math::Vector3d &_dir) const;
    + ***Deprecation:*** public: bool GetWorldPointOnPlane(int _x, int _y,const math::Plane &_plane, math::Vector3 &_result);
    + ***Replacement:*** public: bool WorldPointOnPlane(const int _x, const int _y, const ignition::math::Planed &_plane,ignition::math::Vector3d &_result);
    + ***Deprecation:*** public: Ogre::Texture *GetRenderTexture() const;
    + ***Replacement:*** public: Ogre::Texture *RenderTexture() const;
    + ***Deprecation:*** public: math::Vector3 GetDirection();
    + ***Replacement:*** public: ignition::math::Vector3d Direction() const;
    + ***Deprecation:*** public: common::Time GetLastRenderWallTime();
    + ***Replacement:*** public: common::Time LastRenderWallTime() const;
    + ***Deprecation:*** public: virtual bool MoveToPosition(const math::Pose &_pose,double _time);
    + ***Replacement:***  public: virtual bool MoveToPosition(const ignition::math::Pose3d &_pose,double _time);
    + ***Deprecation:*** public: bool MoveToPositions(const std::vector<math::Pose> &_pts,double _time,boost::function<void()> _onComplete = NULL);
    + ***Replacement:*** public: bool MoveToPositions(const std::vector<ignition::math::Pose3d> &_pts,double _time,boost::function<void()> _onComplete = NULL);
    + ***Deprecation:*** public: std::string GetScreenshotPath() const;
    + ***Replacement:*** public: std::string ScreenshotPath() const;
    + ***Deprecation:*** public: std::string GetProjectionType() const;
    + ***Replacement:*** public: std::string ProjectionType() const;

1. **gazebo/gui/RTShaderSystem.hh**
    + ***Deprecation:*** void AttachEntity(Visual *vis)
    + ***No replacement for AttachEntity ***

1. **gazebo/gui/RTShaderSystem.hh**
    + ***Deprecation:*** void DetachEntity(Visual *_vis)
    + ***No replacement for DetachEntity ***

1. **gazebo/physics/Model.hh**
    + ***Deprecation:*** public: void SetScale(const math::Vector3 &_scale);
    + ***Replacement:*** public: void SetScale(const ignition::math::Vector3d &_scale, const bool _publish = false);

### Deletions

1. **plugins/rest_web/RestUiLogoutDialog.hh.hh**

1. **gazebo rendering libraries**
    * The following libraries have been removed: `libgazebo_skyx`, `libgazebo_selection_buffer`, `libgazebo_rendering_deferred`. Gazebo now combines all the different rendering libraries into `libgazebo_rendering.so`.
    * [BitBucket pull request #1817](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1817)

1. **gazebo physics libraries**
    * The following libraries have been removed: `libgazebo_ode_physics`, `libgazebo_simbody_physics`, `libgazebo_dart_physics`, and `libgazebo_bullet_physics`. Gazebo now combines all the different physics engine libraries into `libgazebo_physics.so`.
    * [BitBucket pull request #1814](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1814)

1. **gazebo/gui/BoxMaker.hh**

1. **gazebo/gui/CylinderMaker.hh**

1. **gazebo/gui/SphereMaker.hh**

1. **gazebo/gui/MeshMaker.hh**

1. **gazebo/gui/EntityMaker.hh**
    + public: typedef boost::function<void(const math::Vector3 &pos,
                  const math::Vector3 &scale)> CreateCallback;
    + public: static void SetSnapToGrid(bool _snap);
    + public: virtual bool IsActive() const = 0;
    + public: virtual void OnMousePush(const common::MouseEvent &_event);
    + public: virtual void OnMouseDrag(const common::MouseEvent &_event);
    + protected: math::Vector3 GetSnappedPoint(math::Vector3 _p);

1. **gazebo/sensors/ForceTorqueSensor.hh**
    + public: math::Vector3 GetTorque() const
    + public: math::Vector3 GetForce() const

1. **gazebo/sensors/GpsSensor.hh**
    + public: math::Angle GetLongitude()
    + public: math::Angle GetLatitude()

1. **gazebo/sensors/GpuRaySensor.hh**
    + public: math::Angle GetAngleMin() const
    + public: math::Angle GetAngleMax() const
    + public: math::Angle GetVerticalAngleMin() const
    + public: math::Angle GetVerticalAngleMax() const

1. **gazebo/sensors/ImuSensor.hh**
    + public: math::Vector3 GetAngularVelocity() const
    + public: math::Vector3 GetLinearAcceleration() const
    + public: math::Quaternion GetOrientation() const

1. **gazebo/sensors/RFIDSensor.hh**
    + private: bool CheckTagRange(const math::Pose &_pose)

1. **gazebo/sensors/RFIDTag.hh**
    + public: math::Pose GetTagPose() const

1. **gazebo/sensors/RaySensor.hh**
    + public: math::Angle GetAngleMin() const
    + public: math::Angle GetAngleMax() const
    + public: math::Angle GetVerticalAngleMin() const
    + public: math::Angle GetVerticalAngleMax() const

1. **gazebo/sensors/Sensor.hh**
    + public: virtual math::Pose GetPose() const
    + public: NoisePtr GetNoise(unsigned int _index = 0) const

1. **gazebo/sensors/WirelessTransmitter.hh**
    + public: double GetSignalStrength(const math::Pose &_receiver, const double _rxGain)

## Gazebo 5.X to 6.X

### Deprecations

1. **gazebo/common/Color.hh**
    + ***Deprecation:*** math::Vector3 GetAsHSV() const;
    + ***Replacement:*** ignition::math::Vector3d HSV() const;

1. **gazebo/common/Dem.hh**
    + ***Deprecation:*** void GetGeoReferenceOrigin(math::Angle &_latitude,math::Angle &_longitude);
    + ***Replacement:*** void GetGeoReferenceOrigin(ignition::math::Angle &_latitude,  ignition::math::Angle &_longitude) const;
    + ***Deprecation:***void FillHeightMap(int _subSampling, unsigned int _vertSize, const math::Vector3 &_size, const math::Vector3 &_scale, bool _flipY, std::vector<float> &_heights);
    + ***Replacement:***void FillHeightMap(const int _subSampling, const unsigned int _vertSize, const ignition::math::Vector3d &_size, const ignition::math::Vector3d &_scale, const bool _flipY, std::vector<float> &_heights);

1. **gazebo/common/GTSMeshUtils.hh**
    + ***Deprecation:***static bool DelaunayTriangulation(const std::vector<math::Vector2d> &_vertices, const std::vector<math::Vector2i> &_edges, SubMesh *_submesh);
    + ***Replacement:***static bool DelaunayTriangulation( const std::vector<ignition::math::Vector2d> &_vertices, const std::vector<ignition::math::Vector2i> &_edges, SubMesh *_submesh);

1. **gazebo/common/HeightmapData.hh**
    + ***Deprecation:***virtual void FillHeightMap(int _subSampling,unsigned int _vertSize, const math::Vector3 &_size,const math::Vector3 &_scale, bool _flipY, std::vector<float> &_heights);
    + ***Replacement:***void FillHeightMap(int _subSampling,unsigned int _vertSize, const ignition::math::Vector3d &_size,const ignition::math::Vector3d &_scale, bool _flipY,std::vector<float> &_heights);

1. **gazebo/common/KeyFrame.hh**
    + ***Deprecation:***void SetTranslation(const math::Vector3 &_trans);
    + ***Replacement:***void Translation(const ignition::math::Vector3d &_trans);
    + ***Deprecation:***math::Vector3 GetTranslation() const;
    + ***Replacement:***ignition::math::Vector3d Translation() const;
    + ***Deprecation:***void SetRotation(const math::Quaternion &_rot);
    + ***Replacement:***void Rotation(const ignition::math::Quaterniond &_rot);
    + ***Deprecation:***math::Quaternion GetRotation();
    + ***Replacement:***ignition::math::Quaterniond Rotation() const;

1. **gazebo/common/Mesh.hh**
    + ***Deprecation:***math::Vector3 GetMax() const;
    + ***Replacement:***ignition::math::Vector3d Max() const;
    + ***Deprecation:***math::Vector3 GetMin() const;
    + ***Replacement:***ignition::math::Vector3d Min() const;
    + ***Deprecation:***void GetAABB(math::Vector3 &_center, math::Vector3 &_min_xyz,math::Vector3 &_max_xyz) const;
    + ***Replacement:***void GetAABB(ignition::math::Vector3d &_center,ignition::math::Vector3d &_minXYZ,ignition::math::Vector3d &_maxXYZ) const;
    + ***Deprecation:***void GenSphericalTexCoord(const math::Vector3 &_center);
    + ***Replacement:***void GenSphericalTexCoord(const ignition::math::Vector3d &_center);
    + ***Deprecation:***void SetScale(const math::Vector3 &_factor);
    + ***Replacement:***void SetScale(const ignition::math::Vector3d &_factor);
    + ***Deprecation:***void Center(const math::Vector3 &_center = math::Vector3::Zero);
    + ***Replacement:***void Center(const ignition::math::Vector3d &_center =ignition::math::Vector3d::Zero);
    + ***Deprecation:***void Translate(const math::Vector3 &_vec);
    + ***Replacement:***void Translate(const ignition::math::Vector3d &_vec);
    + ***Deprecation:*** void CopyVertices(const std::vector<math::Vector3> &_verts);
    + ***Replacement:***void CopyVertices(const std::vector<ignition::math::Vector3d> &_verts);
    + ***Deprecation:***void CopyNormals(const std::vector<math::Vector3> &_norms);
    + ***Replacement:***void CopyNormals( const std::vector<ignition::math::Vector3d> &_norms);
    + ***Deprecation:***void AddVertex(const math::Vector3 &_v);
    + ***Replacement:***void AddVertex(const ignition::math::Vector3d &_v);
    + ***Deprecation:***void AddNormal(const math::Vector3 &_n);
    + ***Replacement:***void AddNormal(const ignition::math::Vector3d &_n);
    + ***Deprecation:***math::Vector3 GetVertex(unsigned int _i) const;
    + ***Replacement:***ignition::math::Vector3d Vertex(unsigned int _i) const;
    + ***Deprecation:***void SetVertex(unsigned int _i, const math::Vector3 &_v);
    + ***Replacement:***void SetVertex(unsigned int _i,const ignition::math::Vector3d &_v);
    + ***Deprecation:***math::Vector3 GetNormal(unsigned int _i) const;
    + ***Replacement:***ignition::math::Vector3d Normal(unsigned int _i) const;
    + ***Deprecation:***void SetNormal(unsigned int _i, const math::Vector3 &_n);
    + ***Replacement:***void SetNormal(unsigned int _i,const ignition::math::Vector3d &_n);
    + ***Deprecation:***math::Vector2d GetTexCoord(unsigned int _i) const;
    + ***Replacement:***ignition::math::Vector2d TexCoord(unsigned int _i) const;
    + ***Deprecation:***void SetTexCoord(unsigned int _i, const math::Vector2d &_t);
    + ***Replacement:***void SetTexCoord(unsigned int _i,const ignition::math::Vector2d &_t);
    + ***Deprecation:***math::Vector3 GetMax() const;
    + ***Replacement:***ignition::math::Vector3d Max() const;
    + ***Deprecation:***math::Vector3 GetMin() const;
    + ***Replacement:***ignition::math::Vector3d Min() const;
    + ***Deprecation:***bool HasVertex(const math::Vector3 &_v) const;
    + ***Replacement:***bool HasVertex(const ignition::math::Vector3d &_v) const;
    + ***Deprecation:***unsigned int GetVertexIndex(const math::Vector3 &_v) const;
    + ***Replacement:***unsigned int GetVertexIndex( const ignition::math::Vector3d &_v) const;
    + ***Deprecation:***void GenSphericalTexCoord(const math::Vector3 &_center);
    + ***Replacement:***void GenSphericalTexCoord(const ignition::math::Vector3d &_center);
    + ***Deprecation:***void Center(const math::Vector3 &_center = math::Vector3::Zero);
    + ***Replacement:***void Center(const ignition::math::Vector3d &_center =ignition::math::Vector3d::Zero);
    + ***Deprecation:***void Translate(const math::Vector3 &_vec) ;
    + ***Replacement:***void Translate(const ignition::math::Vector3d &_vec);
    + ***Deprecation:***void SetScale(const math::Vector3 &_factor);
    + ***Replacement:***void SetScale(const ignition::math::Vector3d &_factor);

1. **gazebo/common/MeshCSG.hh**
    + ***Deprecation:***Mesh *CreateBoolean(const Mesh *_m1, const Mesh *_m2,const int _operation, const math::Pose &_offset = math::Pose::Zero);
    + ***Replacement:***Mesh *CreateBoolean(const Mesh *_m1, const Mesh *_m2,const int _operation,const ignition::math::Pose3d &_offset = ignition::math::Pose3d::Zero);

1. **gazebo/common/MeshManager.hh**
    + ***Deprecation:***void GetMeshAABB(const Mesh *_mesh,math::Vector3 &_center,math::Vector3 &_minXYZ,math::Vector3 &_maxXYZ);
    + ***Replacement:***void GetMeshAABB(const Mesh *_mesh,ignition::math::Vector3d &_center,ignition::math::Vector3d &_min_xyz,ignition::math::Vector3d &_max_xyz);
    + ***Deprecation:***void GenSphericalTexCoord(const Mesh *_mesh,math::Vector3 _center);
    + ***Replacement:*** void GenSphericalTexCoord(const Mesh *_mesh,const ignition::math::Vector3d &_center);
    + ***Deprecation:***void CreateBox(const std::string &_name, const math::Vector3 &_sides,const math::Vector2d &_uvCoords);
    + ***Replacement:***void CreateBox(const std::string &_name,const ignition::math::Vector3d &_sides,const ignition::math::Vector2d &_uvCoords);
    + ***Deprecation:***void CreateExtrudedPolyline(const std::string &_name, const std::vector<std::vector<math::Vector2d> > &_vertices,double _height);
    + ***Replacement:*** void CreateExtrudedPolyline(const std::string &_name,const std::vector<std::vector<ignition::math::Vector2d> > &_vertices, double _height);
    + ***Deprecation:***void CreatePlane(const std::string &_name,const math::Plane &_plane,const math::Vector2d &_segments,const math::Vector2d &_uvTile);
    + ***Replacement:***void CreatePlane(const std::string &_name,const ignition::math::Planed &_plane,const ignition::math::Vector2d &_segments, const ignition::math::Vector2d &_uvTile);
    + ***Deprecation:***void CreatePlane(const std::string &_name,const math::Vector3 &_normal,double _d,const math::Vector2d &_size,const math::Vector2d &_segments,const math::Vector2d &_uvTile);
    + ***Replacement:***void CreatePlane(const std::string &_name,const ignition::math::Vector3d &_normal,const double _d,const ignition::math::Vector2d &_size,const ignition::math::Vector2d &_segments, const ignition::math::Vector2d &_uvTile);
    + ***Deprecation:***void CreateBoolean(const std::string &_name, const Mesh *_m1,const Mesh *_m2, const int _operation,const math::Pose &_offset = math::Pose::Zero);
    + ***Replacement:***void CreateBoolean(const std::string &_name, const Mesh *_m1,const Mesh *_m2, const int _operation,const ignition::math::Pose3d &_offset = ignition::math::Pose3d::Zero);

1. **gazebo/common/SVGLoader.hh**
    + ***Deprecation:***static void PathsToClosedPolylines(const std::vector<common::SVGPath> &_paths, double _tol,std::vector< std::vector<math::Vector2d> > &_closedPolys,std::vector< std::vector<math::Vector2d> > &_openPolys);
    + ***Replacement:***static void PathsToClosedPolylines(const std::vector<common::SVGPath> &_paths,double _tol,std::vector< std::vector<ignition::math::Vector2d> > &_closedPolys,std::vector< std::vector<ignition::math::Vector2d> > &_openPolys);

1. **gazebo/common/Skeleton.hh**
    + ***Deprecation:***void SetBindShapeTransform(math::Matrix4 _trans);
    + ***Replacement:***void SetBindShapeTransform(const ignition::math::Matrix4d &_trans);
    + ***Deprecation:***math::Matrix4 GetBindShapeTransform();
    + ***Replacement:***ignition::math::Matrix4d BindShapeTransform();
    + ***Deprecation:***void SetTransform(math::Matrix4 _trans,bool _updateChildren = true);
    + ***Replacement:***void SetTransform(const ignition::math::Matrix4d &_trans,bool _updateChildren = true);
    + ***Deprecation:***void SetModelTransform(math::Matrix4 _trans,bool _updateChildren = true);
    + ***Replacement:***void SetModelTransform(const ignition::math::Matrix4d &_trans,bool _updateChildren = true);
    + ***Deprecation:***void SetInitialTransform(math::Matrix4 _tras);
    + ***Replacement:***void SetInitialTransform(const ignition::math::Matrix4d &_tras);
    + ***Deprecation:***math::Matrix4 GetTransform();
    + ***Replacement:***ignition::math::Matrix4d Transform();
    + ***Deprecation:***void SetInverseBindTransform(math::Matrix4 _invBM);
    + ***Replacement:***void SetInverseBindTransform(const ignition::math::Matrix4d &_invBM);
    + ***Deprecation:***math::Matrix4 GetInverseBindTransform();
    + ***Replacement:***ignition::math::Matrix4d InverseBindTransform();
    + ***Deprecation:***math::Matrix4 GetModelTransform();
    + ***Replacement:***ignition::math::Matrix4d ModelTransform() const;
    + ***Deprecation:***NodeTransform(math::Matrix4 _mat, std::string _sid = "_default_",TransformType _type = MATRIX);
    + ***Replacement:***NodeTransform(const ignition::math::Matrix4d &_mat,const std::string &_sid = "_default_",TransformType _type = MATRIX);
    + ***Deprecation:***void Set(math::Matrix4 _mat);
    + ***Replacement:***void Set(const ignition::math::Matrix4d &_mat);
    + ***Deprecation:***math::Matrix4 Get();
    + ***Replacement:***ignition::math::Matrix4d GetTransform() const;
    + ***Deprecation:***void SetSourceValues(math::Matrix4 _mat);
    + ***Replacement:***void SetSourceValues(const ignition::math::Matrix4d &_mat);
    + ***Deprecation:***void SetSourceValues(math::Vector3 _vec);
    + ***Replacement:*** void SetSourceValues(const ignition::math::Vector3d &_vec);
    + ***Deprecation:***void SetSourceValues(math::Vector3 _axis, double _angle);
    + ***Replacement:***void SetSourceValues(const ignition::math::Vector3d &_axis,const double _angle);
    + ***Deprecation:***math::Matrix4 operator* (math::Matrix4 _m);
    + ***Replacement:***ignition::math::Matrix4d operator*(const ignition::math::Matrix4d &_m);

1. **gazebo/common/SkeletonAnimation.hh**
    + ***Deprecation:***void AddKeyFrame(const double _time, const math::Matrix4 &_trans);
    + ***Replacement:***void AddKeyFrame(const double _time,const ignition::math::Matrix4d &_trans);
    + ***Deprecation:***void AddKeyFrame(const double _time,const math::Pose &_pose);
    + ***Replacement:***void AddKeyFrame(const double _time,const ignition::math::Pose3d &_pose);
    + ***Deprecation:***void GetKeyFrame(const unsigned int _i, double &_time,math::Matrix4 &_trans);
    + ***Replacement:***void GetKeyFrame(const unsigned int _i, double &_time,ignition::math::Matrix4d &_trans) const;
    + ***Deprecation:***std::pair<double, math::Matrix4> GetKeyFrame(const unsigned int _i);
    + ***Replacement:***std::pair<double, ignition::math::Matrix4d> KeyFrame(const unsigned int _i) const;
    + ***Deprecation:***math::Matrix4 GetFrameAt(double _time, bool _loop = true) const;
    + ***Replacement:***ignition::math::Matrix4d FrameAt(double _time, bool _loop = true) const;
    + ***Deprecation:***void AddKeyFrame(const std::string &_node, const double _time, const math::Matrix4 &_mat);
    + ***Replacement:***void AddKeyFrame(const std::string &_node, const double _time,const ignition::math::Matrix4d &_mat);
    + ***Deprecation:***void AddKeyFrame(const std::string &_node, const double _time,const math::Pose &_pose);
    + ***Replacement:***void AddKeyFrame(const std::string &_node, const double _time,const ignition::math::Pose3d &_pose);
    + ***Deprecation:*** math::Matrix4 GetNodePoseAt(const std::string &_node,const double _time, const bool _loop = true);
    + ***Replacement:***ignition::math::Matrix4d NodePoseAt(const std::string &_node,const double _time, const bool _loop = true);
    + ***Deprecation:***std::map<std::string, math::Matrix4> GetPoseAt(const double _time, const bool _loop = true) const;
    + ***Replacement:***std::map<std::string, ignition::math::Matrix4d> PoseAt(const double _time, const bool _loop = true) const;
    + ***Deprecation:***std::map<std::string, math::Matrix4> GetPoseAtX(const double _x, const std::string &_node, const bool _loop = true) const;
    + ***Replacement:***std::map<std::string, ignition::math::Matrix4d> PoseAtX(const double _x, const std::string &_node, const bool _loop = true) const;

1. **gazebo/common/SphericalCoordinates.hh**
    + ***Deprecation:***SphericalCoordinates(const SurfaceType _type,const math::Angle &_latitude,const math::Angle &_longitude,double _elevation,const math::Angle &_heading);
    + ***Replacement:***SphericalCoordinates(const SurfaceType _type,const ignition::math::Angle &_latitude,const ignition::math::Angle &_longitude,double _elevation,const ignition::math::Angle &_heading);
    + ***Deprecation:***math::Vector3 SphericalFromLocal(const math::Vector3 &_xyz) const;
    + ***Replacement:***ignition::math::Vector3d SphericalFromLocal(const ignition::math::Vector3d &_xyz) const;
    + ***Deprecation:***math::Vector3 GlobalFromLocal(const math::Vector3 &_xyz) const;
    + ***Replacement:***ignition::math::Vector3d GlobalFromLocal(const ignition::math::Vector3d &_xyz) const;
    + ***Deprecation:***static double Distance(const math::Angle &_latA,const math::Angle &_lonA,const math::Angle &_latB,const math::Angle &_lonB);
    + ***Replacement:***static double Distance(const ignition::math::Angle &_latA,const ignition::math::Angle &_lonA,const ignition::math::Angle &_latB,const ignition::math::Angle &_lonB);
    + ***Deprecation:*** math::Angle GetLatitudeReference() const;
    + ***Replacement:***ignition::math::Angle LatitudeReference() const;
    + ***Deprecation:***math::Angle GetLongitudeReference() const;
    + ***Replacement:***ignition::math::Angle LongitudeReference() const;
    + ***Deprecation:***math::Angle GetHeadingOffset() const;
    + ***Replacement:***ignition::math::Angle HeadingOffset() const;
    + ***Deprecation:***void SetLatitudeReference(const math::Angle &_angle);
    + ***Replacement:***void SetLatitudeReference(const ignition::math::Angle &_angle);
    + ***Deprecation:***void SetLongitudeReference(const math::Angle &_angle);
    + ***Replacement:***void SetLongitudeReference(const ignition::math::Angle &_angle);
    + ***Deprecation:***void SetHeadingOffset(const math::Angle &_angle);
    + ***Replacement:***void SetHeadingOffset(const ignition::math::Angle &_angle);

### Modifications

1. **gazebo/common/MouseEvent.hh**
    * Replaced all member variables with functions that use Ignition Math.
    * [BitBucket pull request #1777](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1777)

1. **gazebo/msgs/world_stats.proto**
    + ***Removed:*** optional bool log_playback = 8;
    + ***Replacement:*** optional LogPlaybackStatistics log_playback_stats = 8;

1. **gazebo/physics/JointState.hh**
    + ***Removed:*** public: JointState(JointPtr _joint, const common::Time
    &_realTime, const common::Time &_simTime)
    + ***Replacement:*** public: JointState(JointPtr _joint, const common::Time
    &_realTime, const common::Time &_simTime, const uint64_t _iterations)

1. **gazebo/physics/LinkState.hh**
    + ***Removed:*** public: LinkState(const LinkPtr _link, const common::Time
    &_realTime, const common::Time &_simTime)
    + ***Replacement:*** public: LinkState(const LinkPtr _link,
    const common::Time &_realTime, const common::Time &_simTime, const uint64_t
    _iterations)
    + ***Removed:*** public: void Load(const LinkPtr _link, const common::Time
    &_realTime, const common::Time &_simTime)
    + ***Replacement:*** public: void Load(const LinkPtr _link, const
    common::Time &_realTime, const common::Time &_simTime, const uint64_t
    _iterations)

1. **gazebo/physics/ModelState.hh**
    + ***Removed:*** public: ModelState(const ModelPtr _model, const
    common::Time &_realTime, const common::Time &_simTime)
    + ***Replacement:*** public: ModelState(const ModelPtr _model, const
    common::Time &_realTime, const common::Time &_simTime, const uint64_t
    _iterations)
    + ***Removed:*** public: void Load(const ModelPtr _model, const common::Time
    &_realTime, const common::Time &_simTime)
    + ***Replacement:*** public: void Load(const ModelPtr _model, const
    common::Time &_realTime, const common::Time &_simTime, const uint64_t
    _iterations)

1. **gazebo/physics/State.hh**
    + ***Removed:*** public: State(const std::string &_name, const
    common::Time &_realTime, const common::Time &_simTime)
    + ***Replacement:*** public: State(const std::string &_name,
    const common::Time &_realTime, const common::Time &_simTime, const uint64_t
    _iterations)

1. **gazebo/physics/ModelState.hh**
    + ***Removed:*** public: void Load(const ModelPtr _model, const common::Time
    &_realTime, const common::Time &_simTime)
    + ***Replacement:*** public: void Load(const ModelPtr _model, const
    common::Time &_realTime, const common::Time &_simTime, const uint64_t
    _iterations)

1. ignition-math is now a dependency. Many classes and functions are modified to use ignition-math, please see the pull request listing below for individual changes.
    + [http://ignitionrobotics.org/libraries/math](http://ignitionrobotics.org/libraries/math)
    + [Gazebo migration](https://github.com/osrf/gazebo/src/583edbeb90759d43d994cc57c0797119dd6d2794/ign-math-migration.md)
    * [BitBucket pull request #1756](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1756)
    * [BitBucket pull request #1766](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1766)
    * [BitBucket pull request #1774](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1774)
    * [BitBucket pull request #1771](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1771)
    * [BitBucket pull request #1776](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1776)
    * [BitBucket pull request #1777](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1777)
    * [BitBucket pull request #1772](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1772)
    * [BitBucket pull request #1773](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1773)
    * [BitBucket pull request #1778](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/1778)

1. Gazebo client's should now use `gazebo/gazebo_client.hh` and `libgazebo_client.so` instead of `gazebo/gazebo.hh` and `libgazebo.so`. This separates running a Gazebo server from a Gazebo client.
    + ***Removed:*** bool gazebo::setupClient(int _argc = 0, char **_argv = 0);
    + ***Replacement:*** bool gazebo::client::setup(int _argc = 0, char **_argv = 0);

1. **gazebo/rendering/GpuLaser.hh**
    + ***Removed:*** protected: double near
    + ***Replacement:*** protected: double nearClip

1. **gazebo/rendering/GpuLaser.hh**
    + ***Removed:*** protected: double far
    + ***Replacement:*** protected: double farClip

1. **gazebo/rendering/Visual.hh**
    + ***Removed:*** public: void Fini();
    + ***Replacement:*** public: virtual void Fini();

1. **gazebo/common/MeshManager.hh**
    + ***Removed:*** void CreateExtrudedPolyline(const std::string &_name, const std::vector<math::Vector2d> &_vertices, const double &_height, const math::Vector2d &_uvCoords)
    + ***Replacement:*** void CreateExtrudedPolyline(const std::string &_name, const const std::vector<std::vector<math::Vector2d> > &_vertices, const double &_height, const math::Vector2d &_uvCoords)

1. **gazebo/common/GTSMeshUtils.hh**
    + ***Removed:*** public: static bool CreateExtrudedPolyline(const std::vector<math::Vector2d> &_vertices, const double &_height, SubMesh *_submesh)
    + ***Replacement:*** public: static bool DelaunayTriangulation(const std::vector<std::vector<math::Vector2d> > &_path, SubMesh *_submesh)

1. **gazebo/physics/PolylineShape.hh**
    + ***Removed:*** public: std::vector<math::Vector2d> GetVertices() const
    + ***Replacement:*** public: std::vector<std::vector<math::Vector2d> > GetVertices() const

1. **gazebo/physics/SurfaceParams.hh**
    + ***Removed:*** public: FrictionPyramid frictionPyramid
    + ***Replacement:*** public: FrictionPyramidPtr GetFrictionPyramid() const

### Deletions

1. **gazebo/gui/RenderWidget.hh**
    + The ShowEditor(bool _show)

### Additions

1. **gazebo/msgs/log_playback_control.proto**
    + New message to control the playback from a log file.

1. **gazebo/util/LogPlay.hh**
    + public: bool Rewind()

1. **gazebo/physics/LinkState.hh**
    + public: virtual void SetIterations(const uint64_t _iterations)

1. **gazebo/physics/ModelState.hh**
    + public: virtual void SetIterations(const uint64_t _iterations)

1. **gazebo/physics/State.hh**
    + public: uint64_t GetIterations() const
    + public: virtual void SetIterations(const uint64_t _iterations)

1. **gazebo/physics/WorldState.hh**
    + public: virtual void SetIterations(const uint64_t _iterations)

1. **gazebo/util/LogPlay.hh**
    + public: uint64_t GetInitialIterations() const
    + public: bool HasIterations() const

## Gazebo 4.X to 5.X

### C++11 compiler required

Gazebo 5.x uses features from the new c++11 standard. This requires to have a compatible c++11 compiler. Note that some platforms (like Ubuntu Precise) do not include one by default.

### Modifications

1. Privatized World::dirtyPoses
    + World::dirtyPoses used to be a public attribute. This is now a private attribute, and specific "friends" have been added to the World file.

1. Privatized Scene::skyx
    + Scene::skyx used to be a public attribute. This is now a private attribute, and a GetSkyX() funcion has been added to access the sky object.

1. **gazebo/rendering/Visual.hh**
    + The GetBoundingBox() function now returns a local bounding box without scale applied.

1. **gazebo/math/Box.hh**
    + The constructor that takes two math::Vector3 values now treats these as two corners, and computes the minimum and maximum values automatically. This change is API and ABI compatible.

1. **Informational logs:** The log files will be created inside
  ~/.gazebo/server-<GAZEBO_MASTER_PORT> and
  ~/.gazebo/client-<GAZEBO_MASTER_PORT>. The motivation for this
  change is to avoid name collisions when cloning a simulation. If the
  environment variable GAZEBO_MASTER_URI is not present or invalid,
  <GAZEBO_MASTER_PORT> will be replaced by "default".

1. **gazebo/common/Plugin.hh**
    + ***Removed:*** protected: std::string Plugin::handle
    + ***Replacement:*** protected: std::string Plugin::handleName

1. **gazebo/gui/KeyEventHandler.hh**
    + ***Removed:*** public: void HandlePress(const common::KeyEvent &_event);
    + ***Replacement:*** public: bool HandlePress(const common::KeyEvent &_event);

1. **gazebo/gui/KeyEventHandler.hh**
    + ***Removed:*** public: void HandleRelease(const common::KeyEvent &_event);
    + ***Replacement:*** public: bool HandleRelease(const common::KeyEvent &_event);

1. **gazebo/rendering/UserCamera.hh**
    + ***Removed:*** private: void OnJoy(ConstJoystickPtr &_msg)
    + ***Replacement:*** private: void OnJoyTwist(ConstJoystickPtr &_msg)

1. **gazebo/rendering/Camera.hh**
    + ***Deprecation:*** public: void RotatePitch(math::Angle _angle);
    + ***Replacement:*** public: void Pitch(const math::Angle &_angle,
                                        Ogre::Node::TransformSpace _relativeTo = Ogre::Node::TS_LOCAL);
    + ***Deprecation:*** public: void RotateYaw(math::Angle _angle);
    + ***Replacement:*** public: void Yaw(const math::Angle &_angle,
                                        Ogre::Node::TransformSpace _relativeTo = Ogre::Node::TS_LOCAL);

1. **gazebo/rendering/AxisVisual.hh**
    + ***Removed:*** public: void ShowRotation(unsigned int _axis)
    + ***Replacement:*** public: void ShowAxisRotation(unsigned int _axis, bool _show)

1. **gazebo/rendering/ArrowVisual.hh**
    + ***Removed:*** public: void ShowRotation()
    + ***Replacement:*** public: void ShowRotation(bool _show)

### Deletions

1. **gazebo/physics/Collision.hh**
    + unsigned int GetShapeType()

1. **gazebo/physics/World.hh**
    + EntityPtr GetSelectedEntity() const

1. **gazebo/physics/bullet/BulletJoint.hh**
    + void SetAttribute(Attribute, unsigned int, double)

1. **gazebo/physics/simbody/SimbodyJoint.hh**
    + void SetAttribute(Attribute, unsigned int, double)


## Gazebo 3.1 to 4.0

### New Deprecations

1. **gazebo/physics/Collision.hh**
    + ***Deprecation*** unsigned int GetShapeType()
    + ***Replacement*** unsigned int GetShapeType() const

1. **gazebo/physics/Joint.hh**
    + ***Deprecation*** virtual double GetMaxForce(unsigned int)
    + ***Deprecation*** virtual void SetMaxForce(unsigned int, double)
    + ***Deprecation*** virtual void SetAngle(unsigned int, math::Angle)
    + ***Replacement*** virtual void SetPosition(unsigned int, double)

### Modifications
1. **gazebo/physics/Model.hh**
    + ***Removed:*** Link_V GetLinks() const `ABI Change`
    + ***Replacement:***  const Link_V &GetLinks() const

1. **gzprop command line tool**
    + The `gzprop` command line tool outputs a zip file instead of a tarball.

### Additions

1. **gazebo/msgs/msgs.hh**
    + sdf::ElementPtr LightToSDF(const msgs::Light &_msg, sdf::ElementPtr _sdf = sdf::ElementPtr())

1. **gazebo/rendering/Light.hh**
    + math::Quaternion GetRotation() const
    + void SetRotation(const math::Quaternion &_q)
    + LightPtr Clone(const std::string &_name, ScenePtr _scene)

1. **gazebo/rendering/Scene.hh**
    + void AddLight(LightPtr _light)
    + void RemoveLight(LightPtr _light)

1. **gazebo/gui/GuiEvents.hh**
    + template<typename T> static event::ConnectionPtr ConnectLightUpdate(T _subscriber)
    + static void DisconnectLightUpdate(event::ConnectionPtr _subscriber)

1. **gazebo/gui/ModelMaker.hh**
    + bool InitFromModel(const std::string & _modelName)

1. **gazebo/gui/LightMaker.hh**
    + bool InitFromLight(const std::string & _lightName)

1. **gazebo/common/Mesh.hh**
    + int GetMaterialIndex(const Material *_mat) const

1. **gazebo/math/Filter.hh**
    + ***New classes:*** Filter, OnePole, OnePoleQuaternion, OnePoleVector3, BiQuad, and BiQuadVector3

1. **gazebo/physics/Joint.hh**
      + bool FindAllConnectedLinks(const LinkPtr &_originalParentLink,
          Link_V &_connectedLinks);
      + math::Pose ComputeChildLinkPose( unsigned int _index,
          double _position);

1. **gazebo/physics/Link.hh**
      + void Move(const math::Pose &_worldRefernceFrameSrc,
                        const math::Pose &_worldRefernceFrameDst);
      + bool FindAllConnectedLinksHelper(
          const LinkPtr &_originalParentLink,
          Link_V &_connectedLinks, bool _fistLink = false);
      + bool ContainsLink(const Link_V &_vector, const LinkPtr &_value);
      + msgs::Visual GetVisualMessage(const std::string &_name)

### Modifications
1. **gazebo/physics/Model.hh**
    + ***Removed:*** Link_V GetLinks() const `ABI Change`
    + ***Replacement:***  const Link_V &GetLinks() const

1. **gazebo/physics/Base.cc**
    + ***Removed*** std::string GetScopedName() const
    + ***Replaced*** std::string GetScopedName(bool _prependWorldName=false) const

## Gazebo 3.0 to 3.1

### Additions

1. **gazebo/physics/World.hh**
      + void RemoveModel(const std::string &_name);
      + void RemoveModel(ModelPtr _model);

1. **gazebo/physics/JointController.hh**
    + void SetPositionPID(const std::string &_jointName, const common::PID &_pid);
    + void SetVelocityPID(const std::string &_jointName, const common::PID &_pid);

## Gazebo 2.0 to 3.0

### New Deprecations

1. **gazebo/physics/Joint.hh**
    + ***Deprecation*** virtual void ApplyDamping()
    + ***Replacement*** virtual void ApplyStiffnessDamping()
    ---
    + ***Deprecation*** double GetDampingCoefficient() const
    + ***Replacement*** double GetDamping(int _index)

1. **gazebo/physics/ode/ODEJoint.hh**
    + ***Deprecation*** void CFMDamping()
    + ***Replacement*** void ApplyImplicitStiffnessDamping()

1. **gazebo/physics/ScrewJoint.hh**
    + ***Deprecation*** virtual void SetThreadPitch(unsigned int _index, double _threadPitch) = 0
    + ***Replacement*** virtual void SetThreadPitch(double _threadPitch) = 0
    ---
    + ***Deprecation*** virtual void GetThreadPitch(unsigned int _index) = 0
    + ***Replacement*** virtual void GetThreadPitch() = 0

1. **gazebo/physics/bullet/BulletScrewJoint.hh**
    + ***Deprecation*** protected: virtual void Load(sdf::ElementPtr _sdf)
    + ***Replacement*** public: virtual void Load(sdf::ElementPtr _sdf)

1. **gazebo/physics/PhysicsEngine.hh**
    + ***Deprecation*** virtual void SetSORPGSPreconIters(unsigned int _iters)
    + ***Replacement*** virtual bool SetParam(const std::string &_key, const boost::any &_value)
    ---
    + ***Deprecation*** virtual void SetSORPGSIters(unsigned int _iters)
    + ***Replacement*** virtual bool SetParam(const std::string &_key, const boost::any &_value)
    ---
    + ***Deprecation*** virtual void SetSORPGSW(double _w)
    + ***Replacement*** virtual bool SetParam(const std::string &_key, const boost::any &_value)
    ---
    + ***Deprecation*** virtual int GetSORPGSPreconIters()
    + ***Replacement*** virtual boost::any GetParam(const std::string &_key) const
    ---
    + ***Deprecation*** virtual int GetSORPGSIters()
    + ***Replacement*** virtual boost::any GetParam(const std::string &_key) const
    ---
    + ***Deprecation*** virtual double GetSORPGSW()
    + ***Replacement*** virtual boost::any GetParam(const std::string &_key) const

1. **gazebo/physics/bullet/BulletPhysics.hh**
    + ***Deprecation*** virtual bool SetParam(BulletParam _param, const boost::any &_value)
    + ***Replacement*** virtual bool SetParam(const std::string &_key, const boost::any &_value)
    ---
    + ***Deprecation*** virtual boost::any GetParam(BulletParam _param) const
    + ***Replacement*** virtual boost::any GetParam(const std::string &_key) const

1. **gazebo/physics/ode/ODEPhysics.hh**
    + ***Deprecation*** virtual bool SetParam(ODEParam _param, const boost::any &_value)
    + ***Replacement*** virtual bool SetParam(const std::string &_key, const boost::any &_value)
    ---
    + ***Deprecation*** virtual boost::any GetParam(ODEParam _param) const
    + ***Replacement*** virtual boost::any GetParam(const std::string &_key) const

1. **gazebo/physics/dart/DARTPhysics.hh**
    + ***Deprecation*** virtual boost::any GetParam(DARTParam _param) const
    + ***Replacement*** virtual boost::any GetParam(const std::string &_key) const

1. **gazebo/physics/Joint.hh**
    + ***Deprecation*** virtual double GetAttribute(const std::string &_key, unsigned int _index) = 0
    + ***Replacement*** virtual double GetParam(const std::string &_key, unsigned int _index) = 0;

1. **gazebo/physics/bullet/BulletJoint.hh**
    + ***Deprecation*** virtual double GetAttribute(const std::string &_key, unsigned int _index)
    + ***Replacement*** virtual double GetParam(const std::string &_key, unsigned int _index)

1. **gazebo/physics/bullet/BulletScrewJoint.hh**
    + ***Deprecation*** virtual double GetAttribute(const std::string &_key, unsigned int _index)
    + ***Replacement*** virtual double GetParam(const std::string &_key, unsigned int _index)

1. **gazebo/physics/dart/DARTJoint.hh**
    + ***Deprecation*** virtual double GetParam(const std::string &_key, unsigned int _index)
    + ***Replacement*** virtual double GetAttribute(const std::string &_key, unsigned int _index)

1. **gazebo/physics/ode/ODEJoint.hh**
    + ***Deprecation*** virtual double GetParam(const std::string &_key, unsigned int _index)
    + ***Replacement*** virtual double GetAttribute(const std::string &_key, unsigned int _index)

1. **gazebo/physics/ode/ODEScrewJoint.hh**
    + ***Deprecation*** virtual double GetParam(const std::string &_key, unsigned int _index)
    + ***Replacement*** virtual double GetAttribute(const std::string &_key, unsigned int _index)

1. **gazebo/physics/ode/ODEUniversalJoint.hh**
    + ***Deprecation*** virtual double GetParam(const std::string &_key, unsigned int _index)
    + ***Replacement*** virtual double GetAttribute(const std::string &_key, unsigned int _index)

1. **gazebo/physics/simbody/SimbodyJoint.hh**
    + ***Deprecation*** virtual double GetParam(const std::string &_key, unsigned int _index)
    + ***Replacement*** virtual double GetAttribute(const std::string &_key, unsigned int _index)

1. **gazebo/physics/simbody/SimbodyScrewJoint.hh**
    + ***Deprecation*** virtual double GetParam(const std::string &_key, unsigned int _index)
    + ***Replacement*** virtual double GetAttribute(const std::string &_key, unsigned int _index)

1. **gazebo/physics/Joint.hh**
    + ***Deprecation*** virtual void SetAttribute(const std::string &_key, unsigned int _index, const boost::any &_value) = 0
    + ***Replacement*** virtual bool SetParam(const std::string &_key, unsigned int _index, const boost::any &_value) = 0

1. **gazebo/physics/bullet/BulletJoint.hh**
    + ***Deprecation*** virtual void SetAttribute(const std::string &_key, unsigned int _index, const boost::any &_value)
    + ***Replacement*** virtual bool SetParam(const std::string &_key, unsigned int _index, const boost::any &_value)

1. **gazebo/physics/dart/DARTJoint.hh**
    + ***Deprecation*** virtual void SetAttribute(const std::string &_key, unsigned int _index, const boost::any &_value)
    + ***Replacement*** virtual bool SetParam(const std::string &_key, unsigned int _index, const boost::any &_value)

1. **gazebo/physics/ode/ODEJoint.hh**
    + ***Deprecation*** virtual void SetAttribute(const std::string &_key, unsigned int _index, const boost::any &_value)
    + ***Replacement*** virtual bool SetParam(const std::string &_key, unsigned int _index, const boost::any &_value)

1. **gazebo/physics/ode/ODEScrewJoint.hh**
    + ***Deprecation*** virtual void SetAttribute(const std::string &_key, unsigned int _index, const boost::any &_value)
    + ***Replacement*** virtual bool SetParam(const std::string &_key, unsigned int _index, const boost::any &_value)

1. **gazebo/physics/ode/ODEUniversalJoint.hh**
    + ***Deprecation*** virtual void SetAttribute(const std::string &_key, unsigned int _index, const boost::any &_value)
    + ***Replacement*** virtual bool SetParam(const std::string &_key, unsigned int _index, const boost::any &_value)

1. **gazebo/physics/simbody/SimbodyJoint.hh**
    + ***Deprecation*** virtual void SetAttribute(const std::string &_key, unsigned int _index, const boost::any &_value)
    + ***Replacement*** virtual bool SetParam(const std::string &_key, unsigned int _index, const boost::any &_value)

1. **gazebo/physics/simbody/SimbodyScrewJoint.hh**
    + ***Deprecation*** virtual void SetAttribute(const std::string &_key, unsigned int _index, const boost::any &_value)
    + ***Replacement*** virtual bool SetParam(const std::string &_key, unsigned int _index, const boost::any &_value)

### Modifications
1. **gazebo/physics/Entity.hh**
    + ***Removed:*** inline const math::Pose &GetWorldPose() const `ABI change`
    + ***Replacement:*** inline virutal const math::Pose &GetWorldPose() const
1. **gazebo/physics/Box.hh**
    + ***Removed:*** bool operator==(const Box &_b) `ABI Change`
    + ***Replacement:***  bool operator==(const Box &_b) const

1. **gazebo/gui/GuiIface.hh**
    + ***Removed:*** void load() `ABI change`
    + ***Replacement:*** bool load()
    + ***Note:*** Changed return type from void to bool.
1. **Functions in joint classes use unsigned int, instead of int**
    + All functions in Joint classes (gazebo/physics/\*Joint\*) and subclasses (gazebo/physics/[ode,bullet,simbody,dart]/\*Joint\*) now use unsigned integers instead of integers when referring to a specific joint axis.
    + Add const to Joint::GetInitialAnchorPose(), Joint::GetStopDissipation(), Joint::GetStopStiffness()
1. **gazebo/sensors/Noise.hh** `ABI change`
    + ***Removed:*** void Noise::Load(sdf::ElementPtr _sdf)
    + ***Replacement:*** virtual void Noise::Load(sdf::ElementPtr _sdf)
    + ***Removed:*** void Noise::~Noise()
    + ***Replacement:*** virtual void Noise::~Noise()
    + ***Removed:*** void Noise::Apply() const
    + ***Replacement:*** void Noise::Apply()
    + ***Note:*** Make Noise a base class and refactored out GaussianNoiseModel to its own class.
1. **gazebo/transport/ConnectionManager.hh**
    + ***Removed:*** bool ConnectionManager::Init(const std::string &_masterHost, unsigned int _masterPort) `ABI change`
    + ***Replacement:*** bool ConnectionManager::Init(const std::string &_masterHost, unsigned int _masterPort, uint32_t _timeoutIterations = 30)
    + ***Note:*** No changes to downstream code required. A third parameter has been added that specifies the number of timeout iterations. This parameter has a default value of 30.
1. **gazebo/transport/TransportIface.hh**
    + ***Removed:*** bool init(const std::string &_masterHost = "", unsigned int _masterPort = 0) `ABI change`
    + ***Replacement:*** bool init(const std::string &_masterHost = "", unsigned int _masterPort = 0, uint32_t _timeoutIterations = 30)
    + ***Note:*** No changes to downstream code required. A third parameter has been added that specifies the number of timeout iterations. This parameter has a default value of 30.
1. **gazebo/transport/Publication.hh**
    + ***Removed:*** void Publish(MessagePtr _msg, boost::function<void(uint32_t)> _cb, uint32_t _id) `ABI change`
    + ***Replacement:*** int Publish(MessagePtr _msg, boost::function<void(uint32_t)> _cb, uint32_t _id)
    + ***Note:*** Only the return type changed.

1. **gazebo/common/ModelDatabase.hh** `API change`
    + ***Removed:*** void ModelDatabase::GetModels(boost::function<void (const std::map<std::string, std::string> &)> _func)
    + ***Replacement:*** event::ConnectionPtr ModelDatabase::GetModels(boost::function<void (const std::map<std::string, std::string> &)> _func)
    + ***Note:*** The replacement function requires that the returned connection shared pointer remain valid in order to receive the GetModels callback. Reset the shared pointer to stop receiving GetModels callback.

1. **gazebo/physics/Collision.hh** `API change`
    + ***Modified:*** SurfaceParamsPtr Collision::surface
    + ***Note:*** Changed from `private` to `protected`

1. **gazebo/physics/MultiRayShape.hh** `API change`
    + ***Removed:*** double MultiRayShape::GetRange(int _index)
    + ***Replacement:*** double MultiRayShape::GetRange(unsigned int _index)
    + ***Removed:*** double MultiRayShape::GetRetro(int _index)
    + ***Replacement:*** double MultiRayShape::GetRetro(unsigned int _index)
    + ***Removed:*** double MultiRayShape::GetFiducial(int _index)
    + ***Replacement:*** double MultiRayShape::GetFiducial(unsigned int _index)
    + ***Note:*** Changed argument type from int to unsigned int.

1. **gazebo/physics/SurfaceParams.hh**
    + ***Removed:*** void FillMsg(msgs::Surface &_msg)
    + ***Replacement:*** virtual void FillMsg(msgs::Surface &_msg)

1. **gazebo/sensors/RaySensor.hh** `API change`
    + ***Removed:*** double RaySensor::GetRange(int _index)
    + ***Replacement:*** double RaySensor::GetRange(unsigned int _index)
    + ***Removed:*** double RaySensor::GetRetro(int _index)
    + ***Replacement:*** double RaySensor::GetRetro(unsigned int _index)
    + ***Removed:*** double RaySensor::GetFiducial(int _index)
    + ***Replacement:*** double RaySensor::GetFiducial(unsigned int _index)
    + ***Note:*** Changed argument type from int to unsigned int.

1. **gazebo/physics/PhysicsEngine.hh**
    + ***Removed*** virtual void SetParam(const std::string &_key, const boost::any &_value)
    + ***Replacement*** virtual bool SetParam(const std::string &_key, const boost::any &_value)

1. **gazebo/physics/ode/ODEPhysics.hh**
    + ***Removed*** virtual void SetParam(const std::string &_key, const boost::any &_value)
    + ***Replacement*** virtual bool SetParam(const std::string &_key, const boost::any &_value)

1. **gazebo/physics/bullet/BulletPhysics.hh**
    + ***Removed*** virtual void SetParam(const std::string &_key, const boost::any &_value)
    + ***Replacement*** virtual bool SetParam(const std::string &_key, const boost::any &_value)

1. **gazebo/physics/BallJoint.hh**
    + ***Removed*** virtual void SetHighStop(unsigned int /*_index*/, const math::Angle &/*_angle*/)
    + ***Replacement*** virtual bool SetHighStop(unsigned int /*_index*/, const math::Angle &/*_angle*/)
    ---
    + ***Removed*** virtual void SetLowStop(unsigned int /*_index*/, const math::Angle &/*_angle*/)
    + ***Replacement*** virtual bool SetLowStop(unsigned int /*_index*/, const math::Angle &/*_angle*/)

1. **gazebo/physics/Joint.hh**
    + ***Removed*** virtual void SetHighStop(unsigned int _index, const math::Angle &_angle)
    + ***Replacement*** virtual bool SetHighStop(unsigned int _index, const math::Angle &_angle)
    ---
    + ***Removed*** virtual void SetLowStop(unsigned int _index, const math::Angle &_angle)
    + ***Replacement*** virtual bool SetLowStop(unsigned int _index, const math::Angle &_angle)

1. **gazebo/physics/bullet/BulletBallJoint.hh**
    + ***Removed*** virtual void SetHighStop(unsigned int _index, const math::Angle &_angle)
    + ***Replacement*** virtual bool SetHighStop(unsigned int _index, const math::Angle &_angle)
    ---
    + ***Removed*** virtual void SetLowStop(unsigned int _index, const math::Angle &_angle)
    + ***Replacement*** virtual bool SetLowStop(unsigned int _index, const math::Angle &_angle)

1. **gazebo/physics/bullet/BulletHinge2Joint.hh**
    + ***Removed*** virtual void SetHighStop(unsigned int _index, const math::Angle &_angle)
    + ***Replacement*** virtual bool SetHighStop(unsigned int _index, const math::Angle &_angle)
    ---
    + ***Removed*** virtual void SetLowStop(unsigned int _index, const math::Angle &_angle)
    + ***Replacement*** virtual bool SetLowStop(unsigned int _index, const math::Angle &_angle)

1. **gazebo/physics/bullet/BulletHingeJoint.hh**
    + ***Removed*** virtual void SetHighStop(unsigned int _index, const math::Angle &_angle)
    + ***Replacement*** virtual bool SetHighStop(unsigned int _index, const math::Angle &_angle)
    ---
    + ***Removed*** virtual void SetLowStop(unsigned int _index, const math::Angle &_angle)
    + ***Replacement*** virtual bool SetLowStop(unsigned int _index, const math::Angle &_angle)

1. **gazebo/physics/bullet/BulletScrewJoint.hh**
    + ***Removed*** virtual void SetHighStop(unsigned int _index, const math::Angle &_angle)
    + ***Replacement*** virtual bool SetHighStop(unsigned int _index, const math::Angle &_angle)
    ---
    + ***Removed*** virtual void SetLowStop(unsigned int _index, const math::Angle &_angle)
    + ***Replacement*** virtual bool SetLowStop(unsigned int _index, const math::Angle &_angle)

1. **gazebo/physics/bullet/BulletSliderJoint.hh**
    + ***Removed*** virtual void SetHighStop(unsigned int _index, const math::Angle &_angle)
    + ***Replacement*** virtual bool SetHighStop(unsigned int _index, const math::Angle &_angle)
    ---
    + ***Removed*** virtual void SetLowStop(unsigned int _index, const math::Angle &_angle)
    + ***Replacement*** virtual bool SetLowStop(unsigned int _index, const math::Angle &_angle)

1. **gazebo/physics/bullet/BulletUniversalJoint.hh**
    + ***Removed*** virtual void SetHighStop(unsigned int _index, const math::Angle &_angle)
    + ***Replacement*** virtual bool SetHighStop(unsigned int _index, const math::Angle &_angle)
    ---
    + ***Removed*** virtual void SetLowStop(unsigned int _index, const math::Angle &_angle)
    + ***Replacement*** virtual bool SetLowStop(unsigned int _index, const math::Angle &_angle)

1. **gazebo/physics/dart/DARTJoint.hh**
    + ***Removed*** virtual void SetHighStop(unsigned int _index, const math::Angle &_angle)
    + ***Replacement*** virtual bool SetHighStop(unsigned int _index, const math::Angle &_angle)
    ---
    + ***Removed*** virtual void SetLowStop(unsigned int _index, const math::Angle &_angle)
    + ***Replacement*** virtual bool SetLowStop(unsigned int _index, const math::Angle &_angle)

1. **gazebo/physics/ode/ODEJoint.hh**
    + ***Removed*** virtual void SetHighStop(unsigned int _index, const math::Angle &_angle)
    + ***Replacement*** virtual bool SetHighStop(unsigned int _index, const math::Angle &_angle)
    ---
    + ***Removed*** virtual void SetLowStop(unsigned int _index, const math::Angle &_angle)
    + ***Replacement*** virtual bool SetLowStop(unsigned int _index, const math::Angle &_angle)

1. **gazebo/physics/ode/ODEUniversalJoint.hh**
    + ***Removed*** virtual void SetHighStop(unsigned int _index, const math::Angle &_angle)
    + ***Replacement*** virtual bool SetHighStop(unsigned int _index, const math::Angle &_angle)
    ---
    + ***Removed*** virtual void SetLowStop(unsigned int _index, const math::Angle &_angle)
    + ***Replacement*** virtual bool SetLowStop(unsigned int _index, const math::Angle &_angle)

1. **gazebo/physics/simbody/SimbodyJoint.hh**
    + ***Removed*** virtual void SetHighStop(unsigned int _index, const math::Angle &_angle)
    + ***Replacement*** virtual bool SetHighStop(unsigned int _index, const math::Angle &_angle)
    ---
    + ***Removed*** virtual void SetLowStop(unsigned int _index, const math::Angle &_angle)
    + ***Replacement*** virtual bool SetLowStop(unsigned int _index, const math::Angle &_angle)

1. **gazebo/physics/simbody/SimbodyScrewJoint.hh**
    + ***Removed*** virtual void SetHighStop(unsigned int _index, const math::Angle &_angle)
    + ***Replacement*** virtual bool SetHighStop(unsigned int _index, const math::Angle &_angle)
    ---
    + ***Removed*** virtual void SetLowStop(unsigned int _index, const math::Angle &_angle)
    + ***Replacement*** virtual bool SetLowStop(unsigned int _index, const math::Angle &_angle)

### Additions

1. **gazebo/physics/Joint.hh**
      + bool FindAllConnectedLinks(const LinkPtr &_originalParentLink,
          Link_V &_connectedLinks);
      + math::Pose ComputeChildLinkPose( unsigned int _index,
          double _position);

1. **gazebo/physics/Link.hh**
      + void MoveFrame(const math::Pose &_worldReferenceFrameSrc,
                       const math::Pose &_worldReferenceFrameDst);
      + bool FindAllConnectedLinksHelper(
          const LinkPtr &_originalParentLink,
          Link_V &_connectedLinks, bool _fistLink = false);
      + bool ContainsLink(const Link_V &_vector, const LinkPtr &_value);

1. **gazebo/physics/Collision.hh**
    + void SetWorldPoseDirty()
    + virtual const math::Pose &GetWorldPose() const
1. **gazebo/physics/JointController.hh**
      + common::Time GetLastUpdateTime() const
      + std::map<std::string, JointPtr> GetJoints() const
      + bool SetPositionTarget(const std::string &_jointName, double _target)
      + bool SetVelocityTarget(const std::string &_jointName, double _target)
      + std::map<std::string, common::PID> GetPositionPIDs() const
      + std::map<std::string, common::PID> GetVelocityPIDs() const
      + std::map<std::string, double> GetForces() const
      + std::map<std::string, double> GetPositions() const
      + std::map<std::string, double> GetVelocities() const


1. **gazebo/common/PID.hh**
      + double GetPGain() const
      + double GetIGain() const
      + double GetDGain() const
      + double GetIMax() const
      + double GetIMin() const
      + double GetCmdMax() const
      + double GetCmdMin() const


1. **gazebo/transport/TransportIface.hh**
    +  transport::ConnectionPtr connectToMaster()

1. **gazebo/physics/World.hh**
    +  msgs::Scene GetSceneMsg() const
1. **gazebo/physics/ContactManager.hh**
    + unsigned int GetFilterCount()
    + bool HasFilter(const std::string &_name)
    + void RemoveFilter(const std::string &_name)

1. **gazebo/physics/Joint.hh**
    + virtual void Fini()
    + math::Pose GetAnchorErrorPose() const
    + math::Quaternion GetAxisFrame(unsigned int _index) const
    + double GetWorldEnergyPotentialSpring(unsigned int _index) const
    + math::Pose GetParentWorldPose() const
    + double GetSpringReferencePosition(unsigned int) const
    + math::Pose GetWorldPose() const
    + virtual void SetEffortLimit(unsigned _index, double _stiffness)
    + virtual void SetStiffness(unsigned int _index, double _stiffness) = 0
    + virtual void SetStiffnessDamping(unsigned int _index, double _stiffness, double _damping, double _reference = 0) = 0
    + bool axisParentModelFrame[MAX_JOINT_AXIS]
    + protected: math::Pose parentAnchorPose
    + public: double GetInertiaRatio(const math::Vector3 &_axis) const

1. **gazebo/physics/Link.hh**
    + double GetWorldEnergy() const
    + double GetWorldEnergyKinetic() const
    + double GetWorldEnergyPotential() const
    + bool initialized

1. **gazebo/physics/Model.hh**
    + double GetWorldEnergy() const
    + double GetWorldEnergyKinetic() const
    + double GetWorldEnergyPotential() const

1. **gazebo/physics/SurfaceParams.hh**
    + FrictionPyramid()
    + ~FrictionPyramid()
    + double GetMuPrimary()
    + double GetMuSecondary()
    + void SetMuPrimary(double _mu)
    + void SetMuSecondary(double _mu)
    + math::Vector3 direction1
    + ***Note:*** Replaces mu, m2, fdir1 variables

1. **gazebo/physics/bullet/BulletSurfaceParams.hh**
    + BulletSurfaceParams()
    + virtual ~BulletSurfaceParams()
    + virtual void Load(sdf::ElementPtr _sdf)
    + virtual void FillMsg(msgs::Surface &_msg)
    + virtual void ProcessMsg(msgs::Surface &_msg)
    + FrictionPyramid frictionPyramid

1. **gazebo/physics/ode/ODESurfaceParams.hh**
    + virtual void FillMsg(msgs::Surface &_msg)
    + virtual void ProcessMsg(msgs::Surface &_msg)
    + double bounce
    + double bounce
    + double bounceThreshold
    + double kp
    + double kd
    + double cfm
    + double erp
    + double maxVel
    + double minDepth
    + FrictionPyramid frictionPyramid
    + double slip1
    + double slip2

1. **gazebo/rendering/Light.hh**
    + bool GetVisible() const
    + virtual void LoadFromMsg(const msgs::Light &_msg)

1. **gazebo/sensors/ForceTorqueSensor.hh**
    + physics::JointPtr GetJoint() const

1. **gazebo/sensors/Noise.hh**
    + virtual double ApplyImpl(double _in)
    + virtual void Fini()
    + virtual void SetCustomNoiseCallback(boost::function<double (double)> _cb)

1. **gazebo/sensors/Sensor.hh**
    + NoisePtr GetNoise(unsigned int _index = 0) const

1. **gazebo/sensors/GaussianNoiseModel.hh**

1. **gazebo/physics/ode/ODEUniversalJoint.hh**
    + virtual void SetHighStop(unsigned int _index, const math::Angle &_angle)
    + virtual void SetLowStop(unsigned int _index, const math::Angle &_angle)
    + virtual void SetAttribute(const std::string &_key, unsigned int _index, const boost::any &_value)
    + virtual double GetAttribute(const std::string &_key, unsigned int _index)

1. **gazebo/physics/simbody/SimbodyScrewJoint.hh**
    + virtual void SetThreadPitch(double _threadPitch)
    + virtual void GetThreadPitch()

1. **gazebo/physics/ode/ODEScrewJoint.hh**
    + virtual void SetThreadPitch(double _threadPitch)
    + virtual void GetThreadPitch()

1. **gazebo/physics/ScrewJoint.hh**
    + virtual math::Vector3 GetAnchor(unsigned int _index) const
    + virtual void SetAnchor(unsigned int _index, const math::Vector3 &_anchor)

1. **gazebo/physics/bullet/BulletJoint.hh**
    + virtual math::Angle GetHighStop(unsigned int _index)
    + virtual math::Angle GetLowStop(unsigned int _index)

1. **gazebo/physics/simbody/SimbodyPhysics.hh**
    + virtual boost::any GetParam(const std::string &_key) const
    + virtual bool SetParam(const std::string &_key, const boost::any &_value)

1. **gazebo/physics/dart/DARTPhysics.hh**
    + virtual boost::any GetParam(const std::string &_key) const
    + virtual bool SetParam(const std::string &_key, const boost::any &_value)

1. **gazebo/physics/Joint.hh**
    + math::Quaternion GetAxisFrameOffset(unsigned int _index) const

### Deletions

1. **Removed libtool**
    + Libtool used to be an option for loading plugins. Now, only libdl is supported.

1. **gazebo/physics/Base.hh**
    + Base_V::iterator childrenEnd

1. **gazebo/sensors/Noise.hh**
    + double Noise::GetMean() const
    + double Noise::GetStdDev() const
    + double Noise::GetBias() const
    + ***Note:*** Moved gaussian noise functions to a new GaussianNoiseModel class

1. **gazebo/physics/SurfaceParams.hh**
    + double bounce
    + double bounce
    + double bounceThreshold
    + double kp
    + double kd
    + double cfm
    + double erp
    + double maxVel
    + double minDepth
    + double mu1
    + double mu2
    + double slip1
    + double slip2
    + math::Vector3 fdir1
    + ***Note:*** These parameters were moved to FrictionPyramid,
      ODESurfaceParams, and BulletSurfaceParams.


## Gazebo 1.9 to 2.0

### New Deprecations

1. **gazebo/gazebo.hh**
    + ***Deprecation*** void fini()
    + ***Deprecation*** void stop()
    + ***Replacement*** bool shutdown()
    + ***Note*** Replace fini and stop with shutdown
    ---
    + ***Deprecation*** bool load()
    + ***Deprecation*** bool init()
    + ***Deprecation*** bool run()
    + ***Replacement*** bool setupClient()
        + Use this function to setup gazebo for use as a client
    + ***Replacement*** bool setupServer()
        + Use this function to setup gazebo for use as a server
    + ***Note*** Replace load+init+run with setupClient/setupServer
    ---
    + ***Deprecation*** std::string find_file(const std::string &_file)
    + ***Replacement*** std::string common::find_file(const std::string &_file)
    ---
    + ***Deprecation*** void add_plugin(const std::string &_filename)
    + ***Replacement*** void addPlugin(const std::string &_filename)
    ---
    + ***Deprecation*** void print_version()
    + ***Replacement*** void printVersion()
1. **gazebo/physics/World.hh**
    + ***Deprecation*** void World::StepWorld(int _steps)
    + ***Replacement*** void World::Step(unsigned int _steps)
1. **gazebo/sensors/SensorsIface.hh**
    + ***Deprecation*** std::string sensors::create_sensor(sdf::ElementPtr _elem, const std::string &_worldName,const std::string &_parentName)
    + ***Replacement*** std::string sensors::create_sensor(sdf::ElementPtr _elem, const std::string &_worldName, const std::string &_parentName, uint32_t _parentId)
1. **gazebo/sensors/Sensor.hh**
    + ***Deprecation*** void Sensor::SetParent(const std::string &_name)
    + ***Replacement*** void Sensor::SetParent(const std::string &_name, uint32_t _id)
1. **gazebo/sensors/SensorManager.hh**
    + ***Deprecation*** std::string CreateSensor(sdf::ElementPtr _elem, const std::string &_worldName,  const std::string &_parentName)
    + ***Replacement*** std::string CreateSensor(sdf::ElementPtr _elem, const std::string &_worldName, const std::string &_parentName, uint32_t _parentId)
1. **gazebo/sensors/Collision.hh**
    + ***Deprecation*** void Collision::SetContactsEnabled(bool _enable)
    + ***Replacement*** Use [ContactManager](http://gazebosim.org/api/2.0.0/classgazebo_1_1physics_1_1ContactManager.html).
    ---
    + ***Deprecation*** bool Colliion::GetContactsEnabled() const
    + ***Replacement*** Use [ContactManager](http://gazebosim.org/api/2.0.0/classgazebo_1_1physics_1_1ContactManager.html).
    ---
    + ***Deprecation*** void AddContact(const Contact &_contact)
    + ***Replacement*** Use [ContactManager](http://gazebosim.org/api/2.0.0/classgazebo_1_1physics_1_1ContactManager.html).

### Modifications

1. File rename: `gazebo/common/Common.hh` to `gazebo/common/CommonIface.hh`
1. File rename: `gazebo/physics/Physics.hh` to `gazebo/physics/PhysicsIface.hh`
1. File rename: `gazebo/rendering/Rendering.hh` to `gazebo/rendering/RenderingIface.hh`
1. File rename: `gazebo/sensors/Sensors.hh` to `gazebo/sensors/SensorsIface.hh`
1. File rename: `gazebo/transport/Transport.hh` to `gazebo/transport/TransportIface.hh`
1. File rename: `gazebo/gui/Gui.hh` to `gazebo/gui/GuiIface.hh`
1. File rename: `<model>/manifest.xml` to `<model>/model.config`
1. File rename: `<model_database>/manifest.xml` to `<model_database>/database.config`
1. **gazebo/msgs/physics.proto**
    + ***Removed*** optional double dt
    + ***Replacement*** optional double min_step_size
    ---
    + ***Removed*** optional double update_rate
    + ***Replacement*** optional double real_time_update_rate
1. **gazebo/physics/ModelState.hh**
    + ***Removed*** LinkState ModelState::GetLinkState(int _index) `API change`
    + ***Replacement*** LinkState ModelState::GetLinkState(const std::string &_linkName) const
1. **gazebo/physics/PhyscisEngine.hh**
    + ***Removed*** void PhysicsEngine::SetUpdateRate(double _value) `API change`
    + ***Replacement*** void PhyscisEngine::SetRealTimeUpdateRate(double _rate)
    ---
    + ***Removed*** double PhysicsEngine::GetUpdateRate() `API change`
    + ***Replacement*** double PhysicsEngine::GetRealTimeUpdateRate() const
    ---
    + ***Removed*** void PhysicsEngine::SetStepTime(double _value) `API change`
    + ***Replacement*** void PhysicsEngine::SetMaxStepSize(double _stepSize)
    ---
    + ***Removed*** double PhysicsEngine::GetStepTime() `API change`
    + ***Replacement*** double PhysicsEngine::GetMaxStepSize() const
1. **gazebo/physics/Joint.hh**
    + ***Removed:*** Joint::Load(LinkPtr _parent, LinkPtr _child, const math::Vector3 &_pos) `API chance`
    + ***Replacement:*** Joint::Load(LinkPtr _parent, LinkPtr _child, const math::Pose &_pose)
    ---
    + ***Removed:*** public: double GetInertiaRatio(unsigned int _index) const
    + ***Replacement:*** public: double GetInertiaRatio(const unsigned int _index) const
1. **gazebo/common/Events.hh**
    + ***Removed:*** Events::ConnectWorldUpdateStart(T _subscriber) `API change`
    + ***Replacement*** ConnectionPtr Events::ConnectWorldUpdateBegin(T _subscriber)
    ---
    + ***Removed:*** Events::DisconnectWorldUpdateStart(T _subscriber) `API change`
    + ***Replacement*** ConnectionPtr Events::DiconnectWorldUpdateBegin(T _subscriber)
1. **gazebo/physics/Link.hh**
    + ***Removed*** void Link::RemoveChildJoint(JointPtr _joint) `API change`
    + ***Replacement*** void Link::RemoveChildJoint(const std::string &_jointName)
    ---
    + ***Removed*** void Link::RemoveParentJoint(const std::string &_jointName) `API change`
    + ***Replacement*** void Link::RemoveParentJoint(const std::string &_jointName)
1. **gazebo/physics/MeshShape.hh**
    + ***Removed*** std::string MeshShape::GetFilename() const `API change`
    + ***Replacement*** std::string MeshShape::GetURI() const
    ---
    + ***Removed*** void MeshShape::SetFilename() const `API change`
    + ***Replacement*** std::string MeshShape::SetMesh(const std::string &_uri, const std::string &_submesh = "", bool _center = false) const
1. **gazebo/common/Time.hh**
    + ***Removed*** static Time::NSleep(Time _time) `API change`
    + ***Replacement*** static Time NSleep(unsigned int _ns)

### Deletions

1. **gazebo/physics/Collision.hh**
    + template<typename T> event::ConnectionPtr ConnectContact(T _subscriber)
    + template<typename T> event::ConnectionPtr DisconnectContact(T _subscriber)
    + ***Note:*** The ContactManager::CreateFilter functions can be used to
      create a gazebo topic with contact messages filtered by the name(s)
      of collision shapes. The topic can then be subscribed with a callback
      to replicate this removed functionality. See
      [gazebo pull request #713](https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/713)
      for an example migration.

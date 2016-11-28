# Note on deprecations
A tick-tock release cycle allows easy migration to new software versions.
Obsolete Gazebo code is marked as deprecated for one major release.
Deprecated code produces compile-time warnings. These warning serve as
notification to users that their code should be upgraded. The next major
release will remove the deprecated code.

## Gazebo 7.X to 8.X

### Additions

1. **gazebo/common/Event.hh**
    + public: bool Connection::Id() const;
    + public: bool Event::Signaled() const;
    + public: void Event::SetSignaled(const bool);

### Modifications

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

1. **gazebo/rendering/Visual.hh**
    + ***Deprecation:*** public: void SetScale(const math::Vector3 &_scale)
    + ***Replacement:*** public: void SetScale(const ignition::math::Vector3d &_scale)
    + ***Deprecation:*** public: void SetPosition(const math::Vector3 &_pos)
    + ***Replacement:*** public: void SetPosition(const ignition::math::Vector3d &_pos)

1. **gazebo/rendering/Camera.hh**
    + ***Deprecation:*** public: virtual void SetWorldPose(const math::Pose &_pose)
    + ***Replacement:*** public: virtual void SetWorldPose(const ignition::math::Pose3d &_pose)

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

1. **gazebo/physics/Joint.hh**
    + ***Deprecation:*** public: void Joint::DisconnectJointUpdate(ConnectionPtr);
    + ***Replacement:*** Delete the Connection object, perhaps by calling
    reset() on its smart pointer.

1. **gazebo/physics/Link.hh**
    + ***Deprecation:*** public: void Link::DisconnectEnabled(ConnectionPtr);
    + ***Replacement:*** Delete the Connection object, perhaps by calling
    reset() on its smart pointer.

1. **gazebo/physics/MultiRayShape.hh**
    + ***Deprecation:*** public: void MultiRayShape::DisconnectNewLaserScans(ConnectionPtr);
    + ***Replacement:*** Delete the Connection object, perhaps by calling
    reset() on its smart pointer.

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
    + ***Deprecation:*** public:   gazebo::math::Filter
    + ***Replacement:*** public: ignition::math::Filter
    + ***Deprecation:*** public:   gazebo::math::OnePole
    + ***Replacement:*** public: ignition::math::OnePole

1. **gazebo/math/Helpers.hh**
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

1. **gazebo/math/Kmeans.hh**
    + ***Deprecation:*** public:   gazebo::math::Kmeans
    + ***Replacement:*** public: ignition::math::Kmeans

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

1. **gazebo/math/Vector3Stats.hh**
    + ***Deprecation:*** public:   gazebo::math::Vector3Stats
    + ***Replacement:*** public: ignition::math::Vector3Stats

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

## Gazebo 7.3.1 to 7.4

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

    + [pull request #2394](https://bitbucket.org/osrf/gazebo/pull-request/2394)

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
    + [pull request #2079](https://bitbucket.org/osrf/gazebo/pull-request/2079)

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
    + [pull request #1924](https://bitbucket.org/osrf/gazebo/pull-request/1924)

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
    + [Pull request #1942](https://bitbucket.org/osrf/gazebo/pull-request/1942)

1. **Light topic**
    + ***Removed:*** ~/light
    + ***Replacement:*** ~/factory/light - for spawning new lights
    + ***Replacement:*** ~/light/modify - for modifying existing lights
    * [Pull request #1920](https://bitbucket.org/osrf/gazebo/pull-request/1920)

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
    * [Pull request #1874](https://bitbucket.org/osrf/gazebo/pull-request/1874)

1. **gazebo/gui/building/BuildingMaker.hh**
    * Doesn't inherit from gui::EntityMaker anymore
    * [Pull request #1828](https://bitbucket.org/osrf/gazebo/pull-request/1828)

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
    * [Pull request #1817](https://bitbucket.org/osrf/gazebo/pull-request/1817)

1. **gazebo physics libraries**
    * The following libraries have been removed: `libgazebo_ode_physics`, `libgazebo_simbody_physics`, `libgazebo_dart_physics`, and `libgazebo_bullet_physics`. Gazebo now combines all the different physics engine libraries into `libgazebo_physics.so`.
    * [Pull request #1814](https://bitbucket.org/osrf/gazebo/pull-request/1814)

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
    * [Pull request #1777](https://bitbucket.org/osrf/gazebo/pull-request/1777)

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
    + [Gazebo migration](https://bitbucket.org/osrf/gazebo/src/583edbeb90759d43d994cc57c0797119dd6d2794/ign-math-migration.md)
    * [Pull request #1756](https://bitbucket.org/osrf/gazebo/pull-request/1756)
    * [Pull request #1766](https://bitbucket.org/osrf/gazebo/pull-request/1766)
    * [Pull request #1774](https://bitbucket.org/osrf/gazebo/pull-request/1774)
    * [Pull request #1771](https://bitbucket.org/osrf/gazebo/pull-request/1771)
    * [Pull request #1776](https://bitbucket.org/osrf/gazebo/pull-request/1776)
    * [Pull request #1777](https://bitbucket.org/osrf/gazebo/pull-request/1777)
    * [Pull request #1772](https://bitbucket.org/osrf/gazebo/pull-request/1772)
    * [Pull request #1773](https://bitbucket.org/osrf/gazebo/pull-request/1773)
    * [Pull request #1778](https://bitbucket.org/osrf/gazebo/pull-request/1778)

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
      [gazebo pull request #713](https://bitbucket.org/osrf/gazebo/pull-request/713)
      for an example migration.

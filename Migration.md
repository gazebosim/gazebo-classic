## Gazebo 4.X to 5.X

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

## Gazebo 2.0 to 3.0

### Modifications

1. **ConnectionManager::Init** `ABI change`
    + *Compiliance:* No changes to downstream code required.
    + *Note:* A third parameter has been added that specifies the number of timeout iterations. This parameter has a default value of 30.
1. **transport::init** `ABI change`
    + *Compiliance:* No changes to downstream code required.
    + *Note:* A third parameter has been added that specifies the number of timeout iterations. This parameter has a default value of 30.

### Additions

### Deletions

## Gazebo 1.9 to 2.0

### New Deprecations

1. **gazebo/sensors/SensorsIface.hh**
    + ***Deprecation*** std::string sensors::create_sensor(sdf::ElementPtr _elem, const std::string &_worldName,const std::string &_parentName)
    + ***Replacement*** std::string sensors::create_sensor(sdf::ElementPtr _elem, const std::string &_worldName, const std::string &_parentName, uint32_t _parentId);
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
1. **gazebo/physics/Joint.hh** **Joint::Load(LinkPtr _parent, LinkPtr _child, const math::Vector3 &_pos)** `ABI & API chance`
    + Compliance: Use Joint::Load(LinkPtr _parent, LinkPtr _child, const math::Pose &_pose)

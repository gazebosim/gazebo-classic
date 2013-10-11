## Gazebo 2.0 to 3.0

### Modifications

1. **ConnectionManager::Init** `ABI change`

    *Compiliance:* No changes to downstream code required.

    *Note:* A third parameter has been added that specifies the number of timeout iterations. This parameter has a default value of 30.

1. **transport::init** `ABI change`

    *Compiliance:* No changes to downstream code required.

    *Note:* A third parameter has been added that specifies the number of timeout iterations. This parameter has a default value of 30.

### Additions

### Deletions

## Gazebo 1.9 to 2.0

### New Deprecations

1. **gazebo/sensors/SensorsIface.hh**
  1. *Deprecattion* std::string sensors::create_sensor(sdf::ElementPtr _elem, const std::string &_worldName,const std::string &_parentName)
    1. *Replacement* std::string sensors::create_sensor(sdf::ElementPtr _elem, const std::string &_worldName, const std::string &_parentName, uint32_t _parentId);
1. **gazebo/sensors/Sensor.hh**
  1. *Deprecation* void Sensor::SetParent(const std::string &_name)
    1. *Replacement* void Sensor::SetParent(const std::string &_name, uint32_t _id)
1. **gazebo/sensors/SensorManager.hh**
  1. *Deprecation* std::string CreateSensor(sdf::ElementPtr _elem, const std::string &_worldName,  const std::string &_parentName)
   1. *Replacement* std::string CreateSensor(sdf::ElementPtr _elem, const std::string &_worldName, const std::string &_parentName, uint32_t _parentId)
1. **gazebo/sensors/Collision.hh**
  1. *Deprecation* void Collision::SetContactsEnabled(bool _enable)
    1. *Replacement* Use ContactManager.

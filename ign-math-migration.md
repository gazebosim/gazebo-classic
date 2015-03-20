# Migration from `gazebo::math` to `ignition::math`

`libigntion-math` is a dependency of Gazebo. See: [http://ignitionrobotics.org/libraries/math](http://ignitionrobotics.org/libraries/math)

## gazebo/common

1. **gazebo/common/KeyFrame.hh**
    + ***Deprecated*** public: void SetTranslation(const math::Vector3 &_trans)
    + ***Replacement*** public: void Translation(const ignition::math::Vector3d &_trans);

    + ***Deprecated*** public: math::Vector3 GetTranslation() const
    + ***Replacement*** public: ignition::math::Vector3d Translation() const

    + ***Deprecated*** public: void SetRotation(const math::Quaternion &_rot)
    + ***Replacement*** public: void Rotation(const ignition::math::Quaterniond &_rot)

    + ***Deprecated*** public: math::Quaternion GetRotation() const
    + ***Replacement*** public: ignition::math::Quaterniond Rotation() const


## gazebo/math

### Angle.hh

    + ***Added*** public: Angle(const ignition::math::Angle &_angle)
    + ***Added*** public: ignition::math::Angle Ign() const
    + ***Added*** public: Angle &operator=(const double &_angle)
    + ***Added*** public: Angle &operator=(const ignition::math::Angle &_angle)
    

### Box.hh

    + ***Added*** public: Box(const ignition::math::Box &_box)
    + ***Added*** public: ignition::math::Box Ign() const
    + ***Added***  public: Box &operator=(const ignition::math::Box &_b)

### Quaternion.hh

    + ***Added*** public: Quaternion(const ignition::math::Quaterniond &_qt)
    + ***Added*** public: ignition::math::Quaterniond Ign() const
    + ***Added*** public: Quaternion &operator =(const ignition::math::Quaterniond &_v)


### Vector3.hh

  + ***Added*** public: Vector3(const ignition::math::Vector3d &_v)
  + ***Added*** public: ignition::math::Vector3d Ign() const
  + ***Added*** public: Vector3 &operator=(const ignition::math::Vector3d &_v)


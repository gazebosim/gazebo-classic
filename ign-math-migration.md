# Migration from `gazebo::math` to `ignition::math`

`libigntion-math` is a dependency of Gazebo. See: [http://ignitionrobotics.org/libraries/math](http://ignitionrobotics.org/libraries/math)

## gazebo/common

### KeyFrame.hh
1. ***Deprecated*** public: void SetTranslation(const math::Vector3 &_trans)
    + ***Replacement*** public: void Translation(const ignition::math::Vector3d &_trans);
1. ***Deprecated*** public: math::Vector3 GetTranslation() const
    + ***Replacement*** public: ignition::math::Vector3d Translation() const
1. ***Deprecated*** public: void SetRotation(const math::Quaternion &_rot)
    + ***Replacement*** public: void Rotation(const ignition::math::Quaterniond &_rot)
1. ***Deprecated*** public: math::Quaternion GetRotation() const
    + ***Replacement*** public: ignition::math::Quaterniond Rotation() const

## gazebo/math

### Angle.hh
1. ***Added*** public: Angle(const ignition::math::Angle &_angle)
1. ***Added*** public: ignition::math::Angle Ign() const
1. ***Added*** public: Angle &operator=(const double &_angle)
1. ***Added*** public: Angle &operator=(const ignition::math::Angle &_angle)

### Box.hh
1. ***Added*** public: Box(const ignition::math::Box &_box)
1. ***Added*** public: ignition::math::Box Ign() const
1. ***Added***  public: Box &operator=(const ignition::math::Box &_b)

### Quaternion.hh
1. ***Added*** public: Quaternion(const ignition::math::Quaterniond &_qt)
1. ***Added*** public: ignition::math::Quaterniond Ign() const
1. ***Added*** public: Quaternion &operator =(const ignition::math::Quaterniond &_v)

### Vector3.hh
1. ***Added*** public: Vector3(const ignition::math::Vector3d &_v)
1. ***Added*** public: ignition::math::Vector3d Ign() const
1. ***Added*** public: Vector3 &operator=(const ignition::math::Vector3d &_v)

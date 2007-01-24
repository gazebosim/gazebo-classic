#ifndef POSE3D_HH
#define POSE3D_HH

#include <iostream>

#include "Vector3.hh"
#include "Quatern.hh"

class Pose3d;
std::ostream &operator<<(std::ostream &out, const Pose3d &);

class Pose3d
{
  // Constructors
  public: Pose3d();
  public: Pose3d( const Vector3 &pos, const Quatern &rot);
  public: Pose3d( const Pose3d &pose );

  // Destructors
  public: virtual ~Pose3d();

  // See if a pose is finite (e.g., not nan)
  public: bool IsFinite();

  public: Pose3d operator+(const Pose3d &pose);
  public: const Pose3d &operator+=(const Pose3d &pose);

  public: Pose3d operator-(const Pose3d &pose);
  public: const Pose3d &operator-=(const Pose3d &pose);

  // Add one point to another: result = this + pose
  public: Vector3 CoordPositionAdd(const Pose3d &pose);

  // Subtract one position from another: result = this - pose
  public: Vector3 CoordPositionSub(const Pose3d &pose);

  // Add one rotation to another: result =  this->rot + rot
  public: Quatern CoordRotationAdd(const Quatern &rot);

  // Subtract one rotation from another: result = this->rot - rot
  public: Quatern CoordRotationSub(const Quatern &rot);

  public: void Reset();

  // Ostream operator
  public: friend std::ostream &operator<< (std::ostream &out, const Pose3d &pose);


  public: Vector3 pos;
  public: Quatern rot;
};
#endif

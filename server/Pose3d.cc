#include "Pose3d.hh"

// Constructors
Pose3d::Pose3d()
{
}

Pose3d::Pose3d( const Vector3 &pos, const Quatern &rot)
{
  this->pos = pos;
  this->rot = rot;
}

Pose3d::Pose3d( const Pose3d &pose )
{
  this->pos = pose.pos;
  this->rot = pose.rot;
}

// Destructors
Pose3d::~Pose3d()
{
}

// See if a pose is finite (e.g., not nan)
bool Pose3d::IsFinite()
{
  return this->pos.IsFinite() && this->rot.IsFinite();
}


// Add two poses: result = this + obj 
Pose3d Pose3d::operator+(const Pose3d &obj)
{
  Pose3d result;

  result.pos = this->CoordPositionAdd(obj);
  result.rot = this->CoordRotationAdd(obj.rot);

  return result;
}

const Pose3d &Pose3d::operator+=(const Pose3d &obj)
{
  this->pos = this->CoordPositionAdd(obj);
  this->rot = this->CoordRotationAdd(obj.rot);

  return *this;
}

// Add two poses: result = this - obj 
Pose3d Pose3d::operator-(const Pose3d &obj)
{
  Pose3d result;

  result.pos = this->CoordPositionSub(obj);
  result.rot = this->CoordRotationSub(obj.rot);

  return result;
}

// Add two poses: result = this - obj 
const Pose3d &Pose3d::operator-=(const Pose3d &obj)
{
  this->pos = this->CoordPositionSub(obj);
  this->rot = this->CoordRotationSub(obj.rot);

  return *this;
}

// Add one point to another: result = this + pose
Vector3 Pose3d::CoordPositionAdd(const Pose3d &pose)
{
  Quatern tmp;
  Vector3 result;

  // result = pose.rot + pose.rot * this->pos * pose.rot!
  tmp.u = 0.0;
  tmp.x = this->pos.x;
  tmp.y = this->pos.y;
  tmp.z = this->pos.z;

  tmp = pose.rot * (tmp * pose.rot.GetInverse());
  result.x = pose.pos.x + tmp.x;
  result.y = pose.pos.y + tmp.y;
  result.z = pose.pos.z + tmp.z;

  return result;
}

// Subtract one position from another: result = this - pose
Vector3 Pose3d::CoordPositionSub(const Pose3d &pose)
{
  Quatern tmp;
  Vector3 result;

  // result = pose.rot! * (this->pos - pose.pos) * pose.rot
  tmp.x = (this->pos - pose.pos).x;
  tmp.y = (this->pos - pose.pos).y;
  tmp.z = (this->pos - pose.pos).z;
  tmp.u = 0.0;

  tmp = pose.rot.GetInverse() * (tmp * pose.rot);

  result.x = tmp.x;
  result.y = tmp.y;
  result.z = tmp.z;

  return result;
}

// Add one rotation to another: result =  this->rot + rot
Quatern Pose3d::CoordRotationAdd(const Quatern &rot)
{
  return  Quatern(rot * this->rot);
}

// Subtract one rotation from another: result = this->rot - rot
Quatern Pose3d::CoordRotationSub(const Quatern &rot)
{
  return rot.GetInverse() * this->rot;
}

////////////////////////////////////////////////////////////////////////////////
// Ostream operator
std::ostream &operator<<( std::ostream &out, const Pose3d &pose )
{
  out << "Pos[" << pose.pos << "] Rot[" << pose.rot << "]";

  return out;
}

void Pose3d::Reset()
{
  // set the position to zero
  this->pos.Set();
  this->rot.SetToIdentity();
}

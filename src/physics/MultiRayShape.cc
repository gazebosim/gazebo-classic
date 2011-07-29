/*
 * Copyright 2011 Nate Koenig & Andrew Howard
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include "msgs/msgs.h"
#include "physics/MultiRayShape.hh"

using namespace gazebo;
using namespace physics;


////////////////////////////////////////////////////////////////////////////////
/// Constructor
MultiRayShape::MultiRayShape(GeomPtr parent) 
  : Shape(parent)
{
  this->AddType(MULTIRAY_SHAPE);
  this->SetName("multiray");

  this->rayFanMsg = new msgs::Visual();
  this->rayFanMsg->mutable_header()->set_str_id( this->GetName()+"_fan" );
  this->rayFanMsg->set_parent_id( this->geomParent->GetName() );
  this->rayFanMsg->set_render_type( msgs::Visual::TRIANGLE_FAN );
  this->rayFanMsg->set_material_script( "Gazebo/BlueLaser" );

  this->rayFanOutlineMsg = new msgs::Visual();
  this->rayFanOutlineMsg->mutable_header()->set_str_id( this->GetName()+"_fanoutline" );
  this->rayFanOutlineMsg->set_parent_id( this->geomParent->GetName() );
  this->rayFanOutlineMsg->set_render_type( msgs::Visual::LINE_STRIP );
  this->rayFanOutlineMsg->set_material_script( "Gazebo/BlueGlow" );
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
MultiRayShape::~MultiRayShape()
{
  this->rays.clear();
}

////////////////////////////////////////////////////////////////////////////////
// Load a multi-ray shape from xml file
void MultiRayShape::Load( sdf::ElementPtr &_sdf)
{
  Shape::Load(_sdf);
}

////////////////////////////////////////////////////////////////////////////////
/// Init the shape 
void MultiRayShape::Init()
{
  //gzerr << "initialize MultiRayShape " << this->GetName() << "\n";
  math::Vector3 start, end, axis;
  double yawAngle, pitchAngle; 
  double pDiff, yDiff;
  double horzMinAngle, horzMaxAngle;
  double vertMinAngle, vertMaxAngle;

  int horzSamples = 1;
  double horzResolution = 1.0;
  int vertSamples = 1;
  double vertResolution = 1.0;

  double minRange, maxRange;

  sdf::ElementPtr rayElem = this->sdf->GetElement("ray");
  sdf::ElementPtr scanElem = rayElem->GetElement("scan");
  sdf::ElementPtr horzElem = scanElem->GetElement("horizontal");
  sdf::ElementPtr vertElem = scanElem->GetElement("vertical");
  sdf::ElementPtr rangeElem = rayElem->GetElement("range");

  if (vertElem)
  {
    vertMinAngle = vertElem->GetValueDouble("min_angle");
    vertMaxAngle = vertElem->GetValueDouble("max_angle");
    vertSamples = vertElem->GetValueUInt("samples");
    vertResolution = vertElem->GetValueDouble("resolution");
    pDiff = vertMaxAngle - vertMinAngle;
  }

  horzMinAngle = horzElem->GetValueDouble("min_angle");
  horzMaxAngle = horzElem->GetValueDouble("max_angle");
  horzSamples = horzElem->GetValueUInt("samples");
  horzResolution = horzElem->GetValueDouble("resolution");
  yDiff = horzMaxAngle - horzMinAngle;

  minRange = rangeElem->GetValueDouble("min");
  maxRange = rangeElem->GetValueDouble("max");

  // Create and array of ray geoms
  for (int j = 0; j < vertSamples; j++)
  {
    for (int i = 0; i < horzSamples; i++)
    {
      yawAngle = (horzSamples == 1) ? 0 : 
        i * yDiff / (horzSamples - 1) + horzMinAngle;

      pitchAngle = (vertSamples == 1)? 0 :  
        j * pDiff / (vertSamples - 1) + vertMinAngle;

      axis.Set(cos(pitchAngle) * cos(yawAngle), 
               sin(yawAngle), sin(pitchAngle)* cos(yawAngle));

      start = (axis * minRange) + this->offset;
      end = (axis * maxRange) + this->offset;

      this->AddRay(start,end);
    }
  }

  //TODO: this doesn't belong here
  //if (**this->displayTypeP == "fan")
  {
    msgs::Point *pt = this->rayFanMsg->add_points();
    (*pt) = this->rayFanMsg->points(0);

    pt = this->rayFanOutlineMsg->add_points();
    (*pt) = this->rayFanOutlineMsg->points(0);
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Get detected range for a ray.
/// \returns Returns DBL_MAX for no detection.
double MultiRayShape::GetRange(int index)
{
  if (index < 0 || index >= (int)this->rays.size())
  {
    std::ostringstream stream;
    stream << "index[" << index << "] out of range[0-"
      << this->rays.size() << "]";
    gzthrow(stream.str());
  }

  return this->rays[index]->GetLength();
}

////////////////////////////////////////////////////////////////////////////////
/// Get detected retro (intensity) value for a ray.
double MultiRayShape::GetRetro(int index)
{
  if (index < 0 || index >= (int)this->rays.size())
  {
    std::ostringstream stream;
    stream << "index[" << index << "] out of range[0-"
      << this->rays.size() << "]";
    gzthrow(stream.str());
  }

  return this->rays[index]->GetRetro();
}

////////////////////////////////////////////////////////////////////////////////
/// Get detected fiducial value for a ray.
int MultiRayShape::GetFiducial(int index)
{
  if (index < 0 || index >= (int)this->rays.size())
  {
    std::ostringstream stream;
    stream << "index[" << index << "] out of range[0-"
      << this->rays.size() << "]";
    gzthrow(stream.str());
  }

  return this->rays[index]->GetFiducial();
}

////////////////////////////////////////////////////////////////////////////////
/// Update the geom
void MultiRayShape::Update()
{
  std::vector<RayShapePtr>::iterator iter;
  math::Vector3 a, b;
  int i = 1;

  sdf::ElementPtr rayElem = this->sdf->GetElement("ray");
  sdf::ElementPtr rangeElem = rayElem->GetElement("range");
  double maxRange = rangeElem->GetValueDouble("max");

  // Reset the ray lengths and mark the geoms as dirty (so they get
  // redrawn)
  for (iter = this->rays.begin(); 
      iter != this->rays.end(); iter++, i++)
  {
    (*iter)->SetLength( maxRange );
    (*iter)->SetRetro( 0.0 );

    // Get the global points of the line
    (*iter)->Update();
  }

  this->UpdateRays();

  //TODO: move to rendering engine
  /*if (**this->displayTypeP == "fan")*/
  { 
    i = 1;
    for (iter = this->rays.begin(); 
        iter != this->rays.end(); iter++, i++)
    {
      (*iter)->Update();

      (*iter)->GetRelativePoints(a,b);

      msgs::Set(this->rayFanMsg->mutable_points(i), b );
      msgs::Set(this->rayFanOutlineMsg->mutable_points(i), b );
      //gzdbg << "ray [" << i << "]"
      //      << " length [" << (*iter)->GetLength() << "]\n";
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Add a ray to the geom
void MultiRayShape::AddRay(const math::Vector3 &start, 
                           const math::Vector3 &end )
{
  msgs::Point *pt = NULL;

  //TODO: move to rendering engine
  // Add to the renderable
  /*if (**this->displayTypeP == "fan")*/
  {
    if (this->rayFanMsg->points_size() == 0)
    {
      pt = this->rayFanMsg->add_points();
      msgs::Set(pt, start );

      pt = this->rayFanOutlineMsg->add_points();
      msgs::Set(pt, start);
    }

    pt = this->rayFanMsg->add_points();
    msgs::Set(pt, end);

    pt = this->rayFanOutlineMsg->add_points();
    msgs::Set(pt, end);
  }
}

//////////////////////////////////////////////////////////////////////////////
/// Get the minimum angle
math::Angle MultiRayShape::GetMinAngle() const
{
  return this->sdf->GetElement("scan")->GetElement("horizontal")->GetValueDouble("min_angle");
}

//////////////////////////////////////////////////////////////////////////////
/// Get the maximum angle
math::Angle MultiRayShape::GetMaxAngle() const
{
  return this->sdf->GetElement("scan")->GetElement("horizontal")->GetValueDouble("max_angle");
}

//////////////////////////////////////////////////////////////////////////////
/// Get the minimum range
double MultiRayShape::GetMinRange() const
{
  return this->sdf->GetElement("range")->GetValueDouble("min");
}

//////////////////////////////////////////////////////////////////////////////
///  Get the maximum range
double MultiRayShape::GetMaxRange() const
{
  return this->sdf->GetElement("range")->GetValueDouble("max");
}

//////////////////////////////////////////////////////////////////////////////
///  Get the range resolution
double MultiRayShape::GetResRange() const
{
  return this->sdf->GetElement("range")->GetValueDouble("resolution");
}

//////////////////////////////////////////////////////////////////////////////
/// Get the sample count
int MultiRayShape::GetSampleCount() const
{
  return this->sdf->GetElement("scan")->GetElement("horizontal")->GetValueDouble("samples");
}

//////////////////////////////////////////////////////////////////////////////
/// Get the vertical scan line count
int MultiRayShape::GetVerticalSampleCount() const
{
  return this->sdf->GetElement("scan")->GetElement("vertical")->GetValueDouble("samples");
}

//////////////////////////////////////////////////////////////////////////////
/// Get the vertical min angle
math::Angle MultiRayShape::GetVerticalMinAngle() const
{
  return this->sdf->GetElement("scan")->GetElement("vertical")->GetValueDouble("min_angle");
}

//////////////////////////////////////////////////////////////////////////////
/// Get the vertical max angle
math::Angle MultiRayShape::GetVerticalMaxAngle() const
{
  return this->sdf->GetElement("scan")->GetElement("vertical")->GetValueDouble("max_angle");
}

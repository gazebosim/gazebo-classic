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
#include "common/Messages.hh"
#include "common/XMLConfig.hh"
#include "physics/MultiRayShape.hh"

using namespace gazebo;
using namespace physics;


////////////////////////////////////////////////////////////////////////////////
/// Constructor
MultiRayShape::MultiRayShape(Geom *parent) : Shape(parent)
{
  this->AddType(MULTIRAY_SHAPE);
  this->SetName("multiray");

  this->rayFanMsg = new msgs::Visual();
  this->rayFanMsg->mutable_header()->set_str_id( this->GetName()+"_fan" );
  this->rayFanMsg->set_parent_id( this->geomParent->GetName() );
  this->rayFanMsg->set_render_type( msgs::Visual::TRIANGLE_FAN );
  this->rayFanMsg->set_material( "Gazebo/BlueLaser" );

  this->rayFanOutlineMsg = new msgs::Visual();
  this->rayFanOutlineMsg->mutable_header()->set_str_id( this->GetName()+"_fanoutline" );
  this->rayFanOutlineMsg->set_parent_id( this->geomParent->GetName() );
  this->rayFanOutlineMsg->set_render_type( msgs::Visual::LINE_STRIP );
  this->rayFanOutlineMsg->set_material( "Gazebo/BlueGlow" );

  common::Param::Begin(&this->parameters);
  this->rayCountP = new common::ParamT<int>("ray_count",0,1);
  this->rangeCountP = new common::ParamT<int>("range_count",0,1);
  this->minAngleP = new common::ParamT<common::Angle>("min_angle",DTOR(-90),1);
  this->maxAngleP = new common::ParamT<common::Angle>("max_angle",DTOR(-90),1);
  this->minRangeP = new common::ParamT<double>("min_range",0,1);
  this->maxRangeP = new common::ParamT<double>("max_range",0,1);
  this->resRangeP = new common::ParamT<double>("res_range",0.1,1);
  this->originP = new common::ParamT<common::Vector3>("origin", common::Vector3(0,0,0), 0);
  this->displayTypeP = new common::ParamT<std::string>("display_rays", "off", 0);

  // for block rays, vertical setting
  this->verticalRayCountP = new common::ParamT<int>("vertical_ray_count", 1, 0);
  this->verticalRangeCountP = new common::ParamT<int>("vertical_range_count", 1, 0);
  this->verticalMinAngleP = new common::ParamT<common::Angle>("vertical_min_angle", DTOR(0), 0);
  this->verticalMaxAngleP = new common::ParamT<common::Angle>("vertical_max_angle", DTOR(0), 0);
  common::Param::End();
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
MultiRayShape::~MultiRayShape()
{
  std::vector< RayShape* >::iterator iter;

  for (iter=this->rays.begin(); iter!=this->rays.end(); iter++)
  {
    delete *iter;
  }
  this->rays.clear();

  delete this->rayCountP;
  delete this->rangeCountP;
  delete this->minAngleP;
  delete this->maxAngleP;
  delete this->minRangeP;
  delete this->maxRangeP;
  delete this->resRangeP;
  delete this->originP;
  delete this->displayTypeP;

  delete this->verticalRayCountP;
  delete this->verticalRangeCountP;
  delete this->verticalMinAngleP;
  delete this->verticalMaxAngleP;

}

////////////////////////////////////////////////////////////////////////////////
// Load a multi-ray shape from xml file
void MultiRayShape::Load(common::XMLConfigNode *node)
{
  this->rayCountP->Load(node);
  this->rangeCountP->Load(node);
  this->minAngleP->Load(node);
  this->maxAngleP->Load(node);
  this->minRangeP->Load(node);
  this->maxRangeP->Load(node);
  this->resRangeP->Load(node);
  this->originP->Load(node);
  this->displayTypeP->Load(node);
  this->verticalRayCountP->Load(node);
  this->verticalRangeCountP->Load(node);
  this->verticalMinAngleP->Load(node);
  this->verticalMaxAngleP->Load(node);
//}

////////////////////////////////////////////////////////////////////////////////
/*void MultiRayShape::Load(unsigned int vertRayCount, unsigned int rayCount,
            common::Vector3 origin, double minRange, double maxRange,
            common::Angle minVertAngle, common::Angle maxVertAngle,
            common::Angle minAngle, common::Angle maxcommon::Angle )
{
*/
  common::Vector3 start, end, axis;

  common::Angle yawAngle, pitchAngle; 
  common::Angle pDiff = **this->verticalMaxAngleP - **this->verticalMinAngleP;
  common::Angle yDiff = **this->maxAngleP - **this->minAngleP;

  //this->maxRange = maxRange;

  // Create and array of ray geoms
  for (int j = 0; j < **this->verticalRayCountP; j++)
  {
    for (int i = 0; i < **this->rayCountP; i++)
    {
      yawAngle = (**this->rayCountP == 1)? 0 : 
        i * yDiff.GetAsRadian() / (**this->rayCountP - 1) + 
        (**this->minAngleP).GetAsRadian();

      pitchAngle = (**this->verticalRayCountP == 1)? 0 :  
        j * pDiff.GetAsRadian() / (**this->verticalRayCountP - 1) + 
        (**this->verticalMinAngleP).GetAsRadian();

      axis.Set(cos(pitchAngle.GetAsRadian()) * 
          cos(yawAngle.GetAsRadian()), 
          sin(yawAngle.GetAsRadian()), 
          sin(pitchAngle.GetAsRadian())*
          cos(yawAngle.GetAsRadian()));

      start = (axis * **this->minRangeP) + **this->originP;
      end = (axis * **this->maxRangeP) + **this->originP;

      this->AddRay(start,end);
    }
  }

  if (**this->displayTypeP == "fan")
  {
    msgs::Point *pt = this->rayFanMsg->add_points();
    (*pt) = this->rayFanMsg->points(0);

    pt = this->rayFanOutlineMsg->add_points();
    (*pt) = this->rayFanOutlineMsg->points(0);
  }
}

//////////////////////////////////////////////////////////////////////////////
/// Save the sensor info in XML format
void MultiRayShape::Save(std::string &prefix, std::ostream &stream)
{
  stream << prefix << "  " << *(this->minAngleP) << "\n";
  stream << prefix << "  " << *(this->maxAngleP) << "\n";
  stream << prefix << "  " << *(this->minRangeP) << "\n";
  stream << prefix << "  " << *(this->maxRangeP) << "\n";
  stream << prefix << "  " << *(this->resRangeP) << "\n";
  stream << prefix << "  " << *(this->originP) << "\n";
  stream << prefix << "  " << *(this->rayCountP) << "\n";
  stream << prefix << "  " << *(this->rangeCountP) << "\n";
  stream << prefix << "  " << *(this->displayTypeP) << "\n";
  stream << prefix << "  " << *(this->verticalRayCountP) << "\n";
  stream << prefix << "  " << *(this->verticalRangeCountP) << "\n";
  stream << prefix << "  " << *(this->verticalMinAngleP) << "\n";
  stream << prefix << "  " << *(this->verticalMaxAngleP) << "\n";
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
  std::vector<RayShape*>::iterator iter;
  common::Vector3 a, b;
  int i = 1;

  // Reset the ray lengths and mark the geoms as dirty (so they get
  // redrawn)
  for (iter = this->rays.begin(); 
      iter != this->rays.end(); iter++, i++)
  {
    (*iter)->SetLength( **this->maxRangeP );
    (*iter)->SetRetro( 0.0 );
    (*iter)->SetFiducial( -1 );

    // Get the global points of the line
    (*iter)->Update();
  }

  this->UpdateRays();

  if (**this->displayTypeP == "fan")
  { 
    i = 1;
    for (iter = this->rays.begin(); 
        iter != this->rays.end(); iter++, i++)
    {
      (*iter)->Update();

      (*iter)->GetRelativePoints(a,b);

      common::Message::Set(this->rayFanMsg->mutable_points(i), b );
      common::Message::Set(this->rayFanOutlineMsg->mutable_points(i), b );
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Add a ray to the geom
void MultiRayShape::AddRay(const common::Vector3 &start, const common::Vector3 &end )
{
  msgs::Point *pt = NULL;

  // Add to the renderable
  if (**this->displayTypeP == "fan")
  {
    if (this->rayFanMsg->points_size() == 0)
    {
      pt = this->rayFanMsg->add_points();
      common::Message::Set(pt, start );

      pt = this->rayFanOutlineMsg->add_points();
      common::Message::Set(pt, start);
    }

    pt = this->rayFanMsg->add_points();
    common::Message::Set(pt, end);

    pt = this->rayFanOutlineMsg->add_points();
    common::Message::Set(pt, end);
  }
}

//////////////////////////////////////////////////////////////////////////////
/// Get the minimum angle
common::Angle MultiRayShape::GetMinAngle() const
{
  return **this->minAngleP;
}

//////////////////////////////////////////////////////////////////////////////
/// Get the maximum angle
common::Angle MultiRayShape::GetMaxAngle() const
{
  return **this->maxAngleP;
}

//////////////////////////////////////////////////////////////////////////////
/// Get the minimum range
double MultiRayShape::GetMinRange() const
{
  return **this->minRangeP;
}

//////////////////////////////////////////////////////////////////////////////
///  Get the maximum range
double MultiRayShape::GetMaxRange() const
{
  return **this->maxRangeP;
}

//////////////////////////////////////////////////////////////////////////////
///  Get the range resolution
double MultiRayShape::GetResRange() const
{
  return **this->resRangeP;
}

//////////////////////////////////////////////////////////////////////////////
/// Get the ray count
int MultiRayShape::GetRayCount() const
{
  return **this->rayCountP;
}

//////////////////////////////////////////////////////////////////////////////
/// Get the range count
int MultiRayShape::GetRangeCount() const
{
  return **this->rangeCountP;
}

//////////////////////////////////////////////////////////////////////////////
/// Get the vertical scan line count
int MultiRayShape::GetVerticalRayCount() const
{
  return **this->verticalRayCountP;
}

//////////////////////////////////////////////////////////////////////////////
/// Get the vertical scan line count
int MultiRayShape::GetVerticalRangeCount() const
{
  return **this->verticalRangeCountP;
}

//////////////////////////////////////////////////////////////////////////////
/// Get the vertical min angle
common::Angle MultiRayShape::GetVerticalMinAngle() const
{
  return **this->verticalMinAngleP;
}

//////////////////////////////////////////////////////////////////////////////
/// Get the vertical max angle
common::Angle MultiRayShape::GetVerticalMaxAngle() const
{
  return **this->verticalMaxAngleP;
}


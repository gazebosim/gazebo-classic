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

#include <google/protobuf/descriptor.h>
#include <algorithm>

#include "math/Vector3.hh"
#include "math/Pose.hh"
#include "math/Quaternion.hh"
#include "math/Plane.hh"

#include "common/Exception.hh"
#include "common/Console.hh"
#include "msgs/msgs.h"

namespace gazebo { namespace msgs {


const google::protobuf::FieldDescriptor *GetFD(google::protobuf::Message &message, const std::string &name)
{
  return message.GetDescriptor()->FindFieldByName(name);
}

msgs::Header *GetHeader(google::protobuf::Message &message)
{
  google::protobuf::Message *msg = NULL;
 
  if (GetFD(message, "str_id"))
    msg = &message;
  else 
  {
    const google::protobuf::FieldDescriptor *fd;
    fd = GetFD(message, "header");

    if (fd)
      msg = message.GetReflection()->MutableMessage(&message, fd);
  }

  return (msgs::Header*)msg;
}

void Init(google::protobuf::Message &message, const std::string &id)
{
  msgs::Header *header = GetHeader(message);

  if ( header )
  {
    header->set_str_id(id);
    Stamp(header->mutable_stamp());
  }
  else
    gzerr << "Header is non-existant\n";
}

void Stamp(msgs::Header *hdr)
{
  Stamp(hdr->mutable_stamp());
}

void Stamp(msgs::Time *time)
{
  common::Time tm = common::Time::GetWallTime();

  time->set_sec(tm.sec);
  time->set_nsec(tm.nsec);
}

std::string Package(const std::string &type, 
                             const google::protobuf::Message &message)
{
  std::string data;
  msgs::Packet pkg;

  Stamp( pkg.mutable_stamp() );
  pkg.set_type(type);

  std::string *serialized_data = pkg.mutable_serialized_data();
  if (!message.SerializeToString(serialized_data))
    gzthrow("Failed to serialized message");

  if (!pkg.SerializeToString(&data))
    gzthrow("Failed to serialized message");

  return data;
}

msgs::Packet Package2(const std::string &type, 
                               const google::protobuf::Message &message)
{
  msgs::Packet pkg;
  Stamp( pkg.mutable_stamp() );
  pkg.set_type(type);

  std::string *serialized_data = pkg.mutable_serialized_data();
  if (!message.SerializeToString(serialized_data))
    gzthrow("Failed to serialized message");

  return pkg;
}

void Set(msgs::Point *pt, const math::Vector3 &v)
{
  pt->set_x(v.x);
  pt->set_y(v.y);
  pt->set_z(v.z);
}

void Set(msgs::Quaternion *q, const math::Quaternion &v)
{
  q->set_x(v.x);
  q->set_y(v.y);
  q->set_z(v.z);
  q->set_w(v.w);
}

void Set(msgs::Pose *p, const math::Pose &v)
{
  Set( p->mutable_position(), v.pos );
  Set( p->mutable_orientation(), v.rot );
}

void Set(msgs::Color *c, const common::Color &v)
{
  c->set_r(v.R());
  c->set_g(v.G());
  c->set_b(v.B());
  c->set_a(v.A());
}

void Set(msgs::Time *t, const common::Time &v)
{
  t->set_sec(v.sec);
  t->set_nsec(v.nsec);
}


void Set(msgs::Plane *p, const math::Plane &v)
{
  Set( p->mutable_normal(), v.normal );
  p->set_size_x( v.size.x );
  p->set_size_y( v.size.y );
  p->set_d( v.d );
}

msgs::Point Convert(const math::Vector3 &v)
{
  msgs::Point result;
  result.set_x(v.x);
  result.set_y(v.y);
  result.set_z(v.z);
  return result;
}

msgs::Quaternion Convert(const math::Quaternion &q)
{
  msgs::Quaternion result;
  result.set_x(q.x);
  result.set_y(q.y);
  result.set_z(q.z);
  result.set_w(q.w);
  return result;
}

msgs::Pose Convert(const math::Pose &p)
{
  msgs::Pose result;
  result.mutable_position()->CopyFrom( Convert(p.pos) );
  result.mutable_orientation()->CopyFrom( Convert(p.rot) );
  return result;
}

msgs::Color Convert(const common::Color &c)
{
  msgs::Color result;
  result.set_r(c.R());
  result.set_g(c.G());
  result.set_b(c.B());
  result.set_a(c.A());
  return result;
}

msgs::Time Convert(const common::Time &t)
{
  msgs::Time result;
  result.set_sec(t.sec);
  result.set_nsec(t.nsec);
  return result;
}

msgs::Plane Convert(const math::Plane &p)
{
  msgs::Plane result;
  result.mutable_normal()->CopyFrom( Convert(p.normal) );
  result.set_size_x( p.size.x );
  result.set_size_y( p.size.y );
  result.set_d( p.d );
  return result;
}

math::Vector3 Convert(const msgs::Point &v)
{
  return math::Vector3(v.x(), v.y(), v.z());
}

math::Quaternion Convert(const msgs::Quaternion &q)
{
  return math::Quaternion(q.w(), q.x(), q.y(), q.z());
}

math::Pose Convert(const msgs::Pose &p)
{
  return math::Pose( Convert(p.position()), 
                 Convert(p.orientation()) );
}

common::Color Convert(const msgs::Color &c)
{
  return common::Color( c.r(), c.g(), c.b(), c.a() );
}

common::Time Convert(const msgs::Time &t)
{
  return common::Time(t.sec(), t.nsec());
}

math::Plane Convert(const msgs::Plane &p)
{
  return math::Plane(Convert(p.normal()), 
               math::Vector2d(p.size_x(), p.size_y()),
               p.d() );
}


msgs::Light LightFromSDF( sdf::ElementPtr _sdf )
{
  msgs::Light result;

  std::string type = _sdf->GetValueString("type");
  std::transform( type.begin(), type.end(), type.begin(), ::tolower);

  Init(result, _sdf->GetValueString("name"));

  result.set_cast_shadows( _sdf->GetValueBool("cast_shadows") );

  if (type == "point")
    result.set_type(msgs::Light::POINT);
  else if (type == "spot")
    result.set_type(msgs::Light::SPOT);
  else if (type == "directional")
    result.set_type(msgs::Light::DIRECTIONAL);

  if (_sdf->HasElement("origin"))
  {
    result.mutable_pose()->CopyFrom( 
        Convert(_sdf->GetElement("origin")->GetValuePose("pose") ) );
  }

  if (_sdf->HasElement("diffuse"))
  {
    result.mutable_diffuse()->CopyFrom( 
        Convert( _sdf->GetElement("diffuse")->GetValueColor("rgba")) );
  }

  if (_sdf->HasElement("specular"))
  {
    result.mutable_specular()->CopyFrom( 
        Convert( _sdf->GetElement("specular")->GetValueColor("rgba")) );
  }

  if (_sdf->HasElement("attenuation"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("attenuation");
    result.set_attenuation_constant(elem->GetValueDouble("constant"));
    result.set_attenuation_linear(elem->GetValueDouble("linear"));
    result.set_attenuation_quadratic(elem->GetValueDouble("quadratic"));
    result.set_range( elem->GetValueDouble("range") );
  }

  if (_sdf->HasElement("direction"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("direction");
    result.mutable_direction()->CopyFrom( 
      Convert( elem->GetValueVector3("xyz") ) );
  }

  if (_sdf->HasElement("spot"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("spot");
    result.set_spot_inner_angle( elem->GetValueDouble("inner_angle") );
    result.set_spot_outer_angle( elem->GetValueDouble("outer_angle") );
    result.set_spot_falloff(     elem->GetValueDouble("falloff") );
  }

  return result;
}

msgs::Visual VisualFromSDF( sdf::ElementPtr _sdf )
{
  msgs::Visual result;

  result.set_cast_shadows( _sdf->GetValueBool("cast_shadows") );
  result.set_transparency( _sdf->GetValueDouble("transparency") );

  // Load the geometry
  if (_sdf->HasElement("geometry"))
  {
    sdf::ElementPtr geomElem = _sdf->GetElement("geometry")->GetFirstElement();
    math::Vector3 scale(1,1,1);

    if (geomElem->GetName() == "box")
    {
      scale = geomElem->GetValueVector3("size");
      result.set_mesh_type( msgs::Visual::BOX );
    }
    else if (geomElem->GetName() == "cylinder")
    {
      scale.x = scale.y = (2*geomElem->GetValueDouble("radius"));
      scale.z = geomElem->GetValueDouble("length");
      result.set_mesh_type( msgs::Visual::CYLINDER );
    }
    else if (geomElem->GetName() == "sphere")
    {
      scale.x = scale.y = scale.z = (2*geomElem->GetValueDouble("radius"));
      result.set_mesh_type( msgs::Visual::SPHERE );
    }
    else if (geomElem->GetName() == "plane")
      result.set_mesh_type( msgs::Visual::PLANE );
    else if (geomElem->GetName() == "image")
    {
      scale.x = scale.y = geomElem->GetValueDouble("scale");
      scale.z = geomElem->GetValueDouble("height");
      result.set_mesh_type( msgs::Visual::IMAGE );
    }
    else if (geomElem->GetName() == "heightmap")
    {
      scale= geomElem->GetValueDouble("size");
      result.set_mesh_type( msgs::Visual::HEIGHTMAP );
    }
    else if (geomElem->GetName() == "mesh")
    {
      scale= geomElem->GetValueDouble("scale");
      result.set_mesh_type( msgs::Visual::MESH );
    }
    else
      gzerr << "Unknown geometry type\n";

    result.mutable_scale()->CopyFrom( msgs::Convert(scale) );

    if (result.mesh_type() == msgs::Visual::IMAGE || 
        result.mesh_type() == msgs::Visual::MESH || 
        result.mesh_type() == msgs::Visual::HEIGHTMAP)
      result.set_filename( geomElem->GetValueString("filename") ); 
  }

  /// Load the material
  if (_sdf->HasElement("material"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("material");
    result.set_material_script(elem->GetValueString("script"));
    if (elem->HasElement("color"))
      result.mutable_material_color()->CopyFrom( 
          Convert(elem->GetElement("color")->GetValueColor("rgba")));
  }

  // Set the origin of the visual
  if (_sdf->HasElement("origin"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("origin");
    result.mutable_pose()->CopyFrom( 
        Convert(elem->GetValuePose("pose")));
  }

  return result;
}

msgs::Shadows ShadowsFromSDF( sdf::ElementPtr _sdf )
{
  msgs::Shadows result;

  std::string type = _sdf->GetValueString("type");
  if (type == "stencil_modulative")
    result.set_type( msgs::Shadows::STENCIL_MODULATIVE);
  else if (type == "stencil_additive")
    result.set_type( msgs::Shadows::STENCIL_ADDITIVE);
  else if (type == "texture_additive")
    result.set_type( msgs::Shadows::TEXTURE_ADDITIVE);
  else if (type == "texture_modulative")
    result.set_type( msgs::Shadows::TEXTURE_MODULATIVE);

  result.mutable_color()->CopyFrom( Convert(_sdf->GetValueColor("rgba")) );

  return result;
}

msgs::Fog FogFromSDF( sdf::ElementPtr _sdf )
{
  msgs::Fog result;

  std::string type = _sdf->GetValueString("type");
  if (type == "linear")
    result.set_type(msgs::Fog::LINEAR);
  else if (type == "exp")
    result.set_type(msgs::Fog::EXPONENTIAL);
  else if (type == "exp2")
    result.set_type(msgs::Fog::EXPONENTIAL2);
  else
    gzerr << "Unknown fog type[" << type << "]\n";

  result.mutable_color()->CopyFrom( Convert(_sdf->GetValueColor("rgba")) );
  result.set_density( _sdf->GetValueDouble("density") );
  result.set_start( _sdf->GetValueDouble("start") );
  result.set_end( _sdf->GetValueDouble("end") );
  return result;
}

msgs::Scene SceneFromSDF(sdf::ElementPtr _sdf)
{
  msgs::Scene result;

  Init(result,"scene");

  if (_sdf->HasElement("ambient"))
    result.mutable_ambient()->CopyFrom( 
        Convert( _sdf->GetElement("ambient")->GetValueColor("rgba")) );

  if (_sdf->HasElement("background"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("background");

    result.mutable_background()->CopyFrom( 
        Convert(elem->GetValueColor("rgba")) );

    if (elem->HasElement("sky"))
      result.set_sky_material( 
          elem->GetElement("sky")->GetValueString("material"));
  }

  if (_sdf->HasElement("fog"))
    result.mutable_fog()->CopyFrom( FogFromSDF(_sdf->GetElement("fog")) );

  if (_sdf->HasElement("shadows"))
    result.mutable_shadows()->CopyFrom( ShadowsFromSDF(_sdf->GetElement("shadows")) );


  return result;
}

} }

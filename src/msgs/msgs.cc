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
    result.mutable_diffuse()->CopyFrom( 
        Convert( _sdf->GetElement("specular")->GetValueColor("rgba")) );
  }

  if (_sdf->HasElement("attenuation"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("attenuation");
    result.set_attenuation_constant(elem->GetValueDouble("constant"));
    result.set_attenuation_linear(elem->GetValueDouble("linear"));
    result.set_attenuation_linear(elem->GetValueDouble("quadratic"));
    result.set_range( elem->GetValueDouble("range") );
  }

  if (_sdf->HasElement("attenuation"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("attenuation");
    result.mutable_direction()->CopyFrom( 
      Convert( elem->GetValueVector3("direction") ) );
  }

  if (_sdf->HasElement("spot"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("spot");
    result.set_spot_inner_angle( elem->GetValueDouble("spot_inner_angle") );
    result.set_spot_outer_angle( elem->GetValueDouble("spot_outer_angle") );
    result.set_spot_falloff(     elem->GetValueDouble("spot_falloff") );
  }

  return result;
}

msgs::Visual VisualFromSDF( sdf::ElementPtr _sdf )
{
  msgs::Visual result;
/*  XMLConfigNode *cnode = NULL;

  result.set_cast_shadows( node->GetBool("cast_shadows",true,0) );
  result.set_visible( node->GetBool("visible",true,0) );
  result.set_transparency( node->GetDouble("transparency",0.0,0) );

  // Load the geometry
  if ( (cnode = node->GetChild("geometry")) != NULL )
  {
    XMLConfigNode *ccnode = NULL;

    ccnode = cnode->GetChild("mesh");
    if (ccnode)
    {
      result.set_mesh( ccnode->GetString("filename","",0) );
      result.mutable_scale()->CopyFrom( 
          Convert( ccnode->GetVector3("scale", math::Vector3(1,1,1)) ) );
    }

    ccnode = cnode->GetChild("cylinder");
    if (ccnode)
    {
      result.set_mesh("unit_cylinder");
      double radius = ccnode->GetDouble("radius",1,1);
      double length = ccnode->GetDouble("length",1,1);
      result.mutable_scale()->set_x(radius*2);
      result.mutable_scale()->set_y(radius*2);
      result.mutable_scale()->set_z(length);
    }

    ccnode = cnode->GetChild("sphere");
    if (ccnode)
    {
      result.set_mesh("unit_sphere");
      double radius = ccnode->GetDouble("radius",1,1);
      result.mutable_scale()->set_x(radius*2);
      result.mutable_scale()->set_y(radius*2);
      result.mutable_scale()->set_z(radius*2);
    }

    ccnode = cnode->GetChild("box");
    if (ccnode)
    {
      result.set_mesh("unit_box");
      result.mutable_scale()->CopyFrom( 
          Convert( ccnode->GetVector3("size", math::Vector3(1,1,1)) ) );
    }

    ccnode = cnode->GetChild("plane");
    if ( ccnode )
    {
      result.mutable_plane()->mutable_normal()->CopyFrom( 
          Convert( ccnode->GetVector3("normal",math::Vector3(0,0,1))) );
      result.mutable_plane()->set_d( ccnode->GetDouble("offset", 0, 0) );
      result.mutable_plane()->set_size_x(ccnode->GetTupleDouble("size", 0, 1));
      result.mutable_plane()->set_size_y(ccnode->GetTupleDouble("size", 1, 1));
    }
  }

  /// Load the material
  if ( (cnode = node->GetChild("material")) != NULL)
  {
    result.set_material( cnode->GetString("name","",1) );
    result.set_uv_tile_x( cnode->GetTupleDouble("uv_tile",0,1) );
    result.set_uv_tile_y( cnode->GetTupleDouble("uv_tile",1,1) );
  }

  // Set the origin of the visual
  if (node->GetChild("origin"))
  {
    result.mutable_pose()->mutable_position()->CopyFrom( 
        Convert(node->GetChild("origin")->GetVector3("xyz",math::Vector3())));

    result.mutable_pose()->mutable_orientation()->CopyFrom( 
        Convert(node->GetChild("origin")->GetRotation("rpy",math::Quaternion())));
  }

*/
  return result;
}

msgs::Shadows ShadowsFromSDF( sdf::ElementPtr _sdf )
{
  msgs::Shadows result;

  /*std::string type = node->GetString("type","stencil_modulative",0);
  if (type == "stencil_modulative")
    result.set_type( msgs::Shadows::STENCIL_MODULATIVE);
  else if (type == "stencil_additive")
    result.set_type( msgs::Shadows::STENCIL_ADDITIVE);
  else if (type == "texture_additive")
    result.set_type( msgs::Shadows::TEXTURE_ADDITIVE);
  else if (type == "texture_modulative")
    result.set_type( msgs::Shadows::TEXTURE_MODULATIVE);

  result.mutable_color()->CopyFrom( 
      Convert(node->GetColor("color",Color(1,1,1,1))) );
      */

  return result;
}

msgs::Fog FogFromSDF( sdf::ElementPtr _sdf )
{
  msgs::Fog result;

  /*std::string type = node->GetString("type","linear",1);
  if (type == "linear")
    result.set_type(msgs::Fog::LINEAR);
  else if (type == "exp")
    result.set_type(msgs::Fog::EXPONENTIAL);
  else if (type == "exp2")
    result.set_type(msgs::Fog::EXPONENTIAL2);
  else
    gzerr << "Unknown fog type[" << type << "]\n";

  result.mutable_color()->CopyFrom( 
      Convert(node->GetColor("color",Color(1,1,1,1))) );
  result.set_density(node->GetFloat("density",1,1));
  result.set_start(node->GetFloat("start",0,1));
  result.set_end(node->GetFloat("end",1,1));
  */

  return result;
}

msgs::Scene SceneFromSDF(sdf::ElementPtr _sdf)
{
  msgs::Scene result;
/*
  Init(result,"scene");
  XMLConfigNode *cnode = NULL;

  if (node)
  {
    result.mutable_ambient()->CopyFrom( 
        Convert(node->GetColor("ambient",Color(1,1,1,1))) );

    result.mutable_background()->CopyFrom( 
        Convert(node->GetColor("background_color",Color(1,1,1,1))) );

    if (!node->GetString("sky_material","",0).empty())
      result.set_sky_material( node->GetString("sky_material","",1) );

    if ( (cnode = node->GetChild("fog")) != NULL)
      result.mutable_fog()->CopyFrom( FogFromXML(cnode) );

    if ( (cnode = node->GetChild("shadows")) != NULL && cnode->GetBool("enabled",true,0))
      result.mutable_shadows()->CopyFrom( ShadowsFromXML(cnode) );
  }
  else
    gzwarn << "node is null\n";
    */

  return result;
}

} }

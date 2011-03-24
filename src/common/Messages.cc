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

#include "common/GazeboError.hh"
#include "common/XMLConfig.hh"
#include "common/Messages.hh"

using namespace gazebo;
using namespace common;


const google::protobuf::FieldDescriptor *Message::GetFD(google::protobuf::Message &message, const std::string &name)
{
  return message.GetDescriptor()->FindFieldByName(name);
}

msgs::Header *Message::GetHeader(google::protobuf::Message &message)
{
  google::protobuf::Message *msg = NULL;
 
  if (Message::GetFD(message, "str_id"))
    msg = &message;
  else 
  {
    const google::protobuf::FieldDescriptor *fd;
    fd = Message::GetFD(message, "header");

    if (fd)
      msg = message.GetReflection()->MutableMessage(&message, fd);
  }

  return (msgs::Header*)msg;
}

void Message::Init(google::protobuf::Message &message, const std::string &id)
{
  msgs::Header *header = Message::GetHeader(message);

  if ( header )
  {
    header->set_str_id(id);
    Message::Stamp(*(header->mutable_stamp()));
  }
  else
    std::cout << "Header is non-existant\n";
}

void Message::Stamp(msgs::Header &hdr)
{
  Message::Stamp(*hdr.mutable_stamp());
}

void Message::Stamp(msgs::Time &time)
{
  Time tm = Time::GetWallTime();

  time.set_sec(tm.sec);
  time.set_nsec(tm.nsec);
}

std::string Message::Package(const std::string type, 
                      const google::protobuf::Message &message)
{
  std::string data;
  msgs::Packet pkg;
  Message::Stamp( *pkg.mutable_stamp() );
  pkg.set_type(type);

  std::string *serialized_data = pkg.mutable_serialized_data();
  if (!message.SerializeToString(serialized_data))
    gzthrow("Failed to serialized message");

  if (!pkg.SerializeToString(&data))
    gzthrow("Failed to serialized message");

  return data;
}

void Message::Set(msgs::Point *pt, const Vector3 &v)
{
  pt->set_x(v.x);
  pt->set_y(v.y);
  pt->set_z(v.z);
}

void Message::Set(msgs::Quaternion *q, const Quatern &v)
{
  q->set_x(v.x);
  q->set_y(v.y);
  q->set_z(v.z);
  q->set_w(v.w);
}

void Message::Set(msgs::Pose *p, const Pose3d &v)
{
  Message::Set( p->mutable_position(), v.pos );
  Message::Set( p->mutable_orientation(), v.rot );
}

void Message::Set(msgs::Color *c, const Color &v)
{
  c->set_r(v.R());
  c->set_g(v.G());
  c->set_b(v.B());
  c->set_a(v.A());
}

void Message::Set(msgs::Time *t, const Time &v)
{
  t->set_sec(v.sec);
  t->set_nsec(v.nsec);
}

void Message::Set(msgs::Plane *p, const Plane &v)
{
  Message::Set( p->mutable_normal(), v.normal );
  p->set_size_x( v.size.x );
  p->set_size_y( v.size.y );
  p->set_d( v.d );
}

msgs::Point Message::Convert(const Vector3 &v)
{
  msgs::Point result;
  result.set_x(v.x);
  result.set_y(v.y);
  result.set_z(v.z);
  return result;
}

msgs::Quaternion Message::Convert(const Quatern &q)
{
  msgs::Quaternion result;
  result.set_x(q.x);
  result.set_y(q.y);
  result.set_z(q.z);
  result.set_w(q.w);
  return result;
}

msgs::Pose Message::Convert(const Pose3d &p)
{
  msgs::Pose result;
  result.mutable_position()->CopyFrom( Convert(p.pos) );
  result.mutable_orientation()->CopyFrom( Convert(p.rot) );
  return result;
}

msgs::Color Message::Convert(const Color &c)
{
  msgs::Color result;
  result.set_r(c.R());
  result.set_g(c.G());
  result.set_b(c.B());
  result.set_a(c.A());
  return result;
}

msgs::Time Message::Convert(const Time &t)
{
  msgs::Time result;
  result.set_sec(t.sec);
  result.set_nsec(t.nsec);
  return result;
}

msgs::Plane Message::Convert(const Plane &p)
{
  msgs::Plane result;
  result.mutable_normal()->CopyFrom( Convert(p.normal) );
  result.set_size_x( p.size.x );
  result.set_size_y( p.size.y );
  result.set_d( p.d );
  return result;
}

Vector3 Message::Convert(const msgs::Point &v)
{
  return Vector3(v.x(), v.y(), v.z());
}

Quatern Message::Convert(const msgs::Quaternion &q)
{
  return Quatern(q.w(), q.x(), q.y(), q.z());
}

Pose3d Message::Convert(const msgs::Pose &p)
{
  return Pose3d( Message::Convert(p.position()), 
                 Message::Convert(p.orientation()) );
}

Color Message::Convert(const msgs::Color &c)
{
  return Color( c.r(), c.g(), c.b(), c.a() );
}

Time Message::Convert(const msgs::Time &t)
{
  return Time(t.sec(), t.nsec());
}

Plane Message::Convert(const msgs::Plane &p)
{
  return Plane(Message::Convert(p.normal()), 
               Vector2d(p.size_x(), p.size_y()),
               p.d() );
}





msgs::Light Message::LightFromXML(XMLConfigNode *node)
{
  msgs::Light result;

  std::string type = node->GetString("type","point",1);
  std::transform( type.begin(), type.end(), type.begin(), ::tolower);

  if (type == "point")
    result.set_type(msgs::Light::POINT);
  else if (type == "spot")
    result.set_type(msgs::Light::SPOT);
  else if (type == "directional")
    result.set_type(msgs::Light::DIRECTIONAL);

  result.mutable_pose()->mutable_position()->CopyFrom( Convert(node->GetVector3("xyz",Vector3(0,0,0))) );  
  result.mutable_pose()->mutable_orientation()->CopyFrom( Convert(node->GetRotation("rpy", Quatern() )) );

  result.mutable_diffuse()->CopyFrom( Convert( node->GetColor("diffuse", Color(1,1,1,1)) ) );
  result.mutable_specular()->CopyFrom( Convert( node->GetColor("specular", Color(0,0,0,1)) ) );
  result.mutable_attenuation()->CopyFrom( Convert( 
        node->GetVector3("attenuation",Vector3(.2, 0.1, 0.0)) ) );
  result.mutable_direction()->CopyFrom( Convert( node->GetVector3("direction",Vector3(0, 0, -1)) ) );
  result.set_range( node->GetDouble("range",20,1) );
  result.set_cast_shadows( node->GetBool("cast_shadows",false,0) );

  if (node->GetChild("spot_inner_angle"))
    result.set_spot_inner_angle( node->GetFloat("spot_inner_angle",0,0) );
  if (node->GetChild("spot_outer_angle"))
    result.set_spot_outer_angle( node->GetFloat("spot_outer_angle",0,0) );
  if (node->GetChild("spot_falloff"))
    result.set_spot_falloff( node->GetFloat("spot_falloff",0,0) );

  return result;
}

msgs::Visual Message::VisualFromXML(XMLConfigNode *node)
{
  XMLConfigNode *cnode = NULL;
  msgs::Visual result;

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
          Convert( ccnode->GetVector3("scale", Vector3(1,1,1)) ) );
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
          Convert( ccnode->GetVector3("size", Vector3(1,1,1)) ) );
    }

    ccnode = cnode->GetChild("plane");
    if ( ccnode )
    {
      result.mutable_plane()->mutable_normal()->CopyFrom( 
          Message::Convert( ccnode->GetVector3("normal",Vector3(0,0,1))) );
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
        Convert(node->GetChild("origin")->GetVector3("xyz",Vector3(0,0,0))));

    result.mutable_pose()->mutable_orientation()->CopyFrom( 
        Convert(node->GetChild("origin")->GetRotation("rpy",Quatern(1,0,0,0))));
  }

  return result;
}

msgs::Shadows Message::ShadowsFromXML(XMLConfigNode *node)
{
  msgs::Shadows result;

  std::string type = node->GetString("type","stencil_modulative",0);
  if (type == "stencil_modulative")
    result.set_type( msgs::Shadows::STENCIL_MODULATIVE);
  else if (type == "stencil_additive")
    result.set_type( msgs::Shadows::STENCIL_ADDITIVE);
  else if (type == "texture_additive")
    result.set_type( msgs::Shadows::TEXTURE_ADDITIVE);
  else if (type == "texture_modulative")
    result.set_type( msgs::Shadows::TEXTURE_MODULATIVE);

  result.mutable_color()->CopyFrom( 
      Message::Convert(node->GetColor("color",Color(1,1,1,1))) );

  return result;
}

msgs::Fog Message::FogFromXML(XMLConfigNode *node)
{
  msgs::Fog result;

  std::string type = node->GetString("type","linear",1);
  if (type == "linear")
    result.set_type(msgs::Fog::LINEAR);
  else if (type == "exp")
    result.set_type(msgs::Fog::EXPONENTIAL);
  else if (type == "exp2")
    result.set_type(msgs::Fog::EXPONENTIAL2);
  else
    std::cerr << "Unknown fog type[" << type << "]\n";

  result.mutable_color()->CopyFrom( 
      Message::Convert(node->GetColor("color",Color(1,1,1,1))) );
  result.set_density(node->GetFloat("density",1,1));
  result.set_start(node->GetFloat("start",0,1));
  result.set_end(node->GetFloat("end",1,1));

  return result;
}

msgs::Scene Message::SceneFromXML(XMLConfigNode *node)
{
  msgs::Scene result;
  Message::Init(result,"scene");
  XMLConfigNode *cnode = NULL;

  result.mutable_ambient()->CopyFrom( 
      Message::Convert(node->GetColor("ambient",Color(1,1,1,1))) );

  result.mutable_background()->CopyFrom( 
      Message::Convert(node->GetColor("background_color",Color(1,1,1,1))) );

  if (!node->GetString("sky_material","",0).empty())
    result.set_sky_material( node->GetString("sky_material","",1) );

  if ( (cnode = node->GetChild("fog")) != NULL)
    result.mutable_fog()->CopyFrom( Message::FogFromXML(cnode) );

  if ( (cnode = node->GetChild("shadows")) != NULL && cnode->GetBool("enabled",true,0))
    result.mutable_shadows()->CopyFrom( Message::ShadowsFromXML(cnode) );

  return result;
}

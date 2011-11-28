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
#include "math/Rand.hh"

#include "common/Exception.hh"
#include "common/Console.hh"
#include "msgs/msgs.h"

namespace gazebo { namespace msgs {

/// Create a request message
msgs::Request *CreateRequest( const std::string &_request, 
                              const std::string &_data )
{
  msgs::Request *request = new msgs::Request;

  request->set_request( _request );
  request->set_data( _data );
  request->set_id( math::Rand::GetIntUniform(1, 10000) );

  return request;
}

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

void Set(msgs::Vector3d *pt, const math::Vector3 &v)
{
  pt->set_x(v.x);
  pt->set_y(v.y);
  pt->set_z(v.z);
}

void Set(msgs::Vector2d *_pt, const math::Vector2d &_v)
{
  _pt->set_x(_v.x);
  _pt->set_y(_v.y);
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


void Set(msgs::PlaneGeom *p, const math::Plane &v)
{
  Set( p->mutable_normal(), v.normal );
  p->mutable_size()->set_x( v.size.x );
  p->mutable_size()->set_y( v.size.y );
  p->set_d( v.d );
}

msgs::Vector3d Convert(const math::Vector3 &v)
{
  msgs::Vector3d result;
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

msgs::PlaneGeom Convert(const math::Plane &p)
{
  msgs::PlaneGeom result;
  result.mutable_normal()->CopyFrom( Convert(p.normal) );
  result.mutable_size()->set_x( p.size.x );
  result.mutable_size()->set_y( p.size.y );
  result.set_d( p.d );
  return result;
}

math::Vector3 Convert(const msgs::Vector3d &v)
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

math::Plane Convert(const msgs::PlaneGeom &p)
{
  return math::Plane(Convert(p.normal()), 
               math::Vector2d(p.size().x(), p.size().y()),
               p.d() );
}

msgs::GUI GUIFromSDF(sdf::ElementPtr _sdf)
{
  msgs::GUI result;

  result.set_fullscreen( _sdf->GetValueBool("fullscreen") );


  if (_sdf->HasElement("camera"))
  {
    sdf::ElementPtr camSDF = _sdf->GetElement("camera");
    msgs::GUICamera *guiCam = result.mutable_camera();

    guiCam->set_name( camSDF->GetValueString("name") );

    if (_sdf->HasElement("origin"))
    {
      msgs::Set( guiCam->mutable_origin(), 
          camSDF->GetElement("origin")->GetValuePose("pose") );
    }

    if (camSDF->HasElement("view_controller"))
    {
        guiCam->set_view_controller( 
            camSDF->GetElement("view_controller")->GetValueString("type") );
    }

    if (camSDF->HasElement("track_visual"))
    {
      guiCam->mutable_track()->CopyFrom( 
          TrackVisualFromSDF( camSDF->GetElement("track_visual") ) );
    }
  }

  return result;
}

msgs::TrackVisual TrackVisualFromSDF( sdf::ElementPtr _sdf )
{
  msgs::TrackVisual result;

  result.set_name( _sdf->GetValueString("name") );

  if (_sdf->HasElement("min_dist"))
   result.set_min_dist( _sdf->GetElement("min_dist")->GetValueDouble() ); 

  if (_sdf->HasElement("max_dist"))
   result.set_max_dist( _sdf->GetElement("max_dist")->GetValueDouble() ); 

  return result;
}


msgs::Light LightFromSDF( sdf::ElementPtr _sdf )
{
  msgs::Light result;

  std::string type = _sdf->GetValueString("type");
  std::transform( type.begin(), type.end(), type.begin(), ::tolower);

  result.set_name( _sdf->GetValueString("name") );

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

  result.set_name( _sdf->GetValueString("name") );
  result.set_cast_shadows( _sdf->GetValueBool("cast_shadows") );
  result.set_transparency( _sdf->GetValueDouble("transparency") );

  // Load the geometry
  if (_sdf->HasElement("geometry"))
  {
    sdf::ElementPtr geomElem = _sdf->GetElement("geometry")->GetFirstElement();
    math::Vector3 scale(1,1,1);
    msgs::Geometry *geomMsg = result.mutable_geometry();

    if (geomElem->GetName() == "box")
    {
      geomMsg->set_type( msgs::Geometry::BOX );
      msgs::Set( geomMsg->mutable_box()->mutable_size(), 
                 geomElem->GetValueVector3("size") );
    }
    else if (geomElem->GetName() == "cylinder")
    {
      geomMsg->set_type( msgs::Geometry::CYLINDER );
      geomMsg->mutable_cylinder()->set_radius( 
          geomElem->GetValueDouble("radius") );
      geomMsg->mutable_cylinder()->set_length( 
          geomElem->GetValueDouble("length") );
    }
    else if (geomElem->GetName() == "sphere")
    {
      geomMsg->set_type( msgs::Geometry::SPHERE );
      geomMsg->mutable_sphere()->set_radius( 
          geomElem->GetValueDouble("radius") );
    }
    else if (geomElem->GetName() == "plane")
    {
      geomMsg->set_type( msgs::Geometry::PLANE );
      msgs::Set( geomMsg->mutable_plane()->mutable_normal(),
                 geomElem->GetValueVector3("normal") );
    }
    else if (geomElem->GetName() == "image")
    {
      geomMsg->set_type( msgs::Geometry::IMAGE );
      geomMsg->mutable_image()->set_scale( 
          geomElem->GetValueDouble("scale") );
      geomMsg->mutable_image()->set_height( 
          geomElem->GetValueDouble("height") );
      geomMsg->mutable_image()->set_filename( 
          geomElem->GetValueString("filename") ); 
    }
    else if (geomElem->GetName() == "heightmap")
    {
      geomMsg->set_type( msgs::Geometry::HEIGHTMAP );
      msgs::Set( geomMsg->mutable_heightmap()->mutable_size(), 
                 geomElem->GetValueVector3("size") );
      geomMsg->mutable_heightmap()->set_filename( 
          geomElem->GetValueString("filename") ); 
    }
    else if (geomElem->GetName() == "mesh")
    {
      geomMsg->set_type( msgs::Geometry::MESH );
      msgs::Set( geomMsg->mutable_mesh()->mutable_scale(), 
                 geomElem->GetValueVector3("scale") );
      geomMsg->mutable_mesh()->set_filename( 
          geomElem->GetValueString("filename") ); 
    }
    else
      gzerr << "Unknown geometry type\n";
  }

  /// Load the material
  if (_sdf->HasElement("material"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("material");
    msgs::Material *matMsg = result.mutable_material();
    matMsg->set_script(elem->GetValueString("script"));

    if (elem->HasElement("shader"))
    {
      sdf::ElementPtr shaderElem = elem->GetElement("shader");

      if (shaderElem->GetValueString("type") == "pixel")
        matMsg->set_shader_type( msgs::Material::PIXEL );
      else if (shaderElem->GetValueString("type") == "vertex")
        matMsg->set_shader_type( msgs::Material::VERTEX );
      else if (shaderElem->GetValueString("type") == "normal_map_object_space")
        matMsg->set_shader_type( msgs::Material::NORMAL_MAP_OBJECT_SPACE );
      else if (shaderElem->GetValueString("type") == "normal_map_tangent_space")
        matMsg->set_shader_type( msgs::Material::NORMAL_MAP_TANGENT_SPACE );
      else
        gzerr << "Unknown shader type[" << shaderElem->GetValueString("type") << "]\n";

     if (shaderElem->HasElement("normal_map"))
          matMsg->set_normal_map( 
            shaderElem->GetElement("normal_map")->GetValueString());
    }

    if (elem->HasElement("ambient"))
      msgs::Set( matMsg->mutable_ambient(),
                 elem->GetElement("ambient")->GetValueColor("rgba") );
    if (elem->HasElement("diffuse"))
      msgs::Set( matMsg->mutable_diffuse(),
                 elem->GetElement("diffuse")->GetValueColor("rgba") );
    if (elem->HasElement("specular"))
      msgs::Set( matMsg->mutable_specular(),
                 elem->GetElement("specular")->GetValueColor("rgba"));
    if (elem->HasElement("emissive"))
      msgs::Set( matMsg->mutable_emissive(),
                 elem->GetElement("emissive")->GetValueColor("rgba") );
  }

  // Set the origin of the visual
  if (_sdf->HasElement("origin"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("origin");
    msgs::Set( result.mutable_pose(), elem->GetValuePose("pose") );
  }

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

  if (_sdf->HasElement("grid"))
    result.set_grid( _sdf->GetElement("grid")->GetValueBool("enabled") );
  else
    result.set_grid( true );

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
    result.set_shadows(_sdf->GetElement("shadows")->GetValueBool("enabled"));

  return result;
}


} }

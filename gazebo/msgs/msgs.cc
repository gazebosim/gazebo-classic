/*
 * Copyright (C) 2012-2013 Open Source Robotics Foundation
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

#include "gazebo/math/Vector3.hh"
#include "gazebo/math/Pose.hh"
#include "gazebo/math/Quaternion.hh"
#include "gazebo/math/Plane.hh"
#include "gazebo/math/Rand.hh"

#include "gazebo/common/CommonIface.hh"
#include "gazebo/common/Image.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/msgs/msgs.hh"

namespace gazebo
{
  namespace msgs
  {
    void FillProperties(const sdf::Property &_sdfProp,
                        msgs::Plugin::Property *_msgProp);

    /////////////////////////////////////////////
    void FillProperties(const sdf::Property &_sdfProp,
                        msgs::Plugin::Property *_msgProp)
    {
      _msgProp->set_key(_sdfProp.key);
      _msgProp->set_value(_sdfProp.value);

      for (sdf::PropertyList::const_iterator iter = _sdfProp.properties.begin();
           iter != _sdfProp.properties.end(); ++iter)
      {
        FillProperties(*iter, _msgProp->add_properties());
      }
    }

    /// Create a request message
    msgs::Request *CreateRequest(const std::string &_request,
        const std::string &_data)
    {
      msgs::Request *request = new msgs::Request;

      request->set_request(_request);
      request->set_data(_data);
      request->set_id(math::Rand::GetIntUniform(1, 10000));

      return request;
    }

    const google::protobuf::FieldDescriptor *GetFD(
        google::protobuf::Message &message, const std::string &name)
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

    void Init(google::protobuf::Message &_message, const std::string &_id)
    {
      msgs::Header *header = GetHeader(_message);

      if (header)
      {
        if (!_id.empty())
          header->set_str_id(_id);
        Stamp(header->mutable_stamp());
      }
    }

    void Stamp(msgs::Header *_hdr)
    {
      Stamp(_hdr->mutable_stamp());
    }

    void Stamp(robot_msgs::Time *_time)
    {
      common::Time tm = common::Time::GetWallTime();

      _time->set_sec(tm.sec);
      _time->set_nsec(tm.nsec);
    }

    std::string Package(const std::string &type,
        const google::protobuf::Message &message)
    {
      std::string data;
      robot_msgs::Packet pkg;

      Stamp(pkg.mutable_stamp());
      pkg.set_msg_type(type);
      pkg.set_topic("");

      std::string *serialized_data = pkg.mutable_msg_data();
      if (!message.IsInitialized())
        gzthrow("Can't serialize message of type[" + message.GetTypeName() +
            "] because it is missing required fields");

      if (!message.SerializeToString(serialized_data))
        gzthrow("Failed to serialized message");

      if (!pkg.SerializeToString(&data))
        gzthrow("Failed to serialized message");

      return data;
    }

    /////////////////////////////////////////////
    void Set(robot_msgs::Vector3d *_pt, const math::Vector3 &_v)
    {
      _pt->set_x(_v.x);
      _pt->set_y(_v.y);
      _pt->set_z(_v.z);
    }

    /////////////////////////////////////////////
    void Set(robot_msgs::Vector3d *_pt, const sdf::Vector3 &_v)
    {
      _pt->set_x(_v.x);
      _pt->set_y(_v.y);
      _pt->set_z(_v.z);
    }

    /////////////////////////////////////////////
    void Set(robot_msgs::Vector2d *_pt, const math::Vector2d &_v)
    {
      _pt->set_x(_v.x);
      _pt->set_y(_v.y);
    }

    /////////////////////////////////////////////
    void Set(robot_msgs::Vector2d *_pt, const sdf::Vector2d &_v)
    {
      _pt->set_x(_v.x);
      _pt->set_y(_v.y);
    }

    /////////////////////////////////////////////
    void Set(robot_msgs::Quaternion *_q, const math::Quaternion &_v)
    {
      _q->set_x(_v.x);
      _q->set_y(_v.y);
      _q->set_z(_v.z);
      _q->set_w(_v.w);
    }

    /////////////////////////////////////////////
    void Set(robot_msgs::Quaternion *_q, const sdf::Quaternion &_v)
    {
      _q->set_x(_v.x);
      _q->set_y(_v.y);
      _q->set_z(_v.z);
      _q->set_w(_v.w);
    }

    /////////////////////////////////////////////
    void Set(robot_msgs::Pose *_p, const math::Pose &_v)
    {
      Set(_p->mutable_position(), _v.pos);
      Set(_p->mutable_orientation(), _v.rot);
    }

    /////////////////////////////////////////////
    void Set(robot_msgs::Pose *_p, const sdf::Pose &_v)
    {
      Set(_p->mutable_position(), _v.pos);
      Set(_p->mutable_orientation(), _v.rot);
    }

    void Set(robot_msgs::Color *_c, const common::Color &_v)
    {
      _c->set_r(_v.r);
      _c->set_g(_v.g);
      _c->set_b(_v.b);
      _c->set_a(_v.a);
    }

    void Set(robot_msgs::Color *_c, const sdf::Color &_v)
    {
      _c->set_r(_v.r);
      _c->set_g(_v.g);
      _c->set_b(_v.b);
      _c->set_a(_v.a);
    }

    void Set(robot_msgs::Time *_t, const common::Time &_v)
    {
      _t->set_sec(_v.sec);
      _t->set_nsec(_v.nsec);
    }


    /////////////////////////////////////////////////
    void Set(msgs::PlaneGeom *_p, const math::Plane &_v)
    {
      Set(_p->mutable_normal(), _v.normal);
      _p->mutable_size()->set_x(_v.size.x);
      _p->mutable_size()->set_y(_v.size.y);
      _p->set_d(_v.d);
    }

    /////////////////////////////////////////////////
    robot_msgs::Image::Format Convert(common::Image::PixelFormat _fmt)
    {
      robot_msgs::Image::Format fmt = robot_msgs::Image::UNKNOWN_PIXEL_FORMAT;
      switch(_fmt)
      {
        case common::Image::L_INT8:
          fmt = robot_msgs::Image::L_INT8;
          break;
        case common::Image::L_INT16:
          fmt = robot_msgs::Image::L_INT16;
          break;
        case common::Image::RGB_INT8:
          fmt = robot_msgs::Image::RGB_INT8;
          break;
        case common::Image::RGBA_INT8:
          fmt = robot_msgs::Image::RGBA_INT8;
          break;
        case common::Image::BGRA_INT8:
          fmt = robot_msgs::Image::BGRA_INT8;
          break;
        case common::Image::RGB_INT16:
          fmt = robot_msgs::Image::RGB_INT16;
          break;
        case common::Image::RGB_INT32:
          fmt = robot_msgs::Image::RGB_INT32;
          break;
        case common::Image::BGR_INT8:
          fmt = robot_msgs::Image::BGR_INT8;
          break;
        case common::Image::BGR_INT16:
          fmt = robot_msgs::Image::BGR_INT16;
          break;
        case common::Image::BGR_INT32:
          fmt = robot_msgs::Image::BGR_INT32;
          break;
        case common::Image::R_FLOAT16:
          fmt = robot_msgs::Image::R_FLOAT16;
          break;
        case common::Image::RGB_FLOAT16:
          fmt = robot_msgs::Image::RGB_FLOAT16;
          break;
        case common::Image::R_FLOAT32:
          fmt = robot_msgs::Image::R_FLOAT32;
          break;
        case common::Image::RGB_FLOAT32:
          fmt = robot_msgs::Image::RGB_FLOAT32;
          break;
        case common::Image::BAYER_RGGB8:
          fmt = robot_msgs::Image::BAYER_RGGB8;
          break;
        case common::Image::BAYER_RGGR8:
          fmt = robot_msgs::Image::BAYER_RGGR8;
          break;
        case common::Image::BAYER_GBRG8:
          fmt = robot_msgs::Image::BAYER_GBRG8;
          break;
        case common::Image::BAYER_GRBG8:
          fmt = robot_msgs::Image::BAYER_GRBG8;
          break;
        default:
          gzerr << "Unable to handle format[" << _fmt << "]\n";
          break;
      };

      return fmt;
    }

    /////////////////////////////////////////////////
    common::Image::PixelFormat Convert(robot_msgs::Image::Format _fmt)
    {
      common::Image::PixelFormat fmt = common::Image::UNKNOWN_PIXEL_FORMAT;

      switch(_fmt)
      {
        case robot_msgs::Image::L_INT8:
          fmt = common::Image::L_INT8;
          break;
        case robot_msgs::Image::L_INT16:
          fmt = common::Image::L_INT16;
          break;
        case robot_msgs::Image::RGB_INT8:
          fmt = common::Image::RGB_INT8;
          break;
        case robot_msgs::Image::RGBA_INT8:
          fmt = common::Image::RGBA_INT8;
          break;
        case robot_msgs::Image::BGRA_INT8:
          fmt = common::Image::BGRA_INT8;
          break;
        case robot_msgs::Image::RGB_INT16:
          fmt = common::Image::RGB_INT16;
          break;
        case robot_msgs::Image::RGB_INT32:
          fmt = common::Image::RGB_INT32;
          break;
        case robot_msgs::Image::BGR_INT8:
          fmt = common::Image::BGR_INT8;
          break;
        case robot_msgs::Image::BGR_INT16:
          fmt = common::Image::BGR_INT16;
          break;
        case robot_msgs::Image::BGR_INT32:
          fmt = common::Image::BGR_INT32;
          break;
        case robot_msgs::Image::R_FLOAT16:
          fmt = common::Image::R_FLOAT16;
          break;
        case robot_msgs::Image::RGB_FLOAT16:
          fmt = common::Image::RGB_FLOAT16;
          break;
        case robot_msgs::Image::R_FLOAT32:
          fmt = common::Image::R_FLOAT32;
          break;
        case robot_msgs::Image::RGB_FLOAT32:
          fmt = common::Image::RGB_FLOAT32;
          break;
        case robot_msgs::Image::BAYER_RGGB8:
          fmt = common::Image::BAYER_RGGB8;
          break;
        case robot_msgs::Image::BAYER_RGGR8:
          fmt = common::Image::BAYER_RGGR8;
          break;
        case robot_msgs::Image::BAYER_GBRG8:
          fmt = common::Image::BAYER_GBRG8;
          break;
        case robot_msgs::Image::BAYER_GRBG8:
          fmt = common::Image::BAYER_GRBG8;
          break;
        case robot_msgs::Image::JPEG:
          gzerr << "Unable to handle JPEG format\n";
        default:
          gzerr << "Unable to handle format[" << _fmt << "]\n";
      };

      return fmt;
    }

    /////////////////////////////////////////////////
    void Set(common::Image &_img, const robot_msgs::Image &_msg)
    {
      _img.SetFromData(
          (const unsigned char*)_msg.data().data(),
          _msg.width(), _msg.height(), Convert(_msg.format()));
    }

    /////////////////////////////////////////////////
    void Set(robot_msgs::Image *_msg, const common::Image &_i)
    {
      _msg->set_width(_i.GetWidth());
      _msg->set_height(_i.GetHeight());
      _msg->set_format(Convert(_i.GetPixelFormat()));
      _msg->set_step(_i.GetPitch());

      unsigned char *data = NULL;
      unsigned int size;
      _i.GetData(&data, size);
      _msg->set_data(data, size);
    }

    /////////////////////////////////////////////////
    robot_msgs::Vector3d Convert(const math::Vector3 &_v)
    {
      robot_msgs::Vector3d result;
      result.set_x(_v.x);
      result.set_y(_v.y);
      result.set_z(_v.z);
      return result;
    }

    /////////////////////////////////////////////////
    robot_msgs::Vector3d Convert(const sdf::Vector3 &_v)
    {
      robot_msgs::Vector3d result;
      result.set_x(_v.x);
      result.set_y(_v.y);
      result.set_z(_v.z);
      return result;
    }

    /////////////////////////////////////////////////
    robot_msgs::Quaternion Convert(const math::Quaternion &_q)
    {
      robot_msgs::Quaternion result;
      result.set_x(_q.x);
      result.set_y(_q.y);
      result.set_z(_q.z);
      result.set_w(_q.w);
      return result;
    }

    /////////////////////////////////////////////////
    robot_msgs::Quaternion Convert(const sdf::Quaternion &_q)
    {
      robot_msgs::Quaternion result;
      result.set_x(_q.x);
      result.set_y(_q.y);
      result.set_z(_q.z);
      result.set_w(_q.w);
      return result;
    }

    /////////////////////////////////////////////////
    robot_msgs::Pose Convert(const math::Pose &_p)
    {
      robot_msgs::Pose result;
      result.mutable_position()->CopyFrom(Convert(_p.pos));
      result.mutable_orientation()->CopyFrom(Convert(_p.rot));
      return result;
    }

    /////////////////////////////////////////////////
    robot_msgs::Pose Convert(const sdf::Pose &_p)
    {
      robot_msgs::Pose result;
      result.mutable_position()->CopyFrom(Convert(_p.pos));
      result.mutable_orientation()->CopyFrom(Convert(_p.rot));
      return result;
    }

    robot_msgs::Color Convert(const common::Color &_c)
    {
      robot_msgs::Color result;
      result.set_r(_c.r);
      result.set_g(_c.g);
      result.set_b(_c.b);
      result.set_a(_c.a);
      return result;
    }

    robot_msgs::Color Convert(const sdf::Color &_c)
    {
      robot_msgs::Color result;
      result.set_r(_c.r);
      result.set_g(_c.g);
      result.set_b(_c.b);
      result.set_a(_c.a);
      return result;
    }

    robot_msgs::Time Convert(const common::Time &_t)
    {
      robot_msgs::Time result;
      result.set_sec(_t.sec);
      result.set_nsec(_t.nsec);
      return result;
    }

    msgs::PlaneGeom Convert(const math::Plane &_p)
    {
      msgs::PlaneGeom result;
      result.mutable_normal()->CopyFrom(Convert(_p.normal));
      result.mutable_size()->set_x(_p.size.x);
      result.mutable_size()->set_y(_p.size.y);
      result.set_d(_p.d);
      return result;
    }

    math::Vector3 Convert(const robot_msgs::Vector3d &_v)
    {
      return math::Vector3(_v.x(), _v.y(), _v.z());
    }

    math::Quaternion Convert(const robot_msgs::Quaternion &_q)
    {
      return math::Quaternion(_q.w(), _q.x(), _q.y(), _q.z());
    }

    math::Pose Convert(const robot_msgs::Pose &_p)
    {
      return math::Pose(Convert(_p.position()),
          Convert(_p.orientation()));
    }

    common::Color Convert(const robot_msgs::Color &_c)
    {
      return common::Color(_c.r(), _c.g(), _c.b(), _c.a());
    }

    common::Time Convert(const robot_msgs::Time &_t)
    {
      return common::Time(_t.sec(), _t.nsec());
    }

    math::Plane Convert(const msgs::PlaneGeom &_p)
    {
      return math::Plane(Convert(_p.normal()),
          math::Vector2d(_p.size().x(), _p.size().y()),
          _p.d());
    }

    /////////////////////////////////////////////
    msgs::GUI GUIFromRML(const rml::Gui &_rml)
    {
      msgs::GUI result;

      result.set_fullscreen(_rml.fullscreen());

      if (_rml.has_camera())
      {
        msgs::GUICamera *guiCam = result.mutable_camera();

        guiCam->set_name(_rml.camera().name());

        if (_rml.camera().has_pose())
          msgs::Set(guiCam->mutable_pose(), _rml.camera().pose());

        if (_rml.camera().has_view_controller())
          guiCam->set_view_controller(_rml.camera().view_controller());

        if (_rml.camera().has_track_visual())
        {
          guiCam->mutable_track()->CopyFrom(
              TrackVisualFromRML(_rml.camera().track_visual()));
        }
      }

      return result;
    }

    msgs::GUI GUIFromSDF(sdf::ElementPtr _sdf)
    {
      msgs::GUI result;

      result.set_fullscreen(_sdf->Get<bool>("fullscreen"));


      if (_sdf->HasElement("camera"))
      {
        sdf::ElementPtr camSDF = _sdf->GetElement("camera");
        msgs::GUICamera *guiCam = result.mutable_camera();

        guiCam->set_name(camSDF->Get<std::string>("name"));

        if (camSDF->HasElement("pose"))
        {
          msgs::Set(guiCam->mutable_pose(), camSDF->Get<math::Pose>("pose"));
        }

        if (camSDF->HasElement("view_controller"))
        {
          guiCam->set_view_controller(
              camSDF->Get<std::string>("view_controller"));
        }

        if (camSDF->HasElement("track_visual"))
        {
          guiCam->mutable_track()->CopyFrom(
              TrackVisualFromSDF(camSDF->GetElement("track_visual")));
        }
      }

      return result;
    }

    /////////////////////////////////////////////////
    msgs::TrackVisual TrackVisualFromSDF(sdf::ElementPtr _sdf)
    {
      msgs::TrackVisual result;

      result.set_name(_sdf->Get<std::string>("name"));

      if (_sdf->HasElement("min_dist"))
        result.set_min_dist(_sdf->GetElement("min_dist")->Get<double>());

      if (_sdf->HasElement("max_dist"))
        result.set_max_dist(_sdf->GetElement("max_dist")->Get<double>());

      return result;
    }

    /////////////////////////////////////////////////
    msgs::TrackVisual TrackVisualFromRML(
        const rml::Gui::Camera::Track_Visual &_rml)
    {
      msgs::TrackVisual result;

      result.set_name(_rml.name());

      if (_rml.has_min_dist())
        result.set_min_dist(_rml.min_dist());

      if (_rml.has_max_dist())
        result.set_max_dist(_rml.max_dist());

      return result;
    }

    /////////////////////////////////////////////////
    msgs::Light LightFromRML(const rml::Light &_rml)
    {
      msgs::Light result;

      std::string type = _rml.type();
      std::transform(type.begin(), type.end(), type.begin(), ::tolower);

      result.set_name(_rml.name());

      result.set_cast_shadows(_rml.cast_shadows());

      if (type == "point")
        result.set_type(msgs::Light::POINT);
      else if (type == "spot")
        result.set_type(msgs::Light::SPOT);
      else if (type == "directional")
        result.set_type(msgs::Light::DIRECTIONAL);

      if (_rml.has_pose())
        result.mutable_pose()->CopyFrom(Convert(_rml.pose()));

      if (_rml.has_diffuse())
        result.mutable_diffuse()->CopyFrom(Convert(_rml.diffuse()));

      if (_rml.has_specular())
        result.mutable_specular()->CopyFrom(Convert(_rml.specular()));

      if (_rml.has_attenuation())
      {
        result.set_attenuation_constant(_rml.attenuation().constant());
        result.set_attenuation_linear(_rml.attenuation().linear());
        result.set_attenuation_quadratic(_rml.attenuation().quadratic());
        result.set_range(_rml.attenuation().range());
      }

      if (_rml.has_direction())
        result.mutable_direction()->CopyFrom(Convert(_rml.direction()));

      if (_rml.has_spot())
      {
        result.set_spot_inner_angle(_rml.spot().inner_angle());
        result.set_spot_outer_angle(_rml.spot().outer_angle());
        result.set_spot_falloff(_rml.spot().falloff());
      }

      return result;
    }

    /////////////////////////////////////////////////
    msgs::Light LightFromSDF(sdf::ElementPtr _sdf)
    {
      msgs::Light result;

      std::string type = _sdf->Get<std::string>("type");
      std::transform(type.begin(), type.end(), type.begin(), ::tolower);

      result.set_name(_sdf->Get<std::string>("name"));

      result.set_cast_shadows(_sdf->Get<bool>("cast_shadows"));

      if (type == "point")
        result.set_type(msgs::Light::POINT);
      else if (type == "spot")
        result.set_type(msgs::Light::SPOT);
      else if (type == "directional")
        result.set_type(msgs::Light::DIRECTIONAL);

      if (_sdf->HasElement("pose"))
      {
        result.mutable_pose()->CopyFrom(Convert(_sdf->Get<math::Pose>("pose")));
      }

      if (_sdf->HasElement("diffuse"))
      {
        result.mutable_diffuse()->CopyFrom(
            Convert(_sdf->Get<common::Color>("diffuse")));
      }

      if (_sdf->HasElement("specular"))
      {
        result.mutable_specular()->CopyFrom(
            Convert(_sdf->Get<common::Color>("specular")));
      }

      if (_sdf->HasElement("attenuation"))
      {
        sdf::ElementPtr elem = _sdf->GetElement("attenuation");
        result.set_attenuation_constant(elem->Get<double>("constant"));
        result.set_attenuation_linear(elem->Get<double>("linear"));
        result.set_attenuation_quadratic(elem->Get<double>("quadratic"));
        result.set_range(elem->Get<double>("range"));
      }

      if (_sdf->HasElement("direction"))
      {
        result.mutable_direction()->CopyFrom(
            Convert(_sdf->Get<math::Vector3>("direction")));
      }

      if (_sdf->HasElement("spot"))
      {
        sdf::ElementPtr elem = _sdf->GetElement("spot");
        result.set_spot_inner_angle(elem->Get<double>("inner_angle"));
        result.set_spot_outer_angle(elem->Get<double>("outer_angle"));
        result.set_spot_falloff(elem->Get<double>("falloff"));
      }

      return result;
    }

    /////////////////////////////////////////////////
    msgs::MeshGeom MeshFromSDF(sdf::ElementPtr _sdf)
    {
      rml::Mesh rmlMesh;
      rmlMesh.SetFromXML(_sdf);
      return MeshFromRML(rmlMesh);
    }

    /////////////////////////////////////////////////
    msgs::MeshGeom MeshFromRML(const rml::Mesh &_rml)
    {
      msgs::MeshGeom result;

      msgs::Set(result.mutable_scale(), _rml.scale());

      result.set_filename(_rml.uri());

      if (_rml.has_submesh())
      {
        if (_rml.submesh().has_name() && _rml.submesh().name() != "__default__")
        {
          result.set_submesh(_rml.submesh().name());

          if (_rml.submesh().has_center())
            result.set_center_submesh(_rml.submesh().center());
        }
      }

      return result;
    }

    /////////////////////////////////////////////////
    msgs::Geometry GeometryFromSDF(sdf::ElementPtr _sdf)
    {
      rml::Geometry geomRML;
      geomRML.SetFromXML(_sdf);
      return GeometryFromRML(geomRML);
    }

    /////////////////////////////////////////////////
    msgs::Geometry GeometryFromRML(const rml::Geometry &_rml)
    {
      msgs::Geometry result;

      if (_rml.has_box_shape())
      {
        result.set_type(msgs::Geometry::BOX);
        msgs::Set(result.mutable_box()->mutable_size(),
            _rml.box_shape().size());
      }
      else if (_rml.has_cylinder_shape())
      {
        result.set_type(msgs::Geometry::CYLINDER);
        result.mutable_cylinder()->set_radius(_rml.cylinder_shape().radius());
        result.mutable_cylinder()->set_length(_rml.cylinder_shape().length());
      }
      else if (_rml.has_sphere_shape())
      {
        result.set_type(msgs::Geometry::SPHERE);
        result.mutable_sphere()->set_radius(_rml.sphere_shape().radius());
      }
      else if (_rml.has_plane_shape())
      {
        result.set_type(msgs::Geometry::PLANE);
        msgs::Set(result.mutable_plane()->mutable_normal(),
            _rml.plane_shape().normal());
        msgs::Set(result.mutable_plane()->mutable_size(),
            _rml.plane_shape().size());
      }
      else if (_rml.has_image_shape())
      {
        result.set_type(msgs::Geometry::IMAGE);
        result.mutable_image()->set_scale(_rml.image_shape().scale());
        result.mutable_image()->set_height(_rml.image_shape().height());
        result.mutable_image()->set_uri(_rml.image_shape().uri());
      }
      else if (_rml.has_heightmap_shape())
      {
        result.set_type(msgs::Geometry::HEIGHTMAP);
        msgs::Set(result.mutable_heightmap()->mutable_size(),
            _rml.heightmap_shape().size());
        msgs::Set(result.mutable_heightmap()->mutable_origin(),
            _rml.heightmap_shape().pos());

        common::Image img(_rml.heightmap_shape().uri());
        msgs::Set(result.mutable_heightmap()->mutable_image(), img);

        msgs::HeightmapGeom::Texture *tex;
        for (std::vector<rml::Heightmap::Texture>::const_iterator iter =
            _rml.heightmap_shape().texture().begin();
            iter != _rml.heightmap_shape().texture().end(); ++iter)
        {
          tex = result.mutable_heightmap()->add_texture();
          tex->set_diffuse((*iter).diffuse());
          tex->set_normal((*iter).normal());
          tex->set_size((*iter).size());
        }

        msgs::HeightmapGeom::Blend *blend;
        for (std::vector<rml::Heightmap::Blend>::const_iterator iter =
            _rml.heightmap_shape().blend().begin();
            iter != _rml.heightmap_shape().blend().end(); ++iter)
        {
          blend = result.mutable_heightmap()->add_blend();

          blend->set_min_height((*iter).min_height());
          blend->set_fade_dist((*iter).fade_dist());
        }

        // Set if the rendering engine uses terrain paging
        bool useTerrainPaging = _rml.heightmap_shape().use_terrain_paging();
        result.mutable_heightmap()->set_use_terrain_paging(useTerrainPaging);
      }
      else if (_rml.has_mesh_shape())
      {
        result.set_type(msgs::Geometry::MESH);
        result.mutable_mesh()->CopyFrom(MeshFromRML(_rml.mesh_shape()));
      }
      else if (_rml.has_empty())
      {
        result.set_type(msgs::Geometry::EMPTY);
      }
      else
        gzerr << "Unknown geometry type\n";

      return result;
    }

    /////////////////////////////////////////////////
    msgs::Visual VisualFromSDF(sdf::ElementPtr _sdf)
    {
      rml::Visual rml;
      rml.SetFromXML(_sdf);
      return VisualFromRML(rml);
    }

    /////////////////////////////////////////////////
    msgs::Visual VisualFromRML(const rml::Visual &_rml)
    {
      msgs::Visual result;

      result.set_name(_rml.name());

      if (_rml.cast_shadows())
        result.set_cast_shadows(_rml.cast_shadows());

      if (_rml.has_transparency())
        result.set_transparency(_rml.transparency());

      if (_rml.has_laser_retro())
        result.set_laser_retro(_rml.laser_retro());

      // Load the geometry
      if (_rml.has_geometry())
      {
        msgs::Geometry *geomMsg = result.mutable_geometry();
        geomMsg->CopyFrom(GeometryFromRML(_rml.geometry()));
      }

      /// Load the material
      if (_rml.has_material())
      {
        msgs::Material *matMsg = result.mutable_material();

        if (_rml.material().has_script())
        {
          matMsg->mutable_script()->set_name(_rml.material().script().name());

          for (std::vector<std::string>::const_iterator iter =
              _rml.material().script().uri().begin();
              iter != _rml.material().script().uri().end(); ++iter)
          {
            matMsg->mutable_script()->add_uri(*iter);
          }
        }

        if (_rml.material().has_shader())
        {
          rml::Visual::Material::Shader shader = _rml.material().shader();

          if (shader.type() == "pixel")
            matMsg->set_shader_type(msgs::Material::PIXEL);
          else if (shader.type() == "vertex")
            matMsg->set_shader_type(msgs::Material::VERTEX);
          else if (shader.type() == "normal_map_object_space")
            matMsg->set_shader_type(msgs::Material::NORMAL_MAP_OBJECT_SPACE);
          else if (shader.type() == "normal_map_tangent_space")
            matMsg->set_shader_type(msgs::Material::NORMAL_MAP_TANGENT_SPACE);
          else
            gzerr << "Unknown shader type[" << shader.type() << "]\n";

          if (shader.has_normal_map())
            matMsg->set_normal_map(shader.normal_map());
        }

        if (_rml.material().has_ambient())
          msgs::Set(matMsg->mutable_ambient(), _rml.material().ambient());
        if (_rml.material().has_diffuse())
          msgs::Set(matMsg->mutable_diffuse(), _rml.material().diffuse());
        if (_rml.material().has_specular())
          msgs::Set(matMsg->mutable_specular(), _rml.material().specular());
        if (_rml.material().has_emissive())
          msgs::Set(matMsg->mutable_emissive(), _rml.material().emissive());
      }

      // Set the origin of the visual
      if (_rml.has_pose())
        msgs::Set(result.mutable_pose(), _rml.pose());

      // Set plugins of the visual
      for (std::vector<rml::Plugin>::const_iterator iter =
          _rml.plugin().begin(); iter != _rml.plugin().end(); ++iter)
      {
        msgs::Plugin *plgnMsg = result.add_plugin();
        plgnMsg->set_name((*iter).name());
        plgnMsg->set_filename((*iter).filename());

        // Add all the plugin properties to the message.
        for (sdf::PropertyList::const_iterator iter2 =
            (*iter).properties.begin();
            iter2 != (*iter).properties.end(); ++iter2)
        {
          FillProperties(*iter2, plgnMsg->add_properties());
        }
      }

      return result;
    }

    msgs::Fog FogFromSDF(sdf::ElementPtr _sdf)
    {
      msgs::Fog result;

      std::string type = _sdf->Get<std::string>("type");
      if (type == "linear")
        result.set_type(msgs::Fog::LINEAR);
      else if (type == "exp")
        result.set_type(msgs::Fog::EXPONENTIAL);
      else if (type == "exp2")
        result.set_type(msgs::Fog::EXPONENTIAL2);
      else if (type == "none")
        result.set_type(msgs::Fog::NONE);
      else
        gzthrow(std::string("Unknown fog type[") + type + "]");

      result.mutable_color()->CopyFrom(
          Convert(_sdf->Get<common::Color>("color")));

      result.set_density(_sdf->Get<double>("density"));
      result.set_start(_sdf->Get<double>("start"));
      result.set_end(_sdf->Get<double>("end"));
      return result;
    }

    msgs::Fog FogFromRML(const rml::Scene::Fog &_rml)
    {
      msgs::Fog result;

      std::string type = _rml.type();
      if (type == "linear")
        result.set_type(msgs::Fog::LINEAR);
      else if (type == "exp")
        result.set_type(msgs::Fog::EXPONENTIAL);
      else if (type == "exp2")
        result.set_type(msgs::Fog::EXPONENTIAL2);
      else if (type == "none")
        result.set_type(msgs::Fog::NONE);
      else
        gzthrow(std::string("Unknown fog type[") + type + "]");

      result.mutable_color()->CopyFrom(Convert(_rml.color()));

      result.set_density(_rml.density());
      result.set_start(_rml.start());
      result.set_end(_rml.end());
      return result;
    }

    msgs::Scene SceneFromSDF(sdf::ElementPtr _sdf)
    {
      msgs::Scene result;

      Init(result, "scene");

      if (_sdf->HasElement("grid"))
        result.set_grid(_sdf->Get<bool>("grid"));
      else
        result.set_grid(true);

      if (_sdf->HasElement("ambient"))
        result.mutable_ambient()->CopyFrom(
            Convert(_sdf->Get<common::Color>("ambient")));

      if (_sdf->HasElement("background"))
      {
        result.mutable_background()->CopyFrom(
            Convert(_sdf->Get<common::Color>("background")));
      }

      if (_sdf->HasElement("sky"))
      {
        msgs::Sky *skyMsg = result.mutable_sky();
        skyMsg->set_time(_sdf->GetElement("sky")->Get<double>("time"));
        skyMsg->set_sunrise(_sdf->GetElement("sky")->Get<double>("sunrise"));
        skyMsg->set_sunset(_sdf->GetElement("sky")->Get<double>("sunset"));

        if (_sdf->GetElement("sky")->HasElement("clouds"))
        {
          sdf::ElementPtr cloudsElem =
            _sdf->GetElement("sky")->GetElement("clouds");
          skyMsg->set_wind_speed(cloudsElem->Get<double>("speed"));
          skyMsg->set_wind_direction(cloudsElem->Get<double>("direction"));
          skyMsg->set_humidity(cloudsElem->Get<double>("humidity"));
          skyMsg->set_mean_cloud_size(cloudsElem->Get<double>("mean_size"));
          msgs::Set(skyMsg->mutable_cloud_ambient(),
                    cloudsElem->Get<common::Color>("ambient"));
        }
      }

      if (_sdf->HasElement("fog"))
        result.mutable_fog()->CopyFrom(FogFromSDF(_sdf->GetElement("fog")));

      if (_sdf->HasElement("shadows"))
        result.set_shadows(_sdf->Get<bool>("shadows"));

      return result;
    }

    msgs::Scene SceneFromRML(const rml::Scene &_rml)
    {
      msgs::Scene result;

      Init(result, "scene");

      if (_rml.has_grid())
        result.set_grid(_rml.grid());
      else
        result.set_grid(true);

      if (_rml.has_ambient())
        result.mutable_ambient()->CopyFrom(Convert(_rml.ambient()));

      if (_rml.has_background())
        result.mutable_background()->CopyFrom(Convert(_rml.background()));

      if (_rml.has_sky())
      {
        msgs::Sky *skyMsg = result.mutable_sky();
        skyMsg->set_time(_rml.sky().time());
        skyMsg->set_sunrise(_rml.sky().sunrise());
        skyMsg->set_sunset(_rml.sky().sunset());

        if (_rml.sky().has_clouds())
        {
          skyMsg->set_wind_speed(_rml.sky().clouds().speed());
          skyMsg->set_wind_direction(_rml.sky().clouds().direction());
          skyMsg->set_humidity(_rml.sky().clouds().humidity());
          skyMsg->set_mean_cloud_size(_rml.sky().clouds().mean_size());
          msgs::Set(skyMsg->mutable_cloud_ambient(),
                    _rml.sky().clouds().ambient());
        }
      }

      if (_rml.has_fog())
        result.mutable_fog()->CopyFrom(FogFromRML(_rml.fog()));

      if (_rml.has_shadows())
        result.set_shadows(_rml.shadows());

      return result;
    }

  }
}


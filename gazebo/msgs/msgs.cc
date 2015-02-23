/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

    void Stamp(msgs::Time *_time)
    {
      common::Time tm = common::Time::GetWallTime();

      _time->set_sec(tm.sec);
      _time->set_nsec(tm.nsec);
    }

    std::string Package(const std::string &type,
        const google::protobuf::Message &message)
    {
      std::string data;
      msgs::Packet pkg;

      Stamp(pkg.mutable_stamp());
      pkg.set_type(type);

      std::string *serialized_data = pkg.mutable_serialized_data();
      if (!message.IsInitialized())
        gzthrow("Can't serialize message of type[" + message.GetTypeName() +
            "] because it is missing required fields");

      if (!message.SerializeToString(serialized_data))
        gzthrow("Failed to serialized message");

      if (!pkg.SerializeToString(&data))
        gzthrow("Failed to serialized message");

      return data;
    }

    void Set(msgs::Vector3d *_pt, const math::Vector3 &_v)
    {
      _pt->set_x(_v.x);
      _pt->set_y(_v.y);
      _pt->set_z(_v.z);
    }

    void Set(msgs::Vector2d *_pt, const math::Vector2d &_v)
    {
      _pt->set_x(_v.x);
      _pt->set_y(_v.y);
    }

    void Set(msgs::Quaternion *_q, const math::Quaternion &_v)
    {
      _q->set_x(_v.x);
      _q->set_y(_v.y);
      _q->set_z(_v.z);
      _q->set_w(_v.w);
    }

    void Set(msgs::Pose *_p, const math::Pose &_v)
    {
      Set(_p->mutable_position(), _v.pos);
      Set(_p->mutable_orientation(), _v.rot);
    }

    void Set(msgs::Color *_c, const common::Color &_v)
    {
      _c->set_r(_v.r);
      _c->set_g(_v.g);
      _c->set_b(_v.b);
      _c->set_a(_v.a);
    }

    void Set(msgs::Time *_t, const common::Time &_v)
    {
      _t->set_sec(_v.sec);
      _t->set_nsec(_v.nsec);
    }

    /////////////////////////////////////////////////
    void Set(msgs::SphericalCoordinates *_s,
             const common::SphericalCoordinates &_v)
    {
      switch (_v.GetSurfaceType())
      {
        case common::SphericalCoordinates::EARTH_WGS84:
          _s->set_surface_model(msgs::SphericalCoordinates::EARTH_WGS84);
          break;
        default:
          gzerr << "Unable to map surface type[" <<  _v.GetSurfaceType()
            << "] to a SphericalCoordinates message.\n";
          _s->set_surface_model(msgs::SphericalCoordinates::EARTH_WGS84);
          break;
      };

      _s->set_latitude_deg(_v.GetLatitudeReference().Degree());
      _s->set_longitude_deg(_v.GetLongitudeReference().Degree());
      _s->set_heading_deg(_v.GetHeadingOffset().Degree());
      _s->set_elevation(_v.GetElevationReference());
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
    void Set(common::Image &_img, const msgs::Image &_msg)
    {
      _img.SetFromData(
          (const unsigned char*)_msg.data().data(),
          _msg.width(),
          _msg.height(),
          (common::Image::PixelFormat)(_msg.pixel_format()));
    }

    /////////////////////////////////////////////////
    void Set(msgs::Image *_msg, const common::Image &_i)
    {
      _msg->set_width(_i.GetWidth());
      _msg->set_height(_i.GetHeight());
      _msg->set_pixel_format(_i.GetPixelFormat());
      _msg->set_step(_i.GetPitch());

      unsigned char *data = NULL;
      unsigned int size;
      _i.GetData(&data, size);
      _msg->set_data(data, size);
      if (data)
      {
        delete[] data;
      }
    }

    /////////////////////////////////////////////////
    msgs::Vector3d Convert(const math::Vector3 &_v)
    {
      msgs::Vector3d result;
      result.set_x(_v.x);
      result.set_y(_v.y);
      result.set_z(_v.z);
      return result;
    }

    msgs::Quaternion Convert(const math::Quaternion &_q)
    {
      msgs::Quaternion result;
      result.set_x(_q.x);
      result.set_y(_q.y);
      result.set_z(_q.z);
      result.set_w(_q.w);
      return result;
    }

    msgs::Pose Convert(const math::Pose &_p)
    {
      msgs::Pose result;
      result.mutable_position()->CopyFrom(Convert(_p.pos));
      result.mutable_orientation()->CopyFrom(Convert(_p.rot));
      return result;
    }

    msgs::Color Convert(const common::Color &_c)
    {
      msgs::Color result;
      result.set_r(_c.r);
      result.set_g(_c.g);
      result.set_b(_c.b);
      result.set_a(_c.a);
      return result;
    }

    msgs::Time Convert(const common::Time &_t)
    {
      msgs::Time result;
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

    msgs::Joint::Type ConvertJointType(const std::string &_str)
    {
      msgs::Joint::Type result = msgs::Joint::REVOLUTE;
      if (_str == "revolute")
      {
        result = msgs::Joint::REVOLUTE;
      }
      else if (_str == "revolute2")
      {
        result = msgs::Joint::REVOLUTE2;
      }
      else if (_str == "prismatic")
      {
        result = msgs::Joint::PRISMATIC;
      }
      else if (_str == "universal")
      {
        result = msgs::Joint::UNIVERSAL;
      }
      else if (_str == "ball")
      {
        result = msgs::Joint::BALL;
      }
      else if (_str == "screw")
      {
        result = msgs::Joint::SCREW;
      }
      else if (_str == "gearbox")
      {
        result = msgs::Joint::GEARBOX;
      }
      return result;
    }

    std::string ConvertJointType(const msgs::Joint::Type _type)
    {
      std::string result;
      switch (_type)
      {
        case msgs::Joint::REVOLUTE:
        {
          result = "revolute";
          break;
        }
        case msgs::Joint::REVOLUTE2:
        {
          result = "revolute2";
          break;
        }
        case msgs::Joint::PRISMATIC:
        {
          result = "prismatic";
          break;
        }
        case msgs::Joint::UNIVERSAL:
        {
          result = "universal";
          break;
        }
        case msgs::Joint::BALL:
        {
          result = "ball";
          break;
        }
        case msgs::Joint::SCREW:
        {
          result = "screw";
          break;
        }
        case msgs::Joint::GEARBOX:
        {
          result = "gearbox";
          break;
        }
        default:
        {
          result = "unknown";
          break;
        }
      }
      return result;
    }

    math::Vector3 Convert(const msgs::Vector3d &_v)
    {
      return math::Vector3(_v.x(), _v.y(), _v.z());
    }

    math::Vector2d Convert(const msgs::Vector2d &_v)
    {
      return math::Vector2d(_v.x(), _v.y());
    }

    math::Quaternion Convert(const msgs::Quaternion &_q)
    {
      return math::Quaternion(_q.w(), _q.x(), _q.y(), _q.z());
    }

    math::Pose Convert(const msgs::Pose &_p)
    {
      return math::Pose(Convert(_p.position()),
          Convert(_p.orientation()));
    }

    common::Color Convert(const msgs::Color &_c)
    {
      return common::Color(_c.r(), _c.g(), _c.b(), _c.a());
    }

    common::Time Convert(const msgs::Time &_t)
    {
      return common::Time(_t.sec(), _t.nsec());
    }

    math::Plane Convert(const msgs::PlaneGeom &_p)
    {
      return math::Plane(Convert(_p.normal()),
          math::Vector2d(_p.size().x(), _p.size().y()),
          _p.d());
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
      msgs::MeshGeom result;

      if (_sdf->GetName() != "mesh")
      {
        gzerr << "Cannot create a mesh message from an "
          << _sdf->GetName() << " SDF element.\n";
        return result;
      }

        msgs::Set(result.mutable_scale(), _sdf->Get<math::Vector3>("scale"));

        result.set_filename(_sdf->Get<std::string>("uri"));

        if (_sdf->HasElement("submesh"))
        {
          sdf::ElementPtr submeshElem = _sdf->GetElement("submesh");
          if (submeshElem->HasElement("name") &&
              submeshElem->Get<std::string>("name") != "__default__")
          {
            result.set_submesh(submeshElem->Get<std::string>("name"));

            if (submeshElem->HasElement("center"))
              result.set_center_submesh(submeshElem->Get<bool>("center"));
          }
        }

      return result;
    }


    /////////////////////////////////////////////////
    msgs::Geometry GeometryFromSDF(sdf::ElementPtr _sdf)
    {
      msgs::Geometry result;

      if (_sdf->GetName() != "geometry")
      {
        gzerr << "Cannot create a geometry message from an "
          << _sdf->GetName() << " SDF element.\n";
        return result;
      }

      // Load the geometry
      sdf::ElementPtr geomElem = _sdf->GetFirstElement();
      if (!geomElem)
        gzthrow("Invalid geometry element");

      if (geomElem->GetName() == "box")
      {
        result.set_type(msgs::Geometry::BOX);
        msgs::Set(result.mutable_box()->mutable_size(),
            geomElem->Get<math::Vector3>("size"));
      }
      else if (geomElem->GetName() == "cylinder")
      {
        result.set_type(msgs::Geometry::CYLINDER);
        result.mutable_cylinder()->set_radius(
            geomElem->Get<double>("radius"));
        result.mutable_cylinder()->set_length(
            geomElem->Get<double>("length"));
      }
      else if (geomElem->GetName() == "sphere")
      {
        result.set_type(msgs::Geometry::SPHERE);
        result.mutable_sphere()->set_radius(
            geomElem->Get<double>("radius"));
      }
      else if (geomElem->GetName() == "plane")
      {
        result.set_type(msgs::Geometry::PLANE);
        msgs::Set(result.mutable_plane()->mutable_normal(),
            geomElem->Get<math::Vector3>("normal"));
        msgs::Set(result.mutable_plane()->mutable_size(),
            geomElem->Get<math::Vector2d>("size"));
      }
      else if (geomElem->GetName() == "polyline")
      {
        result.set_type(msgs::Geometry::POLYLINE);
        result.mutable_polyline()->set_height(geomElem->Get<double>("height"));
        sdf::ElementPtr pointElem = geomElem->GetElement("point");
        while (pointElem)
        {
           math::Vector2d point = pointElem->Get<math::Vector2d>();
           pointElem = pointElem->GetNextElement("point");
           msgs::Vector2d *ptMsg = result.mutable_polyline()->add_point();
           msgs::Set(ptMsg, point);
        }
      }
      else if (geomElem->GetName() == "image")
      {
        result.set_type(msgs::Geometry::IMAGE);
        result.mutable_image()->set_scale(
            geomElem->Get<double>("scale"));
        result.mutable_image()->set_height(
            geomElem->Get<double>("height"));
        result.mutable_image()->set_uri(
            geomElem->Get<std::string>("uri"));
      }
      else if (geomElem->GetName() == "heightmap")
      {
        result.set_type(msgs::Geometry::HEIGHTMAP);
        msgs::Set(result.mutable_heightmap()->mutable_size(),
            geomElem->Get<math::Vector3>("size"));
        msgs::Set(result.mutable_heightmap()->mutable_origin(),
            geomElem->Get<math::Vector3>("pos"));

        sdf::ElementPtr textureElem = geomElem->GetElement("texture");
        while (textureElem)
        {
          msgs::HeightmapGeom::Texture *tex =
            result.mutable_heightmap()->add_texture();
          tex->set_diffuse(textureElem->Get<std::string>("diffuse"));
          tex->set_normal(textureElem->Get<std::string>("normal"));
          tex->set_size(textureElem->Get<double>("size"));
          textureElem = textureElem->GetNextElement("texture");
        }

        sdf::ElementPtr blendElem = geomElem->GetElement("blend");
        while (blendElem)
        {
          msgs::HeightmapGeom::Blend *blend =
            result.mutable_heightmap()->add_blend();

          blend->set_min_height(blendElem->Get<double>("min_height"));
          blend->set_fade_dist(blendElem->Get<double>("fade_dist"));
          blendElem = blendElem->GetNextElement("blend");
        }

        // Set if the rendering engine uses terrain paging
        bool useTerrainPaging =
            geomElem->Get<bool>("use_terrain_paging");
        result.mutable_heightmap()->set_use_terrain_paging(useTerrainPaging);
      }
      else if (geomElem->GetName() == "mesh")
      {
        result.set_type(msgs::Geometry::MESH);
        result.mutable_mesh()->CopyFrom(MeshFromSDF(geomElem));
      }
      else if (geomElem->GetName() == "empty")
      {
        result.set_type(msgs::Geometry::EMPTY);
      }
      else
        gzthrow("Unknown geometry type\n");

      return result;
    }

    /////////////////////////////////////////////////
    msgs::Visual VisualFromSDF(sdf::ElementPtr _sdf)
    {
      msgs::Visual result;

      result.set_name(_sdf->Get<std::string>("name"));

      if (_sdf->HasElement("cast_shadows"))
        result.set_cast_shadows(_sdf->Get<bool>("cast_shadows"));

      if (_sdf->HasElement("transparency"))
        result.set_transparency(_sdf->Get<double>("transparency"));

      if (_sdf->HasElement("laser_retro"))
        result.set_laser_retro(_sdf->Get<double>("laser_retro"));

      // Load the geometry
      if (_sdf->HasElement("geometry"))
      {
        msgs::Geometry *geomMsg = result.mutable_geometry();
        geomMsg->CopyFrom(GeometryFromSDF(_sdf->GetElement("geometry")));
      }

      /// Load the material
      if (_sdf->HasElement("material"))
      {
        sdf::ElementPtr elem = _sdf->GetElement("material");
        msgs::Material *matMsg = result.mutable_material();

        if (elem->HasElement("script"))
        {
          sdf::ElementPtr scriptElem = elem->GetElement("script");
          matMsg->mutable_script()->set_name(
              scriptElem->Get<std::string>("name"));

          sdf::ElementPtr uriElem = scriptElem->GetElement("uri");
          while (uriElem)
          {
            matMsg->mutable_script()->add_uri(uriElem->Get<std::string>());
            uriElem = uriElem->GetNextElement("uri");
          }
        }

        if (elem->HasElement("lighting"))
        {
          matMsg->set_lighting(elem->Get<bool>("lighting"));
        }

        if (elem->HasElement("shader"))
        {
          sdf::ElementPtr shaderElem = elem->GetElement("shader");

          if (shaderElem->Get<std::string>("type") == "pixel")
            matMsg->set_shader_type(msgs::Material::PIXEL);
          else if (shaderElem->Get<std::string>("type") == "vertex")
            matMsg->set_shader_type(msgs::Material::VERTEX);
          else if (shaderElem->Get<std::string>("type") ==
              "normal_map_object_space")
            matMsg->set_shader_type(msgs::Material::NORMAL_MAP_OBJECT_SPACE);
          else if (shaderElem->Get<std::string>("type") ==
              "normal_map_tangent_space")
            matMsg->set_shader_type(msgs::Material::NORMAL_MAP_TANGENT_SPACE);
          else
            gzthrow(std::string("Unknown shader type[") +
                shaderElem->Get<std::string>("type") + "]");

          if (shaderElem->HasElement("normal_map"))
            matMsg->set_normal_map(
                shaderElem->GetElement("normal_map")->Get<std::string>());
        }

        if (elem->HasElement("ambient"))
          msgs::Set(matMsg->mutable_ambient(),
              elem->Get<common::Color>("ambient"));
        if (elem->HasElement("diffuse"))
          msgs::Set(matMsg->mutable_diffuse(),
              elem->Get<common::Color>("diffuse"));
        if (elem->HasElement("specular"))
          msgs::Set(matMsg->mutable_specular(),
              elem->Get<common::Color>("specular"));
        if (elem->HasElement("emissive"))
          msgs::Set(matMsg->mutable_emissive(),
              elem->Get<common::Color>("emissive"));
      }

      // Set the origin of the visual
      if (_sdf->HasElement("pose"))
      {
        msgs::Set(result.mutable_pose(), _sdf->Get<math::Pose>("pose"));
      }

      // Set plugins of the visual
      if (_sdf->HasElement("plugin"))
      {
        sdf::ElementPtr elem = _sdf->GetElement("plugin");
        msgs::Plugin *plgnMsg = result.mutable_plugin();
        // if (elem->HasElement("name"))
          plgnMsg->set_name(elem->Get<std::string>("name"));
        // if (elem->HasElement("filename"))
          plgnMsg->set_filename(elem->Get<std::string>("filename"));

        std::stringstream ss;
        for (sdf::ElementPtr innerElem = elem->GetFirstElement();
            innerElem;
            innerElem = innerElem->GetNextElement(""))
        {
          ss << innerElem->ToString("");
        }
        plgnMsg->set_innerxml("<sdf>" + ss.str() + "</sdf>");
      }

      return result;
    }

    /////////////////////////////////////////////////
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

    /////////////////////////////////////////////////
    sdf::ElementPtr LightToSDF(const msgs::Light &_msg, sdf::ElementPtr _sdf)
    {
      sdf::ElementPtr lightSDF;

      if (_sdf)
      {
        lightSDF = _sdf;
      }
      else
      {
        lightSDF.reset(new sdf::Element);
        sdf::initFile("light.sdf", lightSDF);
      }

      lightSDF->GetAttribute("name")->Set(_msg.name());

      if (_msg.has_type() && _msg.type() == msgs::Light::POINT)
        lightSDF->GetAttribute("type")->Set("point");
      else if (_msg.has_type() && _msg.type() == msgs::Light::SPOT)
        lightSDF->GetAttribute("type")->Set("spot");
      else if (_msg.has_type() && _msg.type() == msgs::Light::DIRECTIONAL)
        lightSDF->GetAttribute("type")->Set("directional");

      if (_msg.has_pose())
      {
        lightSDF->GetElement("pose")->Set(msgs::Convert(_msg.pose()));
      }

      if (_msg.has_diffuse())
      {
        lightSDF->GetElement("diffuse")->Set(msgs::Convert(_msg.diffuse()));
      }

      if (_msg.has_specular())
      {
        lightSDF->GetElement("specular")->Set(msgs::Convert(_msg.specular()));
      }

      if (_msg.has_direction())
      {
        lightSDF->GetElement("direction")->Set(msgs::Convert(_msg.direction()));
      }

      if (_msg.has_attenuation_constant())
      {
        sdf::ElementPtr elem = lightSDF->GetElement("attenuation");
        elem->GetElement("constant")->Set(_msg.attenuation_constant());
      }

      if (_msg.has_attenuation_linear())
      {
        sdf::ElementPtr elem = lightSDF->GetElement("attenuation");
        elem->GetElement("linear")->Set(_msg.attenuation_linear());
      }

      if (_msg.has_attenuation_quadratic())
      {
        sdf::ElementPtr elem = lightSDF->GetElement("attenuation");
        elem->GetElement("quadratic")->Set(_msg.attenuation_quadratic());
      }

      if (_msg.has_range())
      {
        sdf::ElementPtr elem = lightSDF->GetElement("attenuation");
        elem->GetElement("range")->Set(_msg.range());
      }

      if (_msg.has_cast_shadows())
        lightSDF->GetElement("cast_shadows")->Set(_msg.cast_shadows());

      if (_msg.has_spot_inner_angle())
      {
        sdf::ElementPtr elem = lightSDF->GetElement("spot");
        elem->GetElement("inner_angle")->Set(_msg.spot_inner_angle());
      }

      if (_msg.has_spot_outer_angle())
      {
        sdf::ElementPtr elem = lightSDF->GetElement("spot");
        elem->GetElement("outer_angle")->Set(_msg.spot_outer_angle());
      }

      if (_msg.has_spot_falloff())
      {
        sdf::ElementPtr elem = lightSDF->GetElement("spot");
        elem->GetElement("falloff")->Set(_msg.spot_falloff());
      }
      return lightSDF;
    }
  }
}

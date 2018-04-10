/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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

#include "plugins/CameraGlintPlugin.hh"
#include "gazebo/sensors/CameraSensor.hh"
#include "gazebo/common/Console.hh"

#include "gazebo/rendering/Camera.hh"
#include "gazebo/rendering/RenderEngine.hh"

#include "gazebo/math/Vector3.hh"

#include "sdf/sdf.hh"

GZ_REGISTER_SENSOR_PLUGIN(gazebo::CameraGlintPlugin)


namespace gazebo
{
  // This computes a constant rotational offset from the fragment shader
  // reference coordinates to the camera reference coordinates.
  static ignition::math::Matrix3d ComputeFragmentShaderToCameraRotation()
  {
    const ignition::math::Quaterniond R_90Y{0.0, 90.0*M_PI/180.0, 0.0};
    const ignition::math::Quaterniond R_n90X{-90.0*M_PI/180.0, 0.0, 0.0};

    return ignition::math::Matrix3d(R_90Y) * ignition::math::Matrix3d(R_n90X);
  }


  class CameraGlintSchemeListener : public Ogre::MaterialManager::Listener
  {
    public: CameraGlintSchemeListener(
        const std::string &_glintScheme)
      : glintScheme(_glintScheme)
    {
      material =
          Ogre::MaterialManager::getSingleton().getByName("Gazebo/CameraGlint");

      if(material.isNull())
        gzerr << "Failed to find Gazebo/CameraGlint naterial\n";

      material->load();
    }

    Ogre::Technique *handleSchemeNotFound(
          unsigned short /*_schemeIndex*/,
          const std::string &_schemeName,
          Ogre::Material * /*_originalMaterial*/,
          unsigned short /*_lodIndex*/,
          const Ogre::Renderable * /*_rend*/) override
    {
      if (_schemeName == glintScheme)
      {
        // Note: DO NOT use material->getBestTechnique() in this function,
        // because it will result in infinite recursion.
        return material->getTechnique(0);
      }

      return nullptr;
    }

    public: const std::string glintScheme;
    public: Ogre::MaterialPtr material;
  };

  class GlintCamera : public sensors::CameraSensor
  {
    protected: bool firstPass = true;

    protected: bool UpdateImpl(const bool _force) override
    {
      CameraSensor::UpdateImpl(_force);

      const unsigned char *imageData = this->ImageData();
      if (!imageData)
        return true;

      if (firstPass)
      {
        firstPass = false;

        common::Image image;
        image.SetFromData(
              imageData,
              this->ImageWidth(),
              this->ImageHeight(),
              common::Image::ConvertPixelFormat(this->Camera()->ImageFormat()));

        unsigned int wCount = 0;
        for (unsigned int i = 0; i < image.GetWidth(); ++i)
        {
          for (unsigned int j = 0; j < image.GetHeight(); ++j)
          {
            const common::Color &color = image.GetPixel(i, j);
            if (color != common::Color::Black)
              ++wCount;
          }
        }
        std::cout << "Glinted pixel count: " << wCount << std::endl;
      }

      return true;
    }
  };


  class CameraGlintPluginPrivate
  {
    public: std::unique_ptr<CameraGlintSchemeListener> glintSchemeListener;
    public: std::shared_ptr<gazebo::sensors::CameraSensor> glinter;
    public: std::shared_ptr<gazebo::sensors::CameraSensor> parent;

    public: const ignition::math::Matrix3d f_R_c;

    public: ignition::math::Vector3d sunlight;
    public: float tolerance;

    public: sdf::ElementPtr sdf;

    public: bool valid;

    public: bool firstPass = true;

    public: event::ConnectionPtr updateConnection;

    CameraGlintPluginPrivate()
      : f_R_c(ComputeFragmentShaderToCameraRotation())
    {
      // Let Init() do the rest of the initializing
    }

    public: void Init()
    {
      if(!this->parent)
      {
        gzerr << "CameraGlintPlugin was not attached to a camera!\n";
        this->valid = false;
        return;
      }

      this->sdf = std::make_shared<sdf::Element>();
      sdf::initFile("sensor.sdf", this->sdf);
      this->sdf->GetAttribute("name")->Set("GlintCamera");
      this->sdf->GetElement("topic")->Set("GlintTopic");
      sdf::ElementPtr camera = this->sdf->GetElement("camera");
      sdf::ElementPtr image = camera->GetElement("image");
      image->GetElement("width")->Set<int>(parent->Camera()->ImageWidth());
      image->GetElement("height")->Set<int>(parent->Camera()->ImageHeight());
      sdf::ElementPtr clip = camera->GetElement("clip");
      clip->GetElement("near")->Set<double>(parent->Camera()->NearClip());
      clip->GetElement("far")->Set<double>(parent->Camera()->FarClip());


      glinter = std::make_shared<GlintCamera>();
      glinter->Load(parent->WorldName(), this->sdf);
      glinter->SetParent(this->parent->ParentName(),
                         this->parent->ParentId());
      glinter->Init();

      glinter->Camera()->SetScene(
            gazebo::rendering::RenderEngine::Instance()->GetScene(0));

      this->SetCamera(this->glinter->Camera());

      this->updateConnection =
          this->parent->ConnectUpdated( [&](){ this->Refresh(); } );
    }

    void Refresh()
    {
      // Something seems to change the background of the viewport to gray after
      // the Init() step, so we force the background to black during the first
      // update cycle.
      if (firstPass)
      {
        glinter->Camera()->OgreViewport()->setBackgroundColour(
              Ogre::ColourValue(0.0, 0.0, 0.0, 1.0));
        firstPass = false;
      }

      const ignition::math::Pose3d &pose = this->parent->Camera()->WorldPose();
      glinter->Camera()->SetWorldPose(pose);

      // Rotate the sunlight direction from the world coordinates to the
      // fragment shader coordinates.
      const ignition::math::Vector3d sunlightFS =
          this->f_R_c * ignition::math::Matrix3d(pose.Rot()).Transposed()
          * this->sunlight;

      const Ogre::GpuProgramParametersSharedPtr &params =
          this->glintSchemeListener->material->getTechnique(0)->
            getPass(0)->getFragmentProgramParameters();

      params->setNamedConstant(
            "sunlight",
            Ogre::Vector3(sunlightFS[0], sunlightFS[1], sunlightFS[2]));

      glinter->Update(true);
    }


    public: void SetCamera(const rendering::CameraPtr &_camera)
    {
      const std::string glintSchemeName = "Glint";
      _camera->OgreViewport()->setMaterialScheme(glintSchemeName);

      this->glintSchemeListener.reset(
            new CameraGlintSchemeListener(glintSchemeName));
      Ogre::MaterialManager::getSingleton()
          .addListener(glintSchemeListener.get());

      // Set the tolerance for the shader program. We can just do this once
      // during initialization, since it doesn't change during the run of the
      // program.
      const Ogre::GpuProgramParametersSharedPtr &params =
          this->glintSchemeListener->material->getTechnique(0)->
            getPass(0)->getFragmentProgramParameters();
      params->setNamedConstant("tolerance", this->tolerance);
    }
  };

  CameraGlintPlugin::CameraGlintPlugin()
    : dataPtr(new CameraGlintPluginPrivate)
  {
    // Do nothing
  }

  CameraGlintPlugin::~CameraGlintPlugin()
  {
    // Defined here for PIMPL
  }

  void CameraGlintPlugin::Init()
  {
    this->dataPtr->Init();
  }

  void CameraGlintPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
  {
    this->dataPtr->parent =
        std::dynamic_pointer_cast<gazebo::sensors::CameraSensor>(_sensor);

    this->dataPtr->sdf = _sdf;

    if (!this->dataPtr->parent)
    {
      gzerr << "The CameraGlintPlugin must be tied to a CameraSensor!\n";
      return;
    }

    // The default sunlight direction will point directly downwards, like the
    // highest possible noon.
    this->dataPtr->sunlight = ignition::math::Vector3d(0.0, 0.0, -1.0);
    if (_sdf->HasElement("sunlight"))
    {
      this->dataPtr->sunlight = _sdf->GetElement("sunlight")->
          Get<gazebo::math::Vector3>("").Normalize().Ign();
    }


    // The angular diameter of the sun from Earth is approximately 32'33" where
    // ' is arc-minutes and " is arc-seconds.
    double sunAngularDiameter = (32.0/60.0 + 33.0/3600.0)*M_PI/180.0;
    if (_sdf->HasElement("sun_angular_diameter"))
    {
      sunAngularDiameter = _sdf->GetElement("sun_angular_diameter")->
          Get<double>("");
    }
    // When we evaluate the dot product between the reflected beam and the
    // sunlight direction, its absolute value must be less than or equal to this
    // tolerance value for it to be a sunlight reflection.
    this->dataPtr->tolerance = 1.0 - cos(sunAngularDiameter/2.0);
  }
}

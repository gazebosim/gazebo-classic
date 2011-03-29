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
#ifndef SCENE_HH
#define SCENE_HH

#include <vector>
#include <map>

#include <OGRE/OgreMesh.h>

#include "transport/Subscriber.hh"
#include "common/Events.hh"
#include "common/Messages.hh"
#include "common/Param.hh"
#include "common/Color.hh"
#include "common/Vector2i.hh"

namespace Ogre
{
  class SceneManager;
  class RaySceneQuery;
  class Node;
}

namespace gazebo
{
  namespace common
  {
    class XMLConfigNode;
    class Message;
  }

	namespace rendering
  {
    class Light;
    class Visual;
    class Grid;
    class Camera;
    class UserCamera;
  
    class Scene
    {
  
      public: enum SceneType {BSP, GENERIC};
  
      /// \brief Constructor
      public: Scene(const std::string &name);
  
      /// \brief Destructor
      public: virtual ~Scene();
  
      /// \brief Load the scene
      public: void Load(common::XMLConfigNode *node);
  
      /// \brief Init
      public: void Init();

      /// \brief Save the scene
      public: void Save(std::string &prefix, std::ostream &stream);
 
      /// \brief Process all received messages 
      public: void PreRender();

      /// \brief Get the OGRE scene manager
      public: Ogre::SceneManager *GetManager() const;
  
      /// \brief Get the name of the scene
      public: std::string GetName() const;
  
      /// \brief Set the ambient color
      public: void SetAmbientColor(const common::Color &color);
  
      /// \brief Get the ambient color
      public: common::Color GetAmbientColor() const;
  
      /// \brief Set the background color
      public: void SetBackgroundColor(const common::Color &color);

      /// \brief Get the background color
      public: common::Color GetBackgroundColor() const;
  
      /// \brief Set the scene type
      public: void SetType(SceneType type);
  
      /// \brief Get the scene type
      public: SceneType GetType() const;
  
      /// \brief Create a grid
      public: void CreateGrid(uint32_t cell_count, float cell_length, 
                              float line_width, const common::Color &color );
  
      /// \brief Get the grid
      public: Grid *GetGrid(unsigned int index) const;
  
      /// \brief Create a camera
      public: Camera *CreateCamera(const std::string &name);
  
      /// \brief Get the number of cameras in this scene
      public: unsigned int GetCameraCount() const;
  
      /// \brief Get a camera
      public: Camera *GetCamera(unsigned int index) const;
  
      /// \brief Create a user camera
      public: UserCamera *CreateUserCamera(const std::string &name);
  
      /// \brief Get the number of user cameras in this scene
      public: unsigned int GetUserCameraCount() const;
  
      /// \brief Get a user camera
      public: UserCamera *GetUserCamera(unsigned int index) const;
  
  
      /// \brief Get an entity at a pixel location using a camera. Used for
      ///        mouse picking. 
      /// \param camera The ogre camera, used to do mouse picking
      /// \param mousePos The position of the mouse in screen coordinates
      /// \return The selected entity, or NULL
      /*public: common::Common *GetEntityAt(Camera *camera, 
                                          common::Vector2i mousePos, 
                                          std::string &mod);
                                          */
  
      /// \brief Get the world pos of a the first contact at a pixel location
      public: common::Vector3 GetFirstContact(Camera *camera, 
                                              common::Vector2i mousePos);
  
              // NATY
      /// \brief Register a user camera
      //public: void RegisterCamera( Camera *cam );
      //public: void UnregisterCamera( Camera *cam );
  
      public: void PrintSceneGraph();
  
      /// \brief Hide a visual
      public: void SetVisible(const std::string &name, bool visible);
  
      /// \brief Draw a named line
      public: void DrawLine(const common::Vector3 &start, 
                            const common::Vector3 &end, 
                            const std::string &name);
  
      public: void SetFog( std::string type, const common::Color &color, 
                           double density, 
                           double start, double end );
  
      // Get the scene ID
      public: unsigned int GetId() const;
  
      // Get the scene Id as a string
      public: std::string GetIdString() const;
  
      /// \brief Print scene graph
      private: void PrintSceneGraphHelper(std::string prefix, 
                                          Ogre::Node *node);
  
      public: void InitShadows();
  
      // \brief Get the mesh information for the given mesh.
      // Code found in Wiki: www.ogre3d.org/wiki/index.php/RetrieveVertexData
      private: void GetMeshInformation(const Ogre::MeshPtr mesh,
                                       size_t &vertex_count,
                                       Ogre::Vector3* &vertices,
                                       size_t &index_count,
                                       unsigned long* &indices,
                                       const Ogre::Vector3 &position,
                                       const Ogre::Quaternion &orient,
                                       const Ogre::Vector3 &scale);
  
      private: void ReceiveSceneMsg(const boost::shared_ptr<msgs::Scene const> &msg);

      private: void ReceiveVisualMsg(const boost::shared_ptr<msgs::Visual const> &msg);
      private: void ProcessVisualMsg(const boost::shared_ptr<msgs::Visual const> &msg);

      private: void ReceiveLightMsg(const boost::shared_ptr<msgs::Light const> &msg);
      private: void ProcessLightMsg(const boost::shared_ptr<msgs::Light const> &msg);

      private: void HandleSelectionMsg(const boost::shared_ptr<msgs::Selection const> &msg);
               
      private: void ReceivePoseMsg(const boost::shared_ptr<msgs::Pose const> &msg);
  
      private: std::string name;
      private: common::ParamT<common::Color> *ambientP;
      private: common::ParamT<common::Color> *backgroundColorP;
      private: common::ParamT<common::Color> *fogColorP;
      private: common::ParamT<std::string> *fogTypeP;
      private: common::ParamT<double> *fogDensityP;
      private: common::ParamT<double> *fogStartP;
      private: common::ParamT<double> *fogEndP;
      private: common::ParamT<std::string> *skyMaterialP;
      private: common::ParamT<bool> *shadowEnabledP;
      private: common::ParamT<common::Color> *shadowColorP;
      private: common::ParamT<std::string> *shadowTypeP;
  
      private: std::vector<common::Param*> parameters;
  
      //bsp attributes saved to write XML file back
      private: SceneType type;
  
      private: std::vector<Camera*> cameras;
      private: std::vector<UserCamera*> userCameras;
  
      private: Ogre::SceneManager *manager;
      private: Ogre::RaySceneQuery *raySceneQuery;
  
      private: std::vector<Grid *> grids;
  
      private: static unsigned int idCounter;
      private: unsigned int id;
      private: std::string idString;
  
      typedef std::list<boost::shared_ptr<msgs::Visual const> > VisualMsgs_L;
      private: VisualMsgs_L visualMsgs;

      typedef std::list<boost::shared_ptr<msgs::Light const> > LightMsgs_L;
      private: LightMsgs_L lightMsgs;

      typedef std::list<boost::shared_ptr<msgs::Pose const> > PoseMsgs_L;
      private: PoseMsgs_L poseMsgs;
  
      typedef std::map<std::string, Visual*> Visual_M;
      private: Visual_M visuals;

      typedef std::map<std::string, Light*> Light_M;
      private: Light_M lights;

      private: boost::mutex *receiveMutex;
  
      private: transport::SubscriberPtr sceneSub;
      private: transport::SubscriberPtr visSub;
      private: transport::SubscriberPtr lightSub;
      private: transport::SubscriberPtr poseSub;
      private: transport::SubscriberPtr selectionSub;

      private: std::vector<event::ConnectionPtr> connections;
    };
  }
}
#endif 

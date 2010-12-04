#ifndef SCENE_HH
#define SCENE_HH

#include <vector>

#include "Param.hh"
#include "Color.hh"
#include "Vector2.hh"

namespace Ogre
{
  class SceneManager;
  class RaySceneQuery;
}

namespace gazebo
{
  class Visual;
  class Message;
  class Grid;
  class Camera;
  class UserCamera;
  class Entity;
  class Common;

  class Scene
  {
    public: enum SceneType {BSP, GENERIC};

    /// \brief Constructor
    public: Scene(const std::string &name);

    /// \brief Destructor
    public: virtual ~Scene();

    /// \brief Load the scene
    public: void Load(XMLConfigNode *node);

    /// \brief Init
    public: void Init();

    /// \brief Save the scene
    public: void Save(std::string &prefix, std::ostream &stream);

    /// \brief Get the OGRE scene manager
    public: Ogre::SceneManager *GetManager() const;

    /// \brief Get the name of the scene
    public: std::string GetName() const;

    /// \brief Set the ambient color
    public: void SetAmbientColor(const Color &color);

    /// \brief Get the ambient color
    public: Color GetAmbientColor() const;

    /// \brief Set the background color
    public: void SetBackgroundColor(const Color &color);
    /// \brief Get the background color
    public: Color GetBackgroundColor() const;

    /// \brief Set the scene type
    public: void SetType(SceneType type);

    /// \brief Get the scene type
    public: SceneType GetType() const;

    /// \brief Create a grid
    public: void CreateGrid(uint32_t cell_count, float cell_length, 
                            float line_width, const Color &color );

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
    public: Common *GetEntityAt(Camera *camera, 
                                Vector2<int> mousePos, std::string &mod);

    /// \brief Get the world pos of a the first contact at a pixel location
    public: Vector3 GetFirstContact(Camera *camera, Vector2<int> mousePos);

            // NATY
    /// \brief Register a user camera
    //public: void RegisterCamera( Camera *cam );
    //public: void UnregisterCamera( Camera *cam );

    public: void PrintSceneGraph();

    /// \brief Hide a visual
    public: void SetVisible(const std::string &name, bool visible);

    /// \brief Draw a named line
    public: void DrawLine(const Vector3 &start, const Vector3 &end, 
                          const std::string &name);

    public: void SetFog( std::string type, const Color &color, double density, 
                         double start, double end );

    // Get the scene ID
    public: unsigned int GetId() const;

    // Get the scene Id as a string
    public: std::string GetIdString() const;

    /// \brief Print scene graph
    private: void PrintSceneGraphHelper(std::string prefix, 
                                        Ogre::Node *node);

    public: void InitShadows();

    /// \brief Receive a message
    public: void ReceiveMessage( const Message &msg );

    /// Process all messages
    public: void ProcessMessages();

    private: std::string name;
    private: ParamT<Color> *ambientP;
    private: ParamT<Color> *backgroundColorP;
    private: ParamT<Color> *fogColorP;
    private: ParamT<std::string> *fogTypeP;
    private: ParamT<double> *fogDensityP;
    private: ParamT<double> *fogStartP;
    private: ParamT<double> *fogEndP;
    private: ParamT<std::string> *skyMaterialP;
    private: ParamT<bool> *shadowsP;

    private: std::vector<Param*> parameters;

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

    private: boost::mutex mutex;
    private: std::vector<Message*> messages;

    private: std::map<std::string, Visual*> visuals;

  };
};
#endif 

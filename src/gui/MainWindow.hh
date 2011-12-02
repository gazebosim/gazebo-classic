#ifndef MAINWINDOW_HH
#define MAINWINDOW_HH

#include <qmainwindow.h>
#include "common/Event.hh"
#include "msgs/MessageTypes.hh"
#include "transport/TransportTypes.hh"

namespace gazebo
{
  namespace gui
  {
    class RenderWidget;
    class TimePanel;
    class GLWidget;

    class WorldPropertiesWidget;

    class MainWindow : public QMainWindow
    {
      Q_OBJECT

      public: MainWindow();
      public: virtual ~MainWindow();

      public: void Load();
      public: void Init();

      public: unsigned int GetEntityId(const std::string &_name);
      public: bool HasEntityName(const std::string &_name);

      protected: void closeEvent(QCloseEvent *event);

      private: void OnGUI(const boost::shared_ptr<msgs::GUI const> &_msg);

      private slots: void New();
      private slots: void Open();
      private slots: void Save();
      private slots: void About();
      private slots: void Play();
      private slots: void Pause();
      private slots: void Step();

      private slots: void NewModel();
      private slots: void EditWorldProperties();

      private slots: void CreateBox();
      private slots: void CreateSphere();
      private slots: void CreateCylinder();
      private slots: void CreatePointLight();
      private slots: void CreateSpotLight();
      private slots: void CreateDirectionalLight();
      private slots: void InsertModel();
      private slots: void ViewFullScreen();
      private slots: void ViewFPS();
      private slots: void ViewOrbit();
      private slots: void OnResetWorld();

      private: void OnFullScreen(bool _value);
      private: void OnMoveMode(bool mode);

      private: void CreateActions();
      private: void CreateMenus();
      private: void CreateToolbars();

      private: void OnModel(const boost::shared_ptr<msgs::Model const> &_msg);
      private: void OnSetSelectedEntity(const std::string &_name);
      private: void OnResponse(
                   const boost::shared_ptr<msgs::Response const> &_msg);
      private: void OnWorldModify(
                   const boost::shared_ptr<msgs::WorldModify const> &_msg);

      private: QMenu *fileMenu;
      private: QMenu *editMenu;
      private: QMenu *viewMenu;
      private: QMenu *helpMenu;
      private: QToolBar *playToolbar;
      private: QToolBar *editToolbar;

      private: QAction *newAct;
      private: QAction *openAct;
      private: QAction *saveAct;
      private: QAction *aboutAct;
      private: QAction *quitAct;

      private: QAction *newModelAct;
      private: QAction *resetWorldAct;
      private: QAction *editWorldPropertiesAct;

      private: QAction *playAct;
      private: QAction *pauseAct;
      private: QAction *stepAct;

      private: QAction *boxCreateAct;
      private: QAction *sphereCreateAct;
      private: QAction *cylinderCreateAct;
      private: QAction *pointLghtCreateAct;
      private: QAction *spotLghtCreateAct;
      private: QAction *dirLghtCreateAct;

      //private: QAction *insertModelAct;
      private: QAction *viewFullScreenAct;
      private: QAction *viewFPSAct;
      private: QAction *viewOrbitAct;

      private: TimePanel *timePanel;
      private: RenderWidget *renderWidget;

      private: transport::NodePtr node;
      private: transport::PublisherPtr worldControlPub;
      private: transport::PublisherPtr serverControlPub;
      private: transport::PublisherPtr selectionPub;
      private: transport::PublisherPtr requestPub;
      private: transport::SubscriberPtr responseSub;
      private: transport::SubscriberPtr guiSub;
      private: transport::SubscriberPtr newEntitySub;
      private: transport::SubscriberPtr worldModSub;

      private: WorldPropertiesWidget *worldPropertiesWidget;
      private: QDockWidget *modelsDock;
      private: QDockWidget *insertModelsDock;

      private: std::vector<event::ConnectionPtr> connections;

      // A map that associates physics_id's with entity names
      private: std::map<std::string, unsigned int> entities;

      private: msgs::Request *requestMsg;
    };
  }
}

#endif

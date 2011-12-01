#ifndef GL_WIDGET_HH
#define GL_WIDGET_HH

#include <QtGui>

#include "rendering/RenderTypes.hh"

#include "transport/TransportTypes.hh"

#include "common/MouseEvent.hh"
#include "common/Event.hh"

#include "math/Pose.hh"

#include "msgs/msgs.h"

#include "gui/BoxMaker.hh"
#include "gui/SphereMaker.hh"
#include "gui/CylinderMaker.hh"
#include "gui/LightMaker.hh"

namespace gazebo
{
  namespace gui
  {
    class GLWidget : public QWidget
    {
      Q_OBJECT

      public: GLWidget(QWidget *parent=0);
      public: virtual ~GLWidget();

      public: void ViewScene(rendering::ScenePtr scene);
      public: rendering::UserCameraPtr GetCamera() const;
      public: rendering::ScenePtr GetScene() const;

      public: void CreateEntity(const std::string &name);

      public: void Clear();

      signals:
        void clicked();

      protected: virtual void moveEvent(QMoveEvent *e);
      protected: virtual void paintEvent(QPaintEvent *e);
      protected: virtual void resizeEvent(QResizeEvent *e);
      protected: virtual void showEvent(QShowEvent *e);


      protected: void keyPressEvent( QKeyEvent *_event);
      protected: void keyReleaseEvent( QKeyEvent *_event);
      protected: void wheelEvent(QWheelEvent *event);
      protected: void mousePressEvent(QMouseEvent *event);
      protected: void mouseMoveEvent(QMouseEvent *event);
      protected: void mouseReleaseEvent(QMouseEvent *event);

      private: std::string GetOgreHandle() const;

      private: void OnCreateScene(const std::string &_name);
      private: void OnRemoveScene(const std::string &_name);
      private: void OnMoveMode(bool mode);
      private: void OnCreateEntity( const std::string &_type );
      private: void OnFPS();
      private: void OnOrbit();

      private: void RotateEntity( rendering::VisualPtr &_vis );
      private: void TranslateEntity( rendering::VisualPtr &_vis );

      private: void OnSelectionMsg(const boost::shared_ptr<msgs::Selection const> &_msg);

      private: bool eventFilter(QObject *obj, QEvent *event);
      private: int windowId;

      private: rendering::UserCameraPtr userCamera;
      private: rendering::ScenePtr scene;
      private: QFrame *renderFrame;
      private: common::MouseEvent mouseEvent;

      private: std::vector<event::ConnectionPtr> connections;

      private: EntityMaker *entityMaker;
      private: BoxMaker boxMaker;
      private: SphereMaker sphereMaker;
      private: CylinderMaker cylinderMaker;
      private: PointLightMaker pointLightMaker;
      private: SpotLightMaker spotLightMaker;
      private: DirectionalLightMaker directionalLightMaker;

      private: rendering::VisualPtr selectionVis, hoverVis;
      private: unsigned int selectionId;
      private: std::string selectionMod;
      private: math::Pose selectionPoseOrig;

      private: transport::NodePtr node;
      private: transport::PublisherPtr modelPub;
      private: transport::SubscriberPtr selectionSub;

      private: Qt::KeyboardModifiers keyModifiers;
      private: int mouseOffset;
    };
  }
}

#endif

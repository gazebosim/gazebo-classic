#ifndef MODEL_BUILDER_WIDGET_HH
#define MODEL_BUILDER_WIDGET_HH

#include <QWidget>

#include "physics/PhysicsTypes.hh"
#include "transport/TransportTypes.hh"

namespace gazebo
{
  class GLWidget;
  namespace common
  {
    class Vector3;
  }

  namespace gui
  {
    class ModelBuilderWidget : public QWidget
    {
      Q_OBJECT
      public: ModelBuilderWidget( QWidget *parent = 0 );
      public: virtual ~ModelBuilderWidget();

      public: void Init();

      private slots: void CreateBox();
      private slots: void CreateSphere();
      private slots: void CreateCylinder();

      private: void OnBoxCreate(const common::Vector3 &pos,  
                                const common::Vector3 &scale);
 
      private: void OnSphereCreate(const common::Vector3 &pos,  
                                   const common::Vector3 &scale);
 
      private: void OnCylinderCreate(const common::Vector3 &pos,  
                                     const common::Vector3 &scale);

      private: GLWidget *glWidget; 
      private: QAction *boxCreateAct, *sphereCreateAct, *cylinderCreateAct;

      private: physics::WorldPtr world;

      protected: transport::NodePtr node;
      protected: transport::PublisherPtr factoryPub;
    };
  }
}

#endif

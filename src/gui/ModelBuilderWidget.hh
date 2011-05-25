#ifndef MODEL_BUILDER_WIDGET_HH
#define MODEL_BUILDER_WIDGET_HH

#include <QWidget>

#include "physics/PhysicsTypes.hh"

namespace gazebo
{
  class GLWidget;

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

      private: GLWidget *glWidget; 
      private: QAction *boxCreateAct, *sphereCreateAct, *cylinderCreateAct;

      private: physics::WorldPtr world;
    };
  }
}

#endif

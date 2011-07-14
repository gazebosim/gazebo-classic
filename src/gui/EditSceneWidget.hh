#ifndef EDIT_SCENE_WIDGET_HH
#define EDIT_SCENE_WIDGET_HH

#include <QWidget>
#include <QPushButton>
#include <QFrame>
#include <QCheckBox>
#include <QLineEdit>
#include <QDoubleSpinBox>

#include "physics/PhysicsTypes.hh"
#include "transport/TransportTypes.hh"
#include "math/Vector3.hh"
namespace gazebo
{
  namespace gui
  {
    class EditSceneWidget : public QWidget
    {
      Q_OBJECT
      public: EditSceneWidget( QWidget *parent = 0 );
      public: virtual ~EditSceneWidget();

      public: void Init();

      private slots: void AmbientColor();
      private slots: void BackgroundColor();
      private slots: void FogColor();
      private slots: void Shadows(bool _state=false);

      private: QPushButton *ambientColorButton;
      private: QFrame *ambientColorFrame;

      private: QPushButton *backgroundColorButton;
      private: QFrame *backgroundColorFrame;

      private: QCheckBox *shadowsButton;

      private: QPushButton *fogColorButton;
      private: QLineEdit *fogStart;
      private: QLineEdit *fogEnd;
      private: QDoubleSpinBox *fogDensitySpin;

      protected: transport::NodePtr node;

      private: transport::PublisherPtr scenePub;
    };
  }
}

#endif

#ifndef EDIT_SCENE_WIDGET_HH
#define EDIT_SCENE_WIDGET_HH

#include <QWidget>
#include <QPushButton>
#include <QFrame>
#include <QCheckBox>
#include <QLineEdit>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <QGroupBox>

#include "physics/PhysicsTypes.hh"
#include "transport/TransportTypes.hh"
#include "math/Vector3.hh"
#include "msgs/msgs.h"

namespace gazebo
{
  namespace gui
  {
    class WorldPropertiesWidget : public QWidget
    {
      Q_OBJECT
      public: WorldPropertiesWidget( QWidget *parent = 0 );
      public: virtual ~WorldPropertiesWidget();
    };

    class PhysicsWidget : public QWidget
    {
      Q_OBJECT
      public: PhysicsWidget( QWidget *parent = 0 );
      public: virtual ~PhysicsWidget();

      protected: transport::NodePtr node;
      private: transport::PublisherPtr physicsPub;
    };


    class SceneWidget : public QWidget
    {
      Q_OBJECT
      public: SceneWidget( QWidget *parent = 0 );
      public: virtual ~SceneWidget();

      public slots: void closeEvent(QCloseEvent * /*_event*/);
      private slots: void OnAmbientColor();
      private slots: void OnBackgroundColor();

      private slots: void OnFogColor();
      private slots: void OnFogStart();
      private slots: void OnFogEnd();
      private slots: void OnFogDensity(double _density);
      private slots: void OnFogType(int _index);
      private slots: void OnFogToggle(bool _value);

      private slots: void OnShadows(bool _state=false);

      private: void ReceiveSceneMsg(const boost::shared_ptr<msgs::Scene const> &_msg);

      private: QPushButton *ambientColorButton;
      private: QFrame *ambientColorFrame;

      private: QPushButton *backgroundColorButton;
      private: QFrame *backgroundColorFrame;

      private: QCheckBox *shadowsButton;

      private: QGroupBox *fogBox;
      private: QPushButton *fogColorButton;
      private: QLineEdit *fogStart;
      private: QLineEdit *fogEnd;
      private: QDoubleSpinBox *fogDensitySpin;
      private: QComboBox *fogTypeBox;

      private: transport::NodePtr node;
      private: transport::PublisherPtr scenePub, sceneRequestPub;
      private: transport::SubscriberPtr sceneSub;

      private: bool initialized;
    };
  }
}

#endif

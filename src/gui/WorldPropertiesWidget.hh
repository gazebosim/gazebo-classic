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
#ifndef EDIT_SCENE_WIDGET_HH
#define EDIT_SCENE_WIDGET_HH

#include <string>

#include "gui/qt.h"
#include "physics/PhysicsTypes.hh"
#include "transport/TransportTypes.hh"
#include "math/Vector3.hh"
#include "msgs/msgs.h"

namespace gazebo
{
  namespace msgs
  {
    class RequestMsg;
  }

  namespace gui
  {
    class SceneWidget;
    class PhysicsWidget;

    class WorldPropertiesWidget : public QWidget
    {
      Q_OBJECT
      public: WorldPropertiesWidget(QWidget *_parent = 0);
      public: virtual ~WorldPropertiesWidget();

      protected: void closeEvent(QCloseEvent *_event);
      protected: void showEvent(QShowEvent *_event);

      private: SceneWidget *sceneWidget;
      private: PhysicsWidget *physicsWidget;
    };


    class PhysicsWidget : public QWidget
    {
      Q_OBJECT
      public: PhysicsWidget(QWidget *_parent = 0);
      public: virtual ~PhysicsWidget();

      public: void Init();

      private slots: void OnSolverType(int _index);

      private slots: void OnGravity();
      private slots: void OnDt();
      private slots: void OnSOR();
      private slots: void OnIters();
      private slots: void OnCFM();
      private slots: void OnERP();
      private slots: void OnMaxVel();
      private slots: void OnSurfaceLayer();

      private: void OnResponse(ConstResponsePtr &_msg);

      private: transport::NodePtr node;
      private: transport::PublisherPtr physicsPub, requestPub;
      private: transport::SubscriberPtr responseSub;
      private: msgs::Request *requestMsg;
      private: bool initialized;

      private: QLineEdit *gravityXLineEdit;
      private: QLineEdit *gravityYLineEdit;
      private: QLineEdit *gravityZLineEdit;
      private: QComboBox *solverTypeBox;
      private: QLineEdit *dtLineEdit;
      private: QLineEdit *itersLineEdit;
      private: QLineEdit *sorLineEdit;
      private: QLineEdit *cfmLineEdit;
      private: QLineEdit *erpLineEdit;
      private: QLineEdit *maxVelLineEdit;
      private: QLineEdit *surfaceLayerLineEdit;
    };


    class SceneWidget : public QWidget
    {
      Q_OBJECT
      public: SceneWidget(QWidget *_parent = 0);
      public: virtual ~SceneWidget();
      public: void Init();

      private slots: void OnAmbientColor();
      private slots: void OnBackgroundColor();

      private slots: void OnFogColor();
      private slots: void OnFogStart();
      private slots: void OnFogEnd();
      private slots: void OnFogDensity(double _density);
      private slots: void OnFogType(int _index);
      private slots: void OnFogToggle(bool _value);

      private slots: void OnShadows(bool _state = false);

      private: void OnResponse(
                   ConstResponsePtr &_msg);

      public: bool initialized;
      private: std::string sceneName;
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
      private: transport::PublisherPtr scenePub, requestPub;
      private: transport::SubscriberPtr responseSub;
      private: msgs::Request *requestMsg;
    };
  }
}

#endif



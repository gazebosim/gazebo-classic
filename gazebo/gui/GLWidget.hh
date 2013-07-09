/*
 * Copyright 2012 Open Source Robotics Foundation
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
#ifndef _GL_WIDGET_HH_
#define _GL_WIDGET_HH_

#include <string>
#include <vector>
#include <utility>
#include <list>

#include "gazebo/gui/qt.h"
#include "gazebo/rendering/RenderTypes.hh"

#include "gazebo/transport/TransportTypes.hh"

#include "gazebo/common/MouseEvent.hh"
#include "gazebo/common/Event.hh"

#include "gazebo/math/Pose.hh"

#include "gazebo/msgs/msgs.hh"

#include "gazebo/gui/BoxMaker.hh"
#include "gazebo/gui/SphereMaker.hh"
#include "gazebo/gui/CylinderMaker.hh"
#include "gazebo/gui/MeshMaker.hh"
#include "gazebo/gui/ModelMaker.hh"
#include "gazebo/gui/LightMaker.hh"

namespace gazebo
{


  namespace gui
  {
    class GLWidget : public QWidget
    {
      Q_OBJECT

      public: GLWidget(QWidget *_parent = 0);
      public: virtual ~GLWidget();

      public: void ViewScene(rendering::ScenePtr _scene);
      public: rendering::UserCameraPtr GetCamera() const;
      public: rendering::ScenePtr GetScene() const;

      public: void Clear();

      signals: void clicked();


      protected: virtual void moveEvent(QMoveEvent *_e);
      protected: virtual void paintEvent(QPaintEvent *_e);
      protected: virtual void resizeEvent(QResizeEvent *_e);
      protected: virtual void showEvent(QShowEvent *_e);
      protected: virtual void enterEvent(QEvent * event);


      protected: void keyPressEvent(QKeyEvent *_event);
      protected: void keyReleaseEvent(QKeyEvent *_event);
      protected: void wheelEvent(QWheelEvent *_event);
      protected: void mousePressEvent(QMouseEvent *_event);
      protected: void mouseDoubleClickEvent(QMouseEvent *_event);
      protected: void mouseMoveEvent(QMouseEvent *_event);
      protected: void mouseReleaseEvent(QMouseEvent *_event);

      private: std::string GetOgreHandle() const;

      /// \brief Callback for a mouse move event.
      /// \param[in] _event The mouse move event
      /// \return True if handled by this function.
      private: bool OnMouseMove(const common::MouseEvent &_event);

      /// \brief Process a normal mouse move event.
      private: void OnMouseMoveNormal();

      /// \brief Process an object translate mouse move event.
      private: void OnMouseMoveTranslate();

      /// \brief Process a make object mouse move event.
      private: void OnMouseMoveMakeEntity();

      /// \brief Process a mouse move event in universal manipulation mode.
      private: void OnMouseMoveUniversal();

      /// \brief Callback for a mouse release event.
      /// \param[in] _event The mouse release event
      /// \return True if handled by this function.
      private: bool OnMouseRelease(const common::MouseEvent &_event);

      /// \brief Process a normal mouse release event.
      private: void OnMouseReleaseNormal();

      /// \brief Process an object translate mouse release event.
      private: void OnMouseReleaseTranslate();

      /// \brief Process a mouse release event in universal manipulation mode.
      private: void OnMouseReleaseUniversal();

      /// \brief Process a make object mouse release event.
      private: void OnMouseReleaseMakeEntity();

      /// \brief Callback for a mouse press event.
      /// \param[in] _event The mouse press event
      /// \return True if handled by this function.
      private: bool OnMousePress(const common::MouseEvent &_event);

      /// \brief Process a normal mouse press event.
      private: void OnMousePressNormal();

      /// \brief Process an object translate mouse press event.
      private: void OnMousePressTranslate();

      /// \brief Process a make object mouse presse event.
      private: void OnMousePressMakeEntity();

      /// \brief Process a mouse press event in universal manipulation mode.
      private: void OnMousePressUniversal();

      private: void OnRequest(ConstRequestPtr &_msg);

      private: void SmartMoveVisual(rendering::VisualPtr _vis);

      private: void OnCreateScene(const std::string &_name);
      private: void OnRemoveScene(const std::string &_name);
      private: void OnMoveMode(bool _mode);
      private: void OnCreateEntity(const std::string &_type,
                                   const std::string &_data);
      private: void OnFPS();
      private: void OnOrbit();
      private: void OnManipMode(const std::string &_mode);

      private: void OnSetSelectedEntity(const std::string &_name,
                                        const std::string &_mode);

      /// \brief Rotate entity.
      /// \param[in] _vis Visual representing the entity.
      /// \param[in] _local True to apply rotation in local frame.
      private: void RotateEntity(rendering::VisualPtr &_vis,
          bool _local = false);

      /// \brief Translate entity.
      /// \param[in] _vis Visual representing the entity.
      /// \param[in] _local True to apply translation in local frame.
      private: void TranslateEntity(rendering::VisualPtr &_vis,
          bool _local = false);

      /// \brief Scale entity.
      /// \param[in] _vis Visual representing the entity.
      /// \param[in] _local True to apply scaling in local frame.
      private: void ScaleEntity(rendering::VisualPtr &_vis,
          bool _local = false);


      private: void OnMouseMoveVisual(const std::string &_visualName);
      private: void OnSelectionMsg(ConstSelectionPtr &_msg);

      private: bool eventFilter(QObject *_obj, QEvent *_event);

      /// \brief Publish visual's pose to the server
      /// \param[in] _vis Pointer to the visual whose pose is to be published.
      private: void PublishVisualPose(rendering::VisualPtr _vis);

      /// \brief Publish visual's scale to the server
      /// \param[in] _vis Pointer to the visual whose scale is to be published.
      private: void PublishVisualScale(rendering::VisualPtr _vis);

      private: void ClearSelection();

      /// \brief Copy an object by name
      private: void Paste(const std::string &_object);

      private: void PushHistory(const std::string &_visName,
                                const math::Pose &_pose);
      private: void PopHistory();

      /// \brief Set the selected visual, which will highlight the
      /// visual
      private: void SetSelectedVisual(rendering::VisualPtr _vis);

      /// \brief Set the visual being moved, which will highlight the
      /// visual
      private: void SetMouseMoveVisual(rendering::VisualPtr _vis);

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
      private: MeshMaker meshMaker;
      private: ModelMaker modelMaker;
      private: PointLightMaker pointLightMaker;
      private: SpotLightMaker spotLightMaker;
      private: DirectionalLightMaker directionalLightMaker;

      private: rendering::VisualPtr hoverVis, selectedVis, mouseMoveVis;

      private: transport::NodePtr node;
      private: transport::PublisherPtr modelPub, factoryPub;
      private: transport::PublisherPtr lightPub;
      private: transport::SubscriberPtr selectionSub, requestSub;

      private: std::string keyText;
      private: Qt::KeyboardModifiers keyModifiers;
      private: QPoint onShiftMousePos;
      private: math::Pose mouseMoveVisStartPose;

      private: std::string copiedObject;

      private: std::string state;

      private: std::list<std::pair<std::string, math::Pose> > moveHistory;

      /// \brief Flag that is set to true when GLWidget has responded to
      ///  OnCreateScene
      private: bool sceneCreated;

      private: rendering::ManipulatorPtr manipulator;
    };
  }
}

#endif

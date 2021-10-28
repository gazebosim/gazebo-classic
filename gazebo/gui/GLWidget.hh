/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
#ifndef GAZEBO_GUI_GLWIDGET_HH_
#define GAZEBO_GUI_GLWIDGET_HH_

#include <memory>
#include <string>
#include <vector>

#include "gazebo/gui/qt.h"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace common
  {
    class MouseEvent;
  }

  namespace gui
  {
    class GLWidgetPrivate;

    class GZ_GUI_VISIBLE GLWidget : public QWidget
    {
      Q_OBJECT

      /// \enum SelectionLevels
      /// \brief Unique identifiers for all selection levels supported.
      public: enum SelectionLevels {
                  /// \brief Model level
                  MODEL,
                  /// \brief Link level
                  LINK
                };

      public: explicit GLWidget(QWidget *_parent = 0);
      public: virtual ~GLWidget();

      /// \brief View a scene in this widget.
      /// This will use the scene's UserCamera to visualize the scene.
      /// If a UserCamera does not exist, one is created with the
      /// name "gzclient_camera".
      /// \param[in] _scene Pointer to the scene to visualize.
      public: void ViewScene(rendering::ScenePtr _scene);

      /// \brief Return the user camera.
      /// \return User camera.
      public: rendering::UserCameraPtr Camera() const;

      /// \brief Return the scene.
      /// \return Scene.
      public: rendering::ScenePtr Scene() const;

      public: void Clear();

      /// \brief Returns the list of selected visuals.
      /// \return List with pointers to selected visuals.
      public: std::vector<rendering::VisualPtr> SelectedVisuals() const;

      /// \brief Change render rate for GLWidget
      /// \param[in] _renderRate Updated render rate
      public: void SetRenderRate(double _renderRate);

      signals: void clicked();

      /// \brief QT signal to notify when we received a selection msg.
      /// \param[in] _name Name of the selected entity.
      signals: void selectionMsgReceived(const QString &_name);

      protected: virtual void moveEvent(QMoveEvent *_e);
      protected: virtual void paintEvent(QPaintEvent *_e);
      protected: virtual void resizeEvent(QResizeEvent *_e);

      /// \brief Custom processing for the QT showEvent. Based on empirical
      /// evidence, we believe Mac needs to create the render window in this
      /// function.
      /// \param[in] _e The QT show event information.
      protected: virtual void showEvent(QShowEvent *_e);

      protected: virtual void enterEvent(QEvent *_event);


      protected: void keyPressEvent(QKeyEvent *_event);
      protected: void keyReleaseEvent(QKeyEvent *_event);
      protected: void wheelEvent(QWheelEvent *_event);
      protected: void mousePressEvent(QMouseEvent *_event);
      protected: void mouseDoubleClickEvent(QMouseEvent *_event);
      protected: void mouseMoveEvent(QMouseEvent *_event);
      protected: void mouseReleaseEvent(QMouseEvent *_event);

      /// \brief Override paintEngine to stop Qt From trying to draw on top of
      /// OGRE.
      /// \return NULL.
      protected: virtual QPaintEngine *paintEngine() const;

      private: std::string OgreHandle() const;

      /// \brief Callback for a mouse move event.
      /// \param[in] _event The mouse move event
      /// \return True if handled by this function.
      private: bool OnMouseMove(const common::MouseEvent &_event);

      /// \brief Process a normal mouse move event.
      private: void OnMouseMoveNormal();

      /// \brief Process a make object mouse move event.
      private: void OnMouseMoveMakeEntity();

      /// \brief Callback for a mouse release event.
      /// \param[in] _event The mouse release event
      /// \return True if handled by this function.
      private: bool OnMouseRelease(const common::MouseEvent &_event);

      /// \brief Process a normal mouse release event.
      private: void OnMouseReleaseNormal();

      /// \brief Process a make object mouse release event.
      private: void OnMouseReleaseMakeEntity();

      /// \brief Callback for a mouse press event.
      /// \param[in] _event The mouse press event
      /// \return True if handled by this function.
      private: bool OnMousePress(const common::MouseEvent &_event);

      /// \brief Process a normal mouse press event.
      private: void OnMousePressNormal();

      /// \brief Process a make object mouse presse event.
      private: void OnMousePressMakeEntity();

      /// \brief Callback for a mouse double click event.
      /// \param[in] _event The mouse double click event
      /// \return True if handled by this function.
      private: bool OnMouseDoubleClick(const common::MouseEvent &_event);

      private: void OnRequest(ConstRequestPtr &_msg);
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

      private: bool eventFilter(QObject *_obj, QEvent *_event);

      private: void ClearSelection();

      /// \brief Set the selected visual, which will highlight the
      /// visual
      private: void SetSelectedVisual(rendering::VisualPtr _vis);

      /// \brief Deselect all visuals, removing highlight and publishing message
      private: void DeselectAllVisuals();

      /// \brief Callback when a specific alignment configuration is set.
      /// \param[in] _axis Axis of alignment: x, y, or z.
      /// \param[in] _config Configuration: min, center, or max.
      /// \param[in] _target Target of alignment: first or last.
      /// \param[in] _preview True to preview alignment without publishing
      /// to server.
      /// \param[in] _inverted True to invert the alignment direction.
      private: void OnAlignMode(const std::string &_axis,
          const std::string &_config, const std::string &_target,
          const bool _preview, const bool _inverted = false);

      /// \brief Copy an entity by name
      /// \param[in] _name Name of entity to be copied.
      private: void Copy(const std::string &_name);

      /// \brief Paste an entity by name
      /// \param[in] _name Name of entity to be pasted.
      private: void Paste(const std::string &_name);

      /// \brief Qt callback when the copy action is triggered.
      private slots: void OnCopy();

      /// \brief Qt callback when the paste action is triggered.
      private slots: void OnPaste();

      /// \brief Qt callback when the model editor action is toggled.
      /// \param[in] _checked True if the model editor was checked.
      private slots: void OnModelEditor(bool _checked);

      /// \brief QT Callback that turns on orthographic projection
      private slots: void OnOrtho();

      /// \brief QT Callback that turns on perspective projection
      private slots: void OnPerspective();

      /// \brief Set this->mouseEvent's Buttons property to the value of
      /// _event->buttons(). Note that this is different from the
      /// SetMouseEventButtons, plural, function.
      /// \sa SetMouseEventButtons
      /// \param[in] _button The QT mouse button
      private: void SetMouseEventButton(const Qt::MouseButton &_button);

      /// \brief Set this->mouseEvent's Button property to the value of
      /// _event->button(). Note that this is different from the
      /// SetMouseEventButton, singular, function.
      /// \sa SetMouseEventButton
      /// \param[in] _button The QT mouse buttons
      private: void SetMouseEventButtons(const Qt::MouseButtons &_buttons);

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<GLWidgetPrivate> dataPtr;
    };
  }
}

#endif
